#include <toponav_ros/utils/CostProfileCalculator.h>

#include <ros/ros.h>

#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/variant/recursive_variant.hpp>
#include <boost/variant/apply_visitor.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_function.hpp>
#include <toponav_ros/utils/CommonCostProfiles.h>

class toponav_ros::CostProfileCalculator::CostProfileCalculatorImpl_ {
public:
  typedef std::map<std::string, double> CostsMap;
  
  struct BinaryOp;
  struct nil {};
  
  struct ExpressionAST
  {
    typedef
    boost::variant<
        nil // can't happen!
        , double
        , std::string
        , boost::recursive_wrapper<ExpressionAST>
        , boost::recursive_wrapper<BinaryOp> >
        type;
    
    ExpressionAST()
        : expr(nil()) {}
    
    template <typename Expr>
    ExpressionAST(Expr const& expr)
        : expr(expr) {}
    
    ExpressionAST& operator+=(ExpressionAST const& rhs) {
      expr = BinaryOp('+', expr, rhs);
      return *this;
    }
    
    ExpressionAST& operator*=(ExpressionAST const& rhs) {
      expr = BinaryOp('*', expr, rhs);
      return *this;
    }
    
    type expr;
    bool is_var_ = false;
  };
  
  struct BinaryOp
  {
    BinaryOp(
        char op
        , ExpressionAST const& left
        , ExpressionAST const& right)
        : op(op), left(left), right(right) {}
    
    char op;
    ExpressionAST left;
    ExpressionAST right;
  };
  
  template <typename Iterator>
  struct ProfileGrammar : boost::spirit::qi::grammar<Iterator, ExpressionAST(), boost::spirit::ascii::space_type>
  {
    ProfileGrammar() : ProfileGrammar::base_type(expression)
    {
      namespace qi = boost::spirit::qi;
      namespace ascii = boost::spirit::ascii;
      
      using qi::_val;
      using qi::_1;
      using qi::lexeme;
      
      var_name %= lexeme[(qi::alpha | '_') >> *(qi::alnum | '_')];
      
      expression =
          summand                            [_val = _1]
              >> *('+' >> summand            [_val += _1])
          ;
  
      summand = u_double [_val = _1] >> -('*' >> var_name [_val *= _1])
             | var_name [_val = _1] >> -('*' >> u_double [_val *= _1])
          ;

    }
    
    boost::spirit::qi::rule<Iterator, ExpressionAST(), boost::spirit::ascii::space_type> expression, summand;
    boost::spirit::qi::rule<Iterator, std::string(), boost::spirit::ascii::space_type> var_name;
    boost::spirit::qi::real_parser<double, boost::spirit::qi::ureal_policies<double>> u_double;
  };
  
  typedef std::string::const_iterator iterator_type;
  typedef ProfileGrammar<iterator_type> Grammar;
  
  ExpressionAST current_ast_;
  std::vector<std::string> variable_names_;
  Grammar grammar_;
  
  CostProfileCalculatorImpl_(): current_ast_(CommonCostProfiles::DEFAULT_PROFILE), variable_names_ {CommonCostProfiles::DEFAULT_PROFILE} {
  }
  
  bool setProfile(std::string const& profile) {
    using boost::spirit::ascii::space;
    
    std::string::const_iterator iter = profile.begin();
    std::string::const_iterator end = profile.end();
    ExpressionAST ast;
    
    bool r = phrase_parse(iter, end, grammar_, space, ast);
    
    if (r && iter == end && checkConsistency(ast))
    {
      current_ast_ = ast;
      return true;
    } else {
      std::string rest(iter, end);
      ROS_ERROR_STREAM("Failed to set topo planner profile: parse error at '"<< rest << "'. resetting to " << CommonCostProfiles::DEFAULT_PROFILE);
      
      current_ast_ = CommonCostProfiles::DEFAULT_PROFILE;
      variable_names_ = {CommonCostProfiles::DEFAULT_PROFILE};
      
      return false;
    }
  }
  
  double computeValue(CostsMap const& costs, std::string const& classname) const {
    struct
    {
      typedef double result_type;
      
      result_type operator()(nil) const { return 0.0;}
      result_type operator()(double n) const { return n;}
      
      result_type operator()(ExpressionAST const & ast) const
      {
        return boost::apply_visitor(*this, ast.expr);
      }
      
      result_type operator()(BinaryOp const& expr) const
      {
        result_type v1 = boost::apply_visitor(*this, expr.left.expr);
        result_type v2 = boost::apply_visitor(*this, expr.right.expr);
        
        if(expr.op == '*') {
          return v1 * v2;
        } else if(expr.op == '+') {
          return v1 + v2;
        } else {
          throw std::runtime_error(std::string("CostProfileCalculatorImpl_: internal error - operator is ") + expr.op);
        }
      }
      
      result_type operator()(std::string const& expr) const
      {
        auto iter = costs_.find(expr);
        if(iter==costs_.end()) {
          throw std::runtime_error(expr);
        }
        
        return iter->second;
      }
  
      const CostsMap& costs_;
    } visitor {costs};
    
    try {
      return visitor(current_ast_);
    } catch(std::runtime_error const& ex) {
      ROS_WARN_STREAM("No variable named '" << ex.what() << "' in available costs profile of module '" << classname << "' present. Returning default.");
      return costs.at(CommonCostProfiles::DEFAULT_PROFILE);
    }
  }
  
  bool checkConsistency(ExpressionAST& ast) {
    struct
    {
      typedef bool result_type;
      
      bool operator()(nil) { return false;}
      bool operator()(double n) { return false;}
      
      bool operator()(ExpressionAST & ast)
      {
        return (ast.is_var_ = boost::apply_visitor(*this, ast.expr));
      }
      
      bool operator()(BinaryOp& expr)
      {
        bool v1 = boost::apply_visitor(*this, expr.left.expr);
        bool v2 = boost::apply_visitor(*this, expr.right.expr);
        
        if(expr.op == '*' && (v1 && v2)) {
          throw std::runtime_error("multiplyer");
        }
        
        return v1 || v2;
      }
      
      bool operator()(std::string const& expr)
      {
        costs_.emplace_back(expr);
        return true;
      }
  
      std::vector<std::string>& costs_;
    } visitor {variable_names_};
  
    variable_names_.clear();
    
    try {
      visitor(ast);
    } catch(std::runtime_error const&) {
      return false;
    }
    
    return true;
  }
};

toponav_ros::CostProfileCalculator::CostProfileCalculator(): impl_(std::make_shared<CostProfileCalculatorImpl_>()) {
}


bool toponav_ros::CostProfileCalculator::setProfile(const std::string &profile) {
  return impl_->setProfile(profile);
}

const std::vector<std::string> &toponav_ros::CostProfileCalculator::getVariables() const {
  return impl_->variable_names_;
}

double toponav_ros::CostProfileCalculator::computeCosts(std::map<std::string, double> const &variables,
                                                        std::string const &classname) const {
  return impl_->computeValue(variables, classname);
}

