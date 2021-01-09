#pragma once

#include <string>
#include <map>
#include <vector>
#include <memory>

namespace toponav_ros {

class CostProfileCalculator {
  class CostProfileCalculatorImpl_;
public:
  CostProfileCalculator();
  
  bool setProfile(std::string const& profile);
  
  const std::vector<std::string>& getVariables() const;
  
  double computeCosts(std::map<std::string, double> const &variables,
                        std::string const &classname) const;
  
private:
  std::shared_ptr<CostProfileCalculatorImpl_> impl_;
};
  
} // namespace topo_nav
