#pragma once

#include <boost/lexical_cast.hpp>
#include <type_traits>
#include <tf/transform_datatypes.h>

#include <yaml-cpp/yaml.h>

// See YAML tutorial on https://github.com/jbeder/yaml-cpp/wiki/Tutorial
namespace YAML {
template<>
struct convert<tf2::Vector3> {
  static Node encode(const tf2::Vector3& rhs) {
    Node node;

#if 0
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
#else
    node["x"] = rhs.x();
    node["y"] = rhs.y();
    node["z"] = rhs.z();
#endif
    return node;
  }
  
  static bool decode(const Node& node, tf2::Vector3& rhs) {
    if(node.size() != 3) {
      return false;
    }
    
    if(node.IsMap()) {
      rhs.setX(node["x"].as<double>());
      rhs.setY(node["y"].as<double>());
      rhs.setZ(node["z"].as<double>());
    } else if(node.IsSequence()) {
      rhs.setX(node[0].as<double>());
      rhs.setY(node[1].as<double>());
      rhs.setZ(node[2].as<double>());
    } else
      return false;
    
    return true;
  }
};

template<>
struct convert<tf2::Quaternion> {
  static Node encode(const tf2::Quaternion& rhs) {
    Node node;
#if 0
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    node.push_back(rhs.w());
#else
    node["x"] = rhs.x();
    node["y"] = rhs.y();
    node["z"] = rhs.z();
    node["w"] = rhs.w();
#endif
    return node;
  }
  
  static bool decode(const Node& node, tf2::Quaternion& rhs) {
    if(node.size() != 4) {
      return false;
    }
  
    if(node.IsMap()) {
      rhs.setX(node["x"].as<double>());
      rhs.setY(node["y"].as<double>());
      rhs.setZ(node["z"].as<double>());
      rhs.setW(node["w"].as<double>());
    } else if(node.IsSequence()) {
      rhs.setX(node[0].as<double>());
      rhs.setY(node[1].as<double>());
      rhs.setZ(node[2].as<double>());
      rhs.setW(node[3].as<double>());
    } else
      return false;
  
    return true;
  }
};

template<>
struct convert<tf2::Stamped<tf2::Transform>> {
  static Node encode(const tf2::Stamped<tf2::Transform>& rhs) {
    Node node;
    node["frame_id"] = rhs.frame_id_;
    node["position"] = rhs.getOrigin();
    node["orientation"] = rhs.getRotation();
    return node;
  }
  
  // decoding though YamlTools::parsePose() since special treatment required
  /*
  static bool decode(const Node& node, tf2::Stamped<tf2::Transform>& rhs) {
    if(!node.IsMap() || node.size() != 3) {
      return false;
    }
    
    rhs.frame_id_ = node["frame_id"].as<std::string>();
    rhs.setOrigin(node["position"].as<tf2::Vector3>());
    rhs.setRotation(node["orientation"].as<tf2::Quaternion>());
    return true;
  } */
};

} // namespace YAML

namespace toponav_ros {

namespace YamlTools {
/**
 * Assume that param is a frame id. Throw exception if emtpy string
 * @param param
 * @param errorMessage
 * @return
 */
inline static std::string parseFrameID(YAML::Node const& param, const std::string& errorMessage) {
  std::string frame_id = param.as<std::string>();
  if(frame_id.empty())
    throw std::runtime_error(errorMessage);

  return frame_id;
}

inline static tf2::Stamped<tf2::Transform> parsePose(YAML::Node const &param, std::string const &errorMessage) {
  tf2::Stamped<tf2::Transform> pose;
  pose.frame_id_ = parseFrameID(param["frame_id"], errorMessage);
  pose.setOrigin(param["position"].as<tf2::Vector3>());
  pose.setRotation(param["orientation"].as<tf2::Quaternion>());
  return pose;
}

inline static void readDefaultCosts(YAML::Node const &config, std::string const &className,
                                    std::string const &costsName, double *value) {
  if(config.IsScalar()) {
    *value = config.as<double>();
  } else {
    ROS_WARN_STREAM(className << " error: no default " << costsName << " defined in topo map YAML file, using default value = " << *value);
  }
}

inline static void readCosts(YAML::Node const &config, std::string const &className,
                             std::string const &costsName, double *value, double defaultValue) {
  if(!config || config.IsNull()) {
    *value = defaultValue;
  } else if(config.IsScalar()) {
    *value = config.as<double>();
    // } else if (config.getType() ==  YAML::Node const::TypeString && config == "default") { // TODO
    //    *value = defaultValue;
  } else {
    ROS_WARN_STREAM(className << " error: ony a number or 'default' for " << costsName << " accepted. Using default value = " << *value);
  }
}

template<class T>
struct CostsParserBase {
  typedef double T::*Member;
  typedef std::map<std::string, Member> MemberMap;
  
  static void parse_(MemberMap const& m, T* t, YAML::Node const & node, std::string const& className, std::string const& costsName) {
    if(!node || node.IsNull()) {
      // skip
    } else if(node.IsMap()) {
      for(auto& e: node) {
        auto iter = m.find(e.first.as<std::string>());
        if(iter == m.end()) {
          ROS_WARN_STREAM(className << "error: skipping YAML cost entry '" << e.first.as<std::string>() << "' for " << costsName << ", since unknown.");
          continue;
        }
        
        t->*(iter->second) = e.second.as<double>();
      }
    } else {
      ROS_WARN_STREAM(className << " YAML error: invalid format for costs '" << costsName << "'. Using default values.");
    }
  }
  
  static YAML::Node storeAll_(MemberMap const& m, const T* t) {
    YAML::Node node;
    
    for(auto& e: m) {
      node[e.first] = t->*(e.second);
    }
    
    return node;
  }
  
  static void storeDiff_(MemberMap const& m, YAML::Node& parent_node, std::string const& name, const T* current, const T* default_) {
    YAML::Node node;
    
    for(auto& e: m) {
      if(current->*(e.second) != default_->*(e.second)) {
        node[e.first] = current->*(e.second);
      }
    }
    
    if(node.size()) {
      parent_node[name] = node;
    }
  }
};

#define GEN_COSTS_READER_WRITER(name, container_type, ...) struct name: public toponav_ros::YamlTools::CostsParserBase<container_type> { \
MemberMap member_map_; \
name(): member_map_{__VA_ARGS__} {} \
inline void parse(container_type* var, YAML::Node const & node, std::string const& className, std::string const& costsName) { \
parse_(member_map_, var, node, className, costsName); \
} \
inline YAML::Node storeAll(container_type const& var) { \
return storeAll_(member_map_, &var);\
} \
inline void storeDiff(YAML::Node& parent_node, std::string const& name, container_type const& current, container_type const& default_) { \
return storeDiff_(member_map_, parent_node, name, &current, &default_);\
} \
};

}


}
