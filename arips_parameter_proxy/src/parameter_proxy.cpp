#include <memory>
#include <thread>

#include <ros/ros.h>
#include <ros/master.h>

#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <arips_arm_msgs/ParameterValue.h>
#include <arips_arm_msgs/GetParameter.h>
#include <arips_arm_msgs/SetParameter.h>

static const std::string clientParamUpdateTopic = "/tr_param_updates"; // needs preceeding /

template <class T>
bool hasValueChanged(T v1, T v2) {
  return v1 != v2;
}

template <>
bool hasValueChanged<double>(double v1, double v2) {
  return std::fabs(v1 - v2) > (v1 / 1000000.0);
}

class ParameterProxy {
public:
  enum State {
    WAITING_PARAMS,
    INITIALIZED,
    FINISHED,
  };
  
  ParameterProxy(const std::string& ns):
      mNamespace(ns) {
    mClientParameterValueSub = mNh.subscribe(ns + clientParamUpdateTopic, 10, &ParameterProxy::clientParameterValuesCb, this);
    mClientGetParamsClient = mNh.serviceClient<arips_arm_msgs::GetParameter>(ns + "/tr_get_param");
    mClientSetParamsServer = mNh.serviceClient<arips_arm_msgs::SetParameter>(ns + "/tr_set_param");
  }
  
  ParameterProxy(const ParameterProxy&) = delete;
  
  void startConfigure() {
    if(mState != WAITING_PARAMS) {
      return;
    }
    
    mInitThread = std::make_unique<std::thread>(&ParameterProxy::waitAndConfigClient, this);
  }

private:
  struct ParameterEntry {
    arips_arm_msgs::GetParameter::Response definition;
  };
  
  State mState = WAITING_PARAMS;
  
  std::unique_ptr<std::thread> mInitThread;
  
  ros::NodeHandle mNh;
  std::string mNamespace;
  
  ros::ServiceServer mSetParamsService;
  
  ros::Publisher mConfigDescriptionPub;
  ros::Publisher mConfigUpdatePub;
  
  ros::Subscriber mClientParameterValueSub;
  ros::ServiceClient mClientGetParamsClient;
  ros::ServiceClient mClientSetParamsServer;
  
  std::vector<ParameterEntry> mParameterList;
  std::map<std::string, size_t> mParamNameToId;
  
  void waitAndConfigClient() {
    if(mState != WAITING_PARAMS) {
      return;
    }
    
    arips_arm_msgs::GetParameter getParam;
    
    // get first param
    ros::Rate r(0.5);
    while(mParameterList.empty()) {
      getParam.request.parameter_id = 0;
      if(mClientGetParamsClient.call(getParam)) {
        if(getParam.response.num_total_parameters > 0) {
          mParameterList.resize(getParam.response.num_total_parameters);
          configParameter(getParam.request.parameter_id, getParam.response);
          
          ROS_INFO_STREAM("Client '" << mNamespace << "' has " << getParam.response.num_total_parameters << " parameters");
        } else {
          ROS_WARN_STREAM("Parameter client '" << mNamespace << "' reported 0 parameters. Ignoring.");
          mState = FINISHED;
          break;
        }
      } else {
        ROS_WARN_STREAM("Failed to call GetParameter service '" << mClientGetParamsClient.getService() << "', retrying...");
        r.sleep();
      }
    }
    
    for(size_t i = 1; i < mParameterList.size(); i++) {
      while(true) {
        getParam.request.parameter_id = i;
        if (mClientGetParamsClient.call(getParam)) {
          // TODO: check if num parameters changed
          configParameter(getParam.request.parameter_id, getParam.response);
          break;
        } else {
          ROS_WARN_STREAM("Failed to call GetParameter service '" << mClientGetParamsClient.getService() << "', retrying...");
          r.sleep();
        }
      }
    }
    
    configureClient();
  }
  
  void configureClient() {
    if(mState != WAITING_PARAMS) {
      return;
    }
    
    mSetParamsService = mNh.advertiseService(mNamespace + "/set_parameters", &ParameterProxy::onSetParametersFromServer, this);
    mConfigDescriptionPub = mNh.advertise<dynamic_reconfigure::ConfigDescription>(mNamespace + "/parameter_descriptions", 1, true);
    mConfigUpdatePub = mNh.advertise<dynamic_reconfigure::Config>(mNamespace + "/parameter_updates", 1, true);
    
    dynamic_reconfigure::ConfigDescription cd;
    cd.groups.resize(1);
    cd.groups.at(0).name = "Default";
    cd.groups.at(0).type = "";
    cd.groups.at(0).id = 0;
    cd.groups.at(0).parent = 0;
    
    
    
    cd.groups.at(0).parameters.resize(mParameterList.size());
    for(size_t i = 0; i < mParameterList.size(); i++) {
      auto& clientDef = mParameterList.at(i).definition;
      auto& serverDef = cd.groups.at(0).parameters.at(i);
      
      serverDef.name = clientDef.name;
      serverDef.level = 0;
      serverDef.description = ""; // TODO
      serverDef.edit_method = ""; // TODO
      if(clientDef.type.type == arips_arm_msgs::ParameterType::TYPE_INT) {
        serverDef.type = "int";
        
        cd.max.ints.emplace_back();
        cd.max.ints.back().name = serverDef.name;
        cd.max.ints.back().value = clientDef.max_int;
        
        cd.min.ints.emplace_back();
        cd.min.ints.back().name = serverDef.name;
        cd.min.ints.back().value = clientDef.min_int;
        
        cd.dflt.ints.emplace_back();
        cd.dflt.ints.back().name = serverDef.name;
        cd.dflt.ints.back().value = clientDef.dfl_int;
      } else if(clientDef.type.type == arips_arm_msgs::ParameterType::TYPE_DOUBLE) {
        serverDef.type = "double";
        
        cd.max.doubles.emplace_back();
        cd.max.doubles.back().name = serverDef.name;
        cd.max.doubles.back().value = clientDef.max_double;
        
        cd.min.doubles.emplace_back();
        cd.min.doubles.back().name = serverDef.name;
        cd.min.doubles.back().value = clientDef.min_double;
        
        cd.dflt.doubles.emplace_back();
        cd.dflt.doubles.back().name = serverDef.name;
        cd.dflt.doubles.back().value = clientDef.dfl_double;
      } else {
        ROS_WARN_STREAM("Unknown parameter type # of '" << mNamespace << "[" << clientDef.name << "]': " << clientDef.type.type);
      }
    }
    
    cd.max.groups.resize(1);
    cd.max.groups.at(0).name = "Default";
    cd.max.groups.at(0).state = true;
    cd.max.groups.at(0).id = 0;
    cd.max.groups.at(0).parent = 0;
    
    cd.min.groups.resize(1);
    cd.min.groups.at(0).name = "Default";
    cd.min.groups.at(0).state = true;
    cd.min.groups.at(0).id = 0;
    cd.min.groups.at(0).parent = 0;
    
    cd.dflt.groups.resize(1);
    cd.dflt.groups.at(0).name = "Default";
    cd.dflt.groups.at(0).state = true;
    cd.dflt.groups.at(0).id = 0;
    cd.dflt.groups.at(0).parent = 0;
    
    mConfigDescriptionPub.publish(cd);
    
    publishParameterValues();
    
    mState = INITIALIZED;
    
    ROS_INFO_STREAM("Configured client '" << mNamespace << "'");
  }
  
  void configParameter(size_t id, arips_arm_msgs::GetParameter::Response const& res)  {
    mParameterList.at(id).definition = res;
    mParamNameToId[res.name] = id;
  }
  
  void publishParameterValues() {
    dynamic_reconfigure::Config config;
    
    for(size_t i = 0; i < mParameterList.size(); i++) {
      auto& entry = mParameterList.at(i);
      
      if(entry.definition.type.type == arips_arm_msgs::ParameterType::TYPE_INT) {
        config.ints.emplace_back();
        config.ints.back().name = entry.definition.name;
        config.ints.back().value = entry.definition.int_value;
      } else if(entry.definition.type.type == arips_arm_msgs::ParameterType::TYPE_DOUBLE) {
        config.doubles.emplace_back();
        config.doubles.back().name = entry.definition.name;
        config.doubles.back().value = entry.definition.double_value;
      } else {
        ROS_WARN_STREAM("Unknown parameter type # of '" << mNamespace << "[" << entry.definition.name << "]': " << entry.definition.type.type);
      }
    }
    
    config.groups.resize(1);
    config.groups.at(0).name = "Default";
    config.groups.at(0).state = true;
    config.groups.at(0).id = 0;
    config.groups.at(0).parent = 0;
    
    mConfigUpdatePub.publish(config);
  }
  
  void clientParameterValuesCb(const arips_arm_msgs::ParameterValue& param) {
    if(mState == INITIALIZED) {
      auto& def = mParameterList.at(param.id).definition;
      if(def.type.type != param.type.type) {
        ROS_WARN_STREAM("Parameter type of '" << mNamespace << "[" << def.name << "]' changed on value update, ignoring.");
      } else {
        if(def.type.type == arips_arm_msgs::ParameterType::TYPE_INT) {
          def.int_value = param.int_value;
        } else if(def.type.type == arips_arm_msgs::ParameterType::TYPE_DOUBLE) {
          def.double_value = param.double_value;
        }
        // else ignore, warning was printed already
      }
    }
    
    publishParameterValues();
  }
  
  bool onSetParametersFromServer(dynamic_reconfigure::Reconfigure::Request  &req,
                                 dynamic_reconfigure::Reconfigure::Response &res)
  {
    res.config = req.config;
  
    for(auto const& int_value: req.config.ints) {
      size_t id = mParamNameToId.at(int_value.name);
      auto& def = mParameterList.at(id).definition;
      if(def.type.type == arips_arm_msgs::ParameterType::TYPE_INT) {
        if(hasValueChanged(def.int_value, int_value.value)) {
          def.int_value = int_value.value;
          
          arips_arm_msgs::SetParameter setParam;
          setParam.request.id = id;
          setParam.request.int_value = def.int_value;
          if(!mClientSetParamsServer.call(setParam)) {
            ROS_WARN_STREAM("Failed to call tr_set_param service for parameter '" << mNamespace << "[" << def.name << "]'");
          }
          // TODO check response
        }
        // else ignore changes
      } else {
        ROS_WARN_STREAM("Received wrong parameter type of '" << mNamespace << "[" << def.name << "]' from dynamic_reconfigure, ignoring.");
      }
    }
    
    for(auto const& double_value: req.config.doubles) {
      size_t id = mParamNameToId.at(double_value.name);
      auto& def = mParameterList.at(id).definition;
      if(def.type.type == arips_arm_msgs::ParameterType::TYPE_DOUBLE) {
        if(hasValueChanged(def.double_value, double_value.value)) {
          def.double_value = double_value.value;
    
          arips_arm_msgs::SetParameter setParam;
          setParam.request.id = id;
          setParam.request.int_value = def.double_value;
          if(!mClientSetParamsServer.call(setParam)) {
            ROS_WARN_STREAM("Failed to call tr_set_param service for parameter '" << mNamespace << "[" << def.name << "]'");
          }
          
          // TODO check response
        }
        // else ignore changes
      } else {
        ROS_WARN_STREAM("Received wrong parameter type of '" << mNamespace << "[" << def.name << "]' from dynamic_reconfigure, ignoring.");
      }
    }
    
    // TODO: check min/max range, if parameters actually exist, ...
    return true;
  }
};

inline bool ends_with(std::string const & value, std::string const & ending)
{
  if (ending.size() > value.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "parameter_proxy");
  ros::NodeHandle nh;
  
  std::map<std::string, ParameterProxy> allProxies;
  
  ros::Rate r(0.5);
  while (ros::ok())
  {
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    
    for(auto const& info: topic_infos) {
      if(info.datatype == "arips_arm_msgs/ParameterValue" && ends_with(info.name, clientParamUpdateTopic)) {
        std::string ns = info.name.substr(0, info.name.size() - clientParamUpdateTopic.size());
        
        if(!ns.empty()) {
          if(allProxies.find(ns) == allProxies.end()) {
            allProxies.emplace(std::piecewise_construct, std::forward_as_tuple(ns), forward_as_tuple(ns)).first->second.startConfigure();
            ROS_INFO_STREAM("Discovered new parameter client '" << ns << "'");
          }
        } else {
          ROS_WARN_STREAM("Client param update topic '" << info.name << "' has empty namespace. Ignoring.");
        }
      }
    }
    
    ros::spinOnce();
    r.sleep();
  }
}
