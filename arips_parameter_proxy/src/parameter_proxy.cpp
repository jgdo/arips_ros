#include <memory>

#include <ros/ros.h>
#include <ros/master.h>

#include <arips_arm_msgs/ParameterValue.h>
#include <arips_arm_msgs/GetParameterGroups.h>
#include <arips_arm_msgs/GetParameterGroupDef.h>
#include <arips_arm_msgs/SetParameter.h>

#include "ParameterProxyServer.h"

inline bool ends_with(std::string const & value, std::string const & ending)
{
  if (ending.size() > value.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "parameter_proxy");
  ros::NodeHandle nh;
  
  std::map<std::string, ParameterProxyServer> allProxies;
  
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
