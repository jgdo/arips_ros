//
// Created by jgdo on 04.01.18.
//

#ifndef ARIPS_PARAMETER_PROXY_PARAMETERPROXYSERVER_H
#define ARIPS_PARAMETER_PROXY_PARAMETERPROXYSERVER_H

#include <thread>
#include <memory>

#include <ros/ros.h>

#include <arips_arm_msgs/ParameterValue.h>
#include <arips_arm_msgs/GetParameterGroups.h>
#include <arips_arm_msgs/GetParameterGroupDef.h>
#include <arips_arm_msgs/SetParameter.h>

#include "GroupProxyClient.h"

static const std::string clientParamUpdateTopic = "/tr_param_updates"; // needs preceeding '/'

class ParameterProxyServer {
public:
  enum State {
    WAITING_PARAMS,
    INITIALIZED,
    FINISHED,
  };
  
  ParameterProxyServer(const std::string& ns):
      mNamespace(ns) {
    mClientParameterValueSub = mNh.subscribe(ns + clientParamUpdateTopic, 10, &ParameterProxyServer::clientParameterValuesCb, this);
    mClientGetGroupsClient = mNh.serviceClient<arips_arm_msgs::GetParameterGroups>(ns + "/tr_get_groups");
    mClientGetGroupDefClient = mNh.serviceClient<arips_arm_msgs::GetParameterGroupDef>(ns + "/tr_get_group_def");
    mClientSetParamsService = mNh.serviceClient<arips_arm_msgs::SetParameter>(ns + "/tr_set_params");
  }
  
  ParameterProxyServer(const ParameterProxyServer&) = delete;
  
  void startConfigure() {
    if(mState != WAITING_PARAMS) {
      return;
    }
    
    mInitThread = std::make_unique<std::thread>(&ParameterProxyServer::waitAndConfigClient, this);
  }
  
  inline std::string const& getNamespace() const {
    return mNamespace;
  }
  
  inline ros::ServiceClient& getClientSetParamService() {
    return mClientSetParamsService;
  }

private:
  State mState = WAITING_PARAMS;
  
  std::unique_ptr<std::thread> mInitThread;
  
  ros::NodeHandle mNh;
  std::string mNamespace;
  
  
  ros::Subscriber mClientParameterValueSub;
  ros::ServiceClient mClientGetGroupsClient;
  ros::ServiceClient mClientGetGroupDefClient;
  ros::ServiceClient mClientSetParamsService;
  
  std::vector<std::unique_ptr<GroupProxyClient>> mGroupsList;
  std::map<std::string, uint16_t> mGroupNameToId;
  
  void waitAndConfigClient() {
    if(mState != WAITING_PARAMS) {
      return;
    }
    
    arips_arm_msgs::GetParameterGroups getGroups;
    
    ros::Rate r(0.5);
    while(mGroupsList.empty()) {
      if(mClientGetGroupsClient.call(getGroups)) {
        if(getGroups.response.groups.size() > 0) {
          for(size_t i = 0; i < getGroups.response.groups.size(); i++) {
            configureGroups(i, getGroups.response.groups.at(i));
          }
          
          ROS_INFO_STREAM("Client '" << mNamespace << "' has " << getGroups.response.groups.size() << " groups");
        } else {
          ROS_WARN_STREAM("Parameter client '" << mNamespace << "' reported 0 parameter groups, ignoring.");
          mState = FINISHED;
          break;
        }
      } else {
        ROS_WARN_STREAM("Failed to call GetParameter service '" << mClientGetGroupsClient.getService() << "', retrying...");
        r.sleep();
      }
    }
    
    configureServer();
  }
  
  void configureGroups(size_t id, arips_arm_msgs::ParameterGroupDef const& groupDef);
  
  void configureServer() {
    if(mState != WAITING_PARAMS) {
      return;
    }
    
    /*
    
     */
    
    mState = INITIALIZED;
    
    ROS_INFO_STREAM("Configured client '" << mNamespace << "'");
  }
  
  void clientParameterValuesCb(const arips_arm_msgs::ParameterValue& param) {
    if(mState == INITIALIZED) {
      if (param.group_id >= mGroupsList.size()) {
        ROS_WARN_STREAM("Could not update parameter '" << mNamespace << "[group_id = " << param.group_id
                                                       << "]' since group does not exist. Ignoring.");
      } else {
        mGroupsList.at(param.group_id)->updateParameter(param);
      }
    }
  }
  
};




#endif //ARIPS_PARAMETER_PROXY_PARAMETERPROXYSERVER_H
