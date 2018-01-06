//
// Created by jgdo on 04.01.18.
//

#ifndef ARIPS_PARAMETER_PROXY_GROUPPROXYCLIENT_H
#define ARIPS_PARAMETER_PROXY_GROUPPROXYCLIENT_H

#include <ros/ros.h>

#include <arips_arm_msgs/ParameterValue.h>
#include <arips_arm_msgs/GetParameterGroups.h>
#include <arips_arm_msgs/GetParameterGroupDef.h>
#include <arips_arm_msgs/SetParameter.h>

#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

class ParameterProxyServer;

struct ParameterEntry {
  uint16_t param_id;
  arips_arm_msgs::ParameterDef definition;
};

struct GroupEntry {
  std::string group_name;
  uint16_t group_id;
  arips_arm_msgs::ParameterGroupDef definition;
  
  std::vector<ParameterEntry> parameterList;
  std::map<std::string, uint16_t> paramNameToId;
};

class GroupProxyClient {
public:
  GroupProxyClient(ParameterProxyServer& server, const std::string& groupName, uint16_t groupId, arips_arm_msgs::GetParameterGroupDef::Response const& clientGroupDef);

private:
  inline std::string paramPrintName(std::string const& paramName) const;
  
  void publishParameterValues();
  
  bool onSetParametersFromServer(dynamic_reconfigure::Reconfigure::Request  &req,
                                 dynamic_reconfigure::Reconfigure::Response &res);
  
  ros::NodeHandle mNh;
  
  ParameterProxyServer& mProxyServer;
  
  GroupEntry mGroupDef;
  
  ros::ServiceServer mSetParamsService;
  
  ros::Publisher mConfigDescriptionPub;
  ros::Publisher mConfigUpdatePub;
};

#endif //ARIPS_PARAMETER_PROXY_GROUPPROXYCLIENT_H
