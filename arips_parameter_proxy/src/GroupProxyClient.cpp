//
// Created by jgdo on 04.01.18.
//

#include "GroupProxyClient.h"
#include "ParameterProxyServer.h"

GroupProxyClient::GroupProxyClient(ParameterProxyServer &server, const std::string &groupName, uint16_t groupId,
                                   arips_arm_msgs::GetParameterGroupDef::Response const &clientGroupDef):
  mProxyServer(server)
{
  ROS_INFO_STREAM("Configuring group " << server.getNamespace() << "/" << groupName);
  
  mSetParamsService = mNh.advertiseService(server.getNamespace() + "/" + groupName + "/set_parameters", &GroupProxyClient::onSetParametersFromServer, this);
  mConfigDescriptionPub = mNh.advertise<dynamic_reconfigure::ConfigDescription>(server.getNamespace() + "/" + groupName + "/parameter_descriptions", 10, true);
  mConfigUpdatePub = mNh.advertise<dynamic_reconfigure::Config>(server.getNamespace() + "/" + groupName + "/parameter_updates", 10, true);
  
  mGroupDef.group_id = groupId;
  mGroupDef.group_name = groupName;
  mGroupDef.definition.groupname = groupName;
  mGroupDef.parameterList.resize(clientGroupDef.param_defs.size());
  
  dynamic_reconfigure::ConfigDescription cd;
  cd.groups.resize(1);
  cd.groups.at(0).name = "Default";
  cd.groups.at(0).type = "";
  cd.groups.at(0).id = 0;
  cd.groups.at(0).parent = 0;
  
  cd.groups.at(0).parameters.resize(clientGroupDef.param_defs.size());
  
  for(size_t i = 0; i < mGroupDef.parameterList.size(); i++) {
    auto& clientDef = clientGroupDef.param_defs.at(i);
    auto& entryDef = mGroupDef.parameterList.at(i);
    auto& serverDef = cd.groups.at(0).parameters.at(i);
  
    entryDef.param_id = i;
    entryDef.definition = clientDef;
    mGroupDef.paramNameToId[clientDef.name] = entryDef.param_id;
    
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
      ROS_WARN_STREAM("Unknown parameter type # of '" << mProxyServer.getNamespace() << "/" << mGroupDef.group_name << "[" << clientDef.name << "]': " << clientDef.type.type);
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
  
}


template <class T>
bool hasValueChanged(T v1, T v2) {
  return v1 != v2;
}

template <>
bool hasValueChanged<double>(double v1, double v2) {
  return std::fabs(v1 - v2) > (v1 / 1000000.0);
}


bool GroupProxyClient::onSetParametersFromServer(dynamic_reconfigure::Reconfigure::Request &req,
                                                 dynamic_reconfigure::Reconfigure::Response &res)
{
  res.config = req.config;
  
  ROS_INFO_STREAM("Called onSetParametersFromServer() from " << mProxyServer.getNamespace());
  
  arips_arm_msgs::SetParameter setParam;
  
  for(auto const& int_value: req.config.ints) {
    size_t id = mGroupDef.paramNameToId.at(int_value.name);
    auto& def = mGroupDef.parameterList.at(id).definition;
    if(def.type.type == arips_arm_msgs::ParameterType::TYPE_INT) {
      if(hasValueChanged(def.current_int_value, int_value.value)) {
        def.current_int_value = int_value.value;
        
        setParam.request.parameters.emplace_back();
        auto& param = setParam.request.parameters.back();
        param.parameter_id = id;
        param.group_id = mGroupDef.group_id;
        param.int_value = def.current_int_value;
      }
      // else ignore changes
    } else {
      ROS_WARN_STREAM("Received wrong parameter type of " << paramPrintName(def.name) << " from dynamic_reconfigure, ignoring.");
    }
  }
  
  for(auto const& double_value: req.config.doubles) {
    size_t id = mGroupDef.paramNameToId.at(double_value.name);
    auto& def = mGroupDef.parameterList.at(id).definition;
    if(def.type.type == arips_arm_msgs::ParameterType::TYPE_DOUBLE) {
      if(hasValueChanged(def.current_double_value, double_value.value)) {
        def.current_double_value = double_value.value;
  
        setParam.request.parameters.emplace_back();
        auto& param = setParam.request.parameters.back();
        param.parameter_id = id;
        param.group_id = mGroupDef.group_id;
        param.double_value = def.current_int_value;
      }
      // else ignore changes
    } else {
      ROS_WARN_STREAM("Received wrong parameter type of " << paramPrintName(def.name) << " from dynamic_reconfigure, ignoring.");
    }
  }
  
  if(!mProxyServer.getClientSetParamService().call(setParam)) {
    ROS_WARN_STREAM("Failed to call tr_set_param service for group '" << mProxyServer.getNamespace() << "/" << mGroupDef.group_name << "'");
  }
  // TODO check response
  
  ROS_INFO_STREAM("onSetParametersFromServer() done");
  // TODO: check min/max range, if parameters actually exist, ...
  
  return true;
}

void GroupProxyClient::publishParameterValues()
{
  dynamic_reconfigure::Config config;
  
  for(size_t i = 0; i < mGroupDef.parameterList.size(); i++) {
    auto& entry = mGroupDef.parameterList.at(i);
    
    if(entry.definition.type.type == arips_arm_msgs::ParameterType::TYPE_INT) {
      config.ints.emplace_back();
      config.ints.back().name = entry.definition.name;
      config.ints.back().value = entry.definition.current_int_value;
    } else if(entry.definition.type.type == arips_arm_msgs::ParameterType::TYPE_DOUBLE) {
      config.doubles.emplace_back();
      config.doubles.back().name = entry.definition.name;
      config.doubles.back().value = entry.definition.current_double_value;
    } else {
      ROS_WARN_STREAM("Unknown parameter type # of '" << mProxyServer.getNamespace() << "/" << mGroupDef.group_name << "[" << entry.definition.name << "]': " << entry.definition.type.type);
    }
  }
  
  config.groups.resize(1);
  config.groups.at(0).name = "Default";
  config.groups.at(0).state = true;
  config.groups.at(0).id = 0;
  config.groups.at(0).parent = 0;
  
  mConfigUpdatePub.publish(config);
}

std::string GroupProxyClient::paramPrintName(std::string const& paramName) const
{
  return mProxyServer.getNamespace() + "/" + mGroupDef.group_name + "[" + paramName + "]";
}
