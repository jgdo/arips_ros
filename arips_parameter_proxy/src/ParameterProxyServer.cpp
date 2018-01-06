//
// Created by jgdo on 04.01.18.
//

#include "ParameterProxyServer.h"

void ParameterProxyServer::configureGroups(size_t id, arips_arm_msgs::ParameterGroupDef const &groupDef) {
  arips_arm_msgs::GetParameterGroupDef groupParamsDef;
  groupParamsDef.request.group_id = id;
  
  ros::Rate r(0.5);
  while(true) {
    if(mClientGetGroupDefClient.call(groupParamsDef)) {
      if(groupParamsDef.response.param_defs.size() > 0) {
        mGroupsList.emplace_back(std::make_unique<GroupProxyClient>(*this, groupDef.groupname, id, groupParamsDef.response));
        
        ROS_INFO_STREAM("Client group'" << mNamespace << "/" << groupDef.groupname << "' has " << groupParamsDef.response.param_defs.size() << " parameters");
        break;
      } else {
        ROS_WARN_STREAM("Parameter client '" << mNamespace << "' reported 0 parameters in group '" << groupDef.groupname << "', ignoring.");
        mState = FINISHED;
        break;
      }
    } else {
      ROS_WARN_STREAM("Failed to call GetParameter service '" << mClientGetGroupDefClient.getService() << "', retrying...");
      r.sleep();
    }
  }
}
