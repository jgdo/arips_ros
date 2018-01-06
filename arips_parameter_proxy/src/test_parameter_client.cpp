#include "ros/ros.h"
#include "arips_arm_msgs/GetParameterGroups.h"
#include <arips_arm_msgs/GetParameterGroupDef.h>
#include <arips_arm_msgs/SetParameter.h>
#include <arips_arm_msgs/ParameterValue.h>

int int_param = 0;
double double_param = 0.0;

bool getParamGroupDefCb(arips_arm_msgs::GetParameterGroupDef::Request &req,
                arips_arm_msgs::GetParameterGroupDef::Response &res)
{
  if(req.group_id == 0) {
    res.param_defs.resize(2);
    
    res.param_defs.at(0).name = "int_param_0";
    res.param_defs.at(0).type.type = arips_arm_msgs::ParameterType::TYPE_INT;
    res.param_defs.at(0).current_int_value = int_param;
    res.param_defs.at(0).max_int = 100;
    res.param_defs.at(0).min_int = 0;
    res.param_defs.at(0).dfl_int = 42;
  
    res.param_defs.at(1).name = "double_param_1";
    res.param_defs.at(1).type.type = arips_arm_msgs::ParameterType::TYPE_DOUBLE;
    res.param_defs.at(1).current_double_value = double_param;
    res.param_defs.at(1).max_double = 1.0;
    res.param_defs.at(1).min_double = 0.0;
    res.param_defs.at(1).dfl_double = 0.5;
  }
  
  if(req.group_id == 1) {
    res.param_defs.resize(2);
    
    res.param_defs.at(1).name = "int_param_2";
    res.param_defs.at(1).type.type = arips_arm_msgs::ParameterType::TYPE_INT;
    res.param_defs.at(1).current_int_value = int_param;
    res.param_defs.at(1).max_int = 1000;
    res.param_defs.at(1).min_int = 0;
    res.param_defs.at(1).dfl_int = 42;
    
    res.param_defs.at(0).name = "double_param_3";
    res.param_defs.at(0).type.type = arips_arm_msgs::ParameterType::TYPE_DOUBLE;
    res.param_defs.at(0).current_double_value = double_param;
    res.param_defs.at(0).max_double = 5.0;
    res.param_defs.at(0).min_double = 0.0;
    res.param_defs.at(0).dfl_double = 0.5;
  }
  
  return true;
}

bool getParamGroupsCb(arips_arm_msgs::GetParameterGroups::Request& req, arips_arm_msgs::GetParameterGroups::Response& res) {
  res.groups.resize(2);
  res.groups.at(0).groupname = "group1";
  res.groups.at(1).groupname = "group2";
}

bool setParamCb(arips_arm_msgs::SetParameter::Request &req,
                arips_arm_msgs::SetParameter::Response &res)
{
  ROS_INFO_STREAM("Set client parameter for id " << req.id);
  
  if(req.id == 0) {
    int_param = req.int_value;
  } else if(req.id == 1) {
    double_param = req.double_value;
  }
  
  res.result = 0;
  
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service_groups = n.advertiseService("/myclient/tr_get_groups", getParamGroupsCb);
  ros::ServiceServer service = n.advertiseService("/myclient/tr_get_group_def", getParamGroupDefCb);
  ros::ServiceServer service_set = n.advertiseService("/myclient/tr_set_param", setParamCb);
  ros::Publisher pub = n.advertise<arips_arm_msgs::ParameterValue>("/myclient/tr_param_updates", 10, false);
  
  ROS_INFO("Ready to getParamCb two ints.");
  
  ros::Rate r(0.3);
  while (ros::ok()) {
    arips_arm_msgs::ParameterValue pv;
    
    
    int_param++;
    if(int_param > 100)
      int_param = 0;
    
    double_param += 0.02;
    if(double_param > 1) {
      double_param = 0;
    }
  
    pv.id = 0;
    pv.int_value = int_param;
    pv.type.type = arips_arm_msgs::ParameterType::TYPE_INT;
  
    pub.publish(pv);
    
    pv.id = 1;
    pv.double_value = double_param;
    pv.type.type = arips_arm_msgs::ParameterType::TYPE_DOUBLE;
    pub.publish(pv);
  
    ros::spinOnce();
    r.sleep();
    
  }
  
  return 0;
}
