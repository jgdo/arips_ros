#include "ros/ros.h"
#include "arips_arm_msgs/GetParameter.h"
#include <arips_arm_msgs/SetParameter.h>

bool add(arips_arm_msgs::GetParameter::Request  &req,
         arips_arm_msgs::GetParameter::Response &res)
{
  res.num_total_parameters = 2;
  if(req.parameter_id == 0) {
    res.name = "int_param_0";
    res.type.type = arips_arm_msgs::ParameterType::TYPE_INT;
    res.int_value = 13;
    res.max_int = 100;
    res.min_int = 0;
    res.dfl_int = 42;
  } else {
    if(req.parameter_id == 1) {
      res.name = "double_param_1";
      res.type.type = arips_arm_msgs::ParameterType::TYPE_DOUBLE;
      res.double_value = 0.13;
      res.max_double = 1.0;
      res.min_double = 0.0;
      res.dfl_double = 0.5;
    }
  }
  
  
  return true;
}

bool setParam(arips_arm_msgs::SetParameter::Request  &req,
         arips_arm_msgs::SetParameter::Response &res)
{
  ROS_INFO_STREAM("Set client parameter for id " << req.id);
  
  
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("/myclient/tr_get_param", add);
  ros::ServiceServer service_set = n.advertiseService("/myclient/tr_set_param", setParam);
  ROS_INFO("Ready to add two ints.");
  ros::spin();
  
  return 0;
}
