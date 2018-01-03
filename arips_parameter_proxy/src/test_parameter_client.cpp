#include "ros/ros.h"
#include "arips_arm_msgs/GetParameter.h"
#include <arips_arm_msgs/SetParameter.h>
#include <arips_arm_msgs/ParameterValue.h>

int int_param = 0;
double double_param = 0.0;

bool getParamCb(arips_arm_msgs::GetParameter::Request &req,
                arips_arm_msgs::GetParameter::Response &res)
{
  res.num_total_parameters = 2;
  if(req.parameter_id == 0) {
    res.name = "int_param_0";
    res.type.type = arips_arm_msgs::ParameterType::TYPE_INT;
    res.int_value = int_param;
    res.max_int = 100;
    res.min_int = 0;
    res.dfl_int = 42;
  } else {
    if(req.parameter_id == 1) {
      res.name = "double_param_1";
      res.type.type = arips_arm_msgs::ParameterType::TYPE_DOUBLE;
      res.double_value = double_param;
      res.max_double = 1.0;
      res.min_double = 0.0;
      res.dfl_double = 0.5;
    }
  }
  
  
  return true;
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
  
  ros::ServiceServer service = n.advertiseService("/myclient/tr_get_param", getParamCb);
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
