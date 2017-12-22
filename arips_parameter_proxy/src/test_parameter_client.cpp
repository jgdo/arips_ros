#include "ros/ros.h"
#include "arips_arm_msgs/GetParameter.h"

bool add(arips_arm_msgs::GetParameter::Request  &req,
         arips_arm_msgs::GetParameter::Response &res)
{
  res.num_total_parameters = 2;
    res.value.id = req.parameter_id;

    if(req.parameter_id == 0) {
        res.name = "int_param_0";
        res.value.type = arips_arm_msgs::ParameterValue::TYPE_INT;
        res.value.int_value = 13;
    } else {
        if(req.parameter_id == 1) {
            res.name = "double_param_1";
            res.value.type = arips_arm_msgs::ParameterValue::TYPE_DOUBLE;
            res.value.double_value = 0.13;
        }
    }


  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/myclient/tr_get_param", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
