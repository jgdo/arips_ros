#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Dense>

#pragma GCC diagnostic pop

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>

#include <pluginlib/class_list_macros.h>

#include <tf/tf.h>

#include <arips_arm_ik_plugin/IKComputations.h>



namespace arips_arm_plugins {


class AripsArmIkPlugin: public kinematics::KinematicsBase {
public:
  bool getPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                     std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                     const kinematics::KinematicsQueryOptions &options) const override;
  
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                        std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options) const override;
  
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                        const std::vector<double> &consistency_limits, std::vector<double> &solution,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options) const override;
  
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                        std::vector<double> &solution, const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options) const override;
  
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                        const std::vector<double> &consistency_limits, std::vector<double> &solution,
                        const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options) const override;
  
  bool getPositionFK(const std::vector<std::string> &link_names, const std::vector<double> &joint_angles,
                     std::vector<geometry_msgs::Pose> &poses) const override;
  
  bool initialize(const std::string &robot_description, const std::string &group_name, const std::string &base_frame,
                  const std::string &tip_frame, double search_discretization) override;
  
  const std::vector<std::string> &getJointNames() const override;
  
  const std::vector<std::string> &getLinkNames() const override;

private:
    std::vector<double> mLinkLength;
  
};

bool AripsArmIkPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                     std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                                     const kinematics::KinematicsQueryOptions &options) const {
  ROS_INFO_STREAM_NAMED("arips_ik", __LINE__);
  return false;
}

bool AripsArmIkPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                        double timeout, std::vector<double> &solution,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const {
  ROS_INFO_STREAM_NAMED("arips_ik", __LINE__);
  return false;
}

bool AripsArmIkPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                        double timeout, const std::vector<double> &consistency_limits,
                                        std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const {
  ROS_INFO_STREAM_NAMED("arips_ik", __LINE__);
  return false;
}

bool AripsArmIkPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                        double timeout, std::vector<double> &solution,
                                        const kinematics::KinematicsBase::IKCallbackFn &solution_callback,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const {
  ROS_INFO_STREAM_NAMED("arips_ik", __LINE__);
  return false;
}

bool AripsArmIkPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                        double timeout, const std::vector<double> &consistency_limits,
                                        std::vector<double> &solution,
                                        const kinematics::KinematicsBase::IKCallbackFn &solution_callback,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const {

    ROS_INFO_STREAM_NAMED("arips_ik", __LINE__);


    using v3 = tf::Vector3;
    using quat = tf::Quaternion;

    v3 posOrig;
    quat rotOrig;

    tf::pointMsgToTF(ik_pose.position, posOrig);
    tf::quaternionMsgToTF(ik_pose.orientation, rotOrig);

    IKComputations::computeIK(posOrig, rotOrig, mLinkLength, solution);

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

bool
AripsArmIkPlugin::getPositionFK(const std::vector<std::string> &link_names, const std::vector<double> &joint_angles,
                                std::vector<geometry_msgs::Pose> &poses) const {
  ROS_INFO_STREAM_NAMED("arips_ik", __LINE__);
  return false;
}

bool AripsArmIkPlugin::initialize(const std::string &robot_description, const std::string &group_name,
                                  const std::string &base_frame, const std::string &tip_frame,
                                  double search_discretization) {
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
  ros::NodeHandle node_handle("~");
  
  urdf::Model robot_model;
  std::string xml_string;
  
  std::string urdf_xml,full_urdf_xml;
  node_handle.param("urdf_xml",urdf_xml,robot_description);
  node_handle.searchParam(urdf_xml,full_urdf_xml);
  
  ROS_DEBUG_NAMED("arips_ik","Reading xml file from parameter server");
  if (!node_handle.getParam(full_urdf_xml, xml_string))
  {
    ROS_FATAL_NAMED("arips_ik","Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }
  
  node_handle.param(full_urdf_xml,xml_string,std::string());
  robot_model.initString(xml_string);
  
  ROS_DEBUG_STREAM_NAMED("arips_ik","Reading joints and links from URDF");

    mLinkLength.reserve(robot_model.joints_.size());

  for(auto& e: robot_model.joints_) {
    auto& pos = e.second->parent_to_joint_origin_transform.position;
    tf::Vector3 postf(pos.x, pos.y, pos.z);
    ROS_INFO_STREAM_NAMED("arips_ik","joint " << e.second->name << " len = " << postf.length());
      mLinkLength.push_back(postf.length());
  }
  
  return true;
}

const std::vector<std::string> &AripsArmIkPlugin::getJointNames() const {
  static const std::vector<std::string> joints = { "joint1", "joint2", "joint3", "joint4", "joint5"};
  return joints;
}

const std::vector<std::string> &AripsArmIkPlugin::getLinkNames() const {
  static const std::vector<std::string> links = { "base_link", "link2", "link3", "link4", "link5", "link6", "tool0"};
  return links;
}
  
  
} // ns arips_arm_plugins


PLUGINLIB_EXPORT_CLASS(arips_arm_plugins::AripsArmIkPlugin, kinematics::KinematicsBase)
