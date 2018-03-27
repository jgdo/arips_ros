#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Dense>

#pragma GCC diagnostic pop

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>

#include <pluginlib/class_list_macros.h>

#include <tf/tf.h>



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
    std::vector<float> mLinkLength;
  
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

    v3 pos;
    tf::pointMsgToTF(ik_pose.position, pos);

    ROS_INFO_STREAM("pos = " << pos.x() << " " << pos.y() << " " << pos.z());

    v3 posJ2Diff = pos - v3(0, 0, mLinkLength.at(0) + mLinkLength.at(1));
    ROS_INFO_STREAM("posJ2Diff = " << posJ2Diff.x() << " " << posJ2Diff.y() << " " << posJ2Diff.z());


    float len2 = mLinkLength.at(2);
    float len3 = mLinkLength.at(3) + mLinkLength.at(4) + mLinkLength.at(5);
    float dist = posJ2Diff.length();

    ROS_INFO_STREAM("len2 = " << len2 << ", len3 = " << len3 << ", dist = " << dist);


    float j3 = 0;
    float offsetj2 = 0;

    if(std::abs(dist) < len2 + len3) {
        j3 = M_PI - std::acos((len2*len2 + len3*len3 - dist*dist) / (2*len2*len3));
        offsetj2 = std::acos((len2*len2 + dist*dist - len3*len3) / (2*len2*dist));
    }

    float basej2 = std::acos(posJ2Diff.z() / dist);

    ROS_INFO_STREAM("offsetj2 = " << offsetj2 << ", basej2 = " << basej2);

    float j2 = basej2 - offsetj2;

    float j1 = std::atan2(posJ2Diff.y(), posJ2Diff.x());

    solution = {j1, j2, j3, 0, 0};
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    ROS_INFO_STREAM("j1 = " << ", j2 = " << j2 << ", j3 = " << j3);
    ROS_INFO_STREAM("");
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
