//
// Created by jgdo on 1/4/21.
//

#include <arips_navigation/utils/transforms.h>

namespace tf2 {

template <>
void tf2::doTransform(tf2::Stamped<tf2::Transform> const &in,
                      tf2::Stamped<tf2::Transform> &out,
                      geometry_msgs::TransformStamped const &transformMsg) {
  tf2::Stamped<tf2::Transform> transform;
  tf2::fromMsg(transformMsg, transform);

  tf2::Transform res = transform * in;
  out.frame_id_ = transformMsg.child_frame_id;
  out.stamp_ = in.stamp_;
  out.setData(res);
}

} // namespace tf2

std::optional<tf2::Stamped<tf2::Transform>>
tryTransformPoseMsgToTransform(tf2_ros::Buffer &tf,
                               const geometry_msgs::PoseStamped &msg,
                               const std::string &targetFrame,
                               ros::Duration timeout) {
  try {
    const geometry_msgs::PoseStamped transformedMsg = tf.transform(msg, targetFrame, timeout);

    tf2::Stamped<tf2::Transform> tfType;
    tf2::fromMsg(transformedMsg, tfType);
    return tfType;
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_STREAM("tryTransformFromMsg() error: " << ex.what());
    return {};
  }
}