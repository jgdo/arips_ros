#pragma once

#include <optional>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
template<class TTf, class TMsg>
std::optional<TTf> tryTransformFromMsg(tf2_ros::Buffer &tf, const TMsg &msg, const std::string &targetFrame,
                                       ros::Duration timeout = ros::Duration(0.0)) {
    try {
        const TMsg transformedMsg = tf.transform(msg, targetFrame, timeout);

        TTf tfType;
        tf2::fromMsg<TMsg, TTf>(transformedMsg, tfType);
        return tfType;
    } catch (const tf2::TransformException &ex) {
        ROS_WARN_STREAM("tryTransformFromMsg() error: " << ex.what());
        return {};
    }
}

 */

std::optional<tf2::Stamped<tf2::Transform>> tryTransformPoseMsgToTransform(tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &msg, const std::string &targetFrame,
                                       ros::Duration timeout = ros::Duration(0.0));

template<class TMsg>
std::optional<TMsg> tryTransform(tf2_ros::Buffer &tf, const TMsg &msg, const std::string &targetFrame,
                                       ros::Duration timeout = ros::Duration(0.0)) {
    try {
        return tf.transform(msg, targetFrame, timeout);
    } catch (const tf2::TransformException &ex) {
        ROS_WARN_STREAM("tryTransform() error: " << ex.what());
        return {};
    }
}