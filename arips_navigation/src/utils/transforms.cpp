//
// Created by jgdo on 1/4/21.
//

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2 {

template<>
void tf2::doTransform(tf2::Stamped<tf2::Transform> const& in,
                                                     tf2::Stamped<tf2::Transform> &out,
                                                     geometry_msgs::TransformStamped const & transformMsg) {
    tf2::Stamped<tf2::Transform> transform;
    tf2::fromMsg(transformMsg, transform);

    tf2::Transform res = transform * in;
    out.frame_id_ = transformMsg.child_frame_id;
    out.stamp_ = in.stamp_;
    out.setData(res);
}

} // tf2

