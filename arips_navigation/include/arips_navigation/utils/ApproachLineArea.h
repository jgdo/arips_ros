#pragma once

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "ApproachExit3D.h"

namespace toponav_ros {

class ApproachLineArea: public ApproachExit3D {
public:
  typedef std::shared_ptr<ApproachLineArea> Ptr;
  typedef std::shared_ptr<const ApproachLineArea> ConstPtr;
  
  tf2::Vector3 start, end; // frame is from orientation
  tf2::Stamped<tf2::Quaternion> orientation;
  
  inline ApproachLineArea(tf2::Vector3 const& start, tf2::Vector3 const& end, tf2::Stamped<tf2::Quaternion>const& rot):
      start(start), end(end), orientation(rot) {
  }
  
  inline ApproachLineArea transformToFrameNow(tf2_ros::Buffer& listener, std::string const& frame) const {
    tf2::Stamped<tf2::Vector3> s = listener.transform(tf2::Stamped<tf2::Vector3>(start, orientation.stamp_, orientation.frame_id_), frame);
      tf2::Stamped<tf2::Vector3> e = listener.transform(tf2::Stamped<tf2::Vector3>(end, orientation.stamp_, orientation.frame_id_), frame);
      tf2::Stamped<tf2::Quaternion> rot = listener.transform(orientation, frame);
    
    return ApproachLineArea(s, e, rot);
  }
  
  inline static ApproachExit3DPtr create(tf2::Vector3 const& start, tf2::Vector3 const& end, tf2::Stamped<tf2::Quaternion>const& rot) {
    return ApproachExit3DPtr(new ApproachLineArea(start, end, rot) );
  }
  
  tf2::Stamped<tf2::Transform> getCenter() const override {
    return tf2::Stamped<tf2::Transform>(tf2::Transform(orientation, (start+end)*0.5), orientation.stamp_, orientation.frame_id_);
  }
};

typedef ApproachLineArea::Ptr ApproachLineAreaPtr;
typedef ApproachLineArea::ConstPtr ApproachLineAreaConstPtr;

} // namespace topo_nav