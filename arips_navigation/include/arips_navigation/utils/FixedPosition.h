#pragma once

#include "ApproachExit3D.h"
#include <tf/transform_datatypes.h>

namespace toponav_ros {

class FixedPosition: public ApproachExit3D {
public:
  typedef std::shared_ptr<FixedPosition> Ptr;
  typedef std::shared_ptr<const FixedPosition> ConstPtr;
  
  inline FixedPosition(tf2::Stamped<tf2::Transform> const& pose): pose(pose) {}
  inline FixedPosition() {}
  
  inline static Ptr create(tf2::Stamped<tf2::Transform> const& pose) {
    return std::make_shared<FixedPosition>(pose);
  }
  
  tf2::Stamped<tf2::Transform> getCenter() const override {
    return pose;
  }
  
  tf2::Stamped<tf2::Transform> pose;
};

typedef FixedPosition::Ptr FixedPositionPtr;
typedef FixedPosition::ConstPtr FixedPositionConstPtr;

} // namespace topo_nav
