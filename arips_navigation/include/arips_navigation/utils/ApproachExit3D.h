#pragma once

#include <memory>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include "MoveGoalProperties.h"

namespace toponav_ros {

class ApproachExit3D {
public:
  typedef std::shared_ptr<ApproachExit3D> Ptr;
  typedef std::shared_ptr<const ApproachExit3D> ConstPtr;
  
  virtual tf2::Stamped<tf2::Transform> getCenter() const = 0;
  
  inline virtual ~ApproachExit3D() {}
  
  MoveGoalProperties goal_properties;
};

typedef ApproachExit3D::Ptr ApproachExit3DPtr;
typedef ApproachExit3D::ConstPtr ApproachExit3DConstPtr;

} // namespace topo_nav

