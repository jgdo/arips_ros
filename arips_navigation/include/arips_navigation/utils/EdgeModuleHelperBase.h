#pragma once

#include <boost/any.hpp>
#include <toponav_core/TopoMap.h>
#include "BaseTraits.h"

namespace toponav_ros {

template<class Parent>
class EdgeModuleHelperBase  {
  typedef typename BaseTraits<Parent>::ApproachData _ApproachData;
  typedef typename BaseTraits<Parent>::ExitData _ExitData;
  typedef typename BaseTraits<Parent>::EdgeData _EdgeData;
  typedef typename BaseTraits<Parent>::MapData _MapData;
  
public:
  static inline const _ApproachData& getApproachData(const toponav_core::TopoMap::Edge* edge) {
    return *boost::any_cast<const _ApproachData>(&(edge->approachData()));
  }
  
  static inline const _ExitData& getExitData(const toponav_core::TopoMap::Edge* edge) {
    return *boost::any_cast<const _ExitData>(&(edge->exitData()));
  }
  
  static void setApproachData(toponav_core::TopoMap::Edge *edge, const _ApproachData& approachData) {
    edge->approachData() = approachData;
  }
  
  static void setExitData(toponav_core::TopoMap::Edge *edge, const _ExitData& exitData) {
    edge->exitData() = exitData;
  }
  
  static inline const _EdgeData& getEdgeData(const toponav_core::TopoMap::Edge* edge) {
    return *boost::any_cast<const _EdgeData>(&(edge->edgeData()));
  }
  
  static inline _EdgeData& getEdgeData(toponav_core::TopoMap::Edge* edge) {
    return *boost::any_cast<_EdgeData>(&(edge->edgeData()));
  }
  
  static inline const _MapData& getMapData(const toponav_core::TopoMap* map) {
    return *boost::any_cast<const _MapData>(&(map->propertyMap().at(Parent::className)));
  }
  
  static inline _MapData& getMapData(toponav_core::TopoMap* map) {
    return *boost::any_cast<_MapData>(&(map->propertyMap().at(Parent::className)));
  }
};

}
