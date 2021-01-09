#pragma once

#include "BaseTraits.h"

namespace toponav_ros {

template <class Parent>
class NodeModuleHelperBase {
  typedef typename BaseTraits<Parent>::NodeData _NodeData;
  typedef typename BaseTraits<Parent>::MapData _MapData;
  
public:
  static inline const _NodeData& getNodeData(const toponav_core::TopoMap::Node* node) {
    return *boost::any_cast<const _NodeData>(&(node->nodeData()));
  }
  
  static inline _NodeData& getNodeData(toponav_core::TopoMap::Node* node) {
    return *boost::any_cast<_NodeData>(&(node->nodeData()));
  }
  
  static inline _MapData& getMapData(toponav_core::TopoMap* map) {
    return *boost::any_cast<_MapData>(&(map->propertyMap().at(Parent::className)));
  }
  
  static inline const _MapData& getMapData(const toponav_core::TopoMap* map) {
    return *boost::any_cast<const _MapData>(&(map->propertyMap().at(Parent::className)));
  }
  
  static inline void setMapData(toponav_core::TopoMap* map, _MapData const& mapData) {
    map->propertyMap()[Parent::className] = mapData;
  }
};

}