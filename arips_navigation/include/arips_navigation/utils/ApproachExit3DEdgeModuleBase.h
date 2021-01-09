#pragma once

#include <toponav_core/interfaces/EdgePlanningInterface.h>
#include <toponav_ros/interfaces/EdgeStorageInterface.h>
#include "EdgeModuleHelperBase.h"
#include "ApproachExit3D.h"
#include "BaseTraits.h"
#include "ApproachExit3DStorageHelper.h"
#include "MoveGoalProperties.h"
#include <toponav_ros/utils/YamlTools.h>

namespace toponav_ros {

struct ApproachExit3DEdgeModuleBaseTraits {
  typedef ApproachExit3DPtr ApproachData;
  typedef ApproachExit3DPtr ExitData;
};

template <class Parent, class TNodePlugin>
class ApproachExit3DEdgeModuleBase: public EdgeModuleHelperBase<Parent>, public toponav_core::EdgePlanningInterface, public EdgeStorageInterface {
public:
  using EdgeModuleHelperBase<Parent>::setApproachData;
  using EdgeModuleHelperBase<Parent>::setExitData;
  using EdgeModuleHelperBase<Parent>::getApproachData;
  using EdgeModuleHelperBase<Parent>::getExitData;
  using EdgeModuleHelperBase<Parent>::getMapData;
  
  typedef typename TNodePlugin::PositionData PositionData;
  typedef typename TNodePlugin::PositionDataPtr PositionDataPtr;
  
  MoveGoalProperties goal_properties;
  
  virtual void initApproachData(const toponav_core::TopoMap::Edge *edge, toponav_core::AbstractApproachExitData* approachData) override  {
    auto& outData = getApproachData(edge);
    *approachData = outData;
  }
  
  virtual void initExitData(const toponav_core::TopoMap::Edge *edge, toponav_core::AbstractApproachExitData* exitData) override {
    auto& inData = getExitData(edge);
    *exitData = inData;
  }
  
  virtual void parseEdgeOutData(YAML::Node const &config, toponav_core::TopoMap::Edge *edge) override {
    auto approach = ApproachExit3DStorageHelper::parseApproachExit(config["approach"], "Error when parsing approach pose for elevator door '" + edge->getName() + "': frame_id is empty");
    if(!approach) {
      ROS_ERROR_STREAM("Failed to parse approach pose of edge '" << edge->getName() << "' since approach type '" << config["approach"]["type"] << "' is unknown");
      return;
      // FIXME: what to do
    }
    
    approach->goal_properties = goal_properties;
    setApproachData(edge, approach);
  }
  
  virtual void parseEdgeInData(YAML::Node const &config, toponav_core::TopoMap::Edge *edge) override {
    auto exit = ApproachExit3DStorageHelper::parseApproachExit(config["exit"], "Error when parsing exit pose for edge '" + edge->getName() + "': frame_id is empty");
    if(!exit) {
      ROS_ERROR_STREAM("Failed to parse exit pose of edge '" << edge->getName() << "' since exit type '" << config["exit"]["type"] << "' is unknown");
      return;
      // FIXME: what to do
    }
  
    setExitData(edge, exit);
  }
  
  virtual YAML::Node saveEdgeOutData(toponav_core::TopoMap::Edge const *edge) override {
    auto const& outData = getApproachData(edge);
    YAML::Node root;
  
    if(!(root["approach"] = outData)) {
      ROS_ERROR_STREAM("Cannot save approach pose of edge '" << edge->getName() << "' type since type is unknown");
      // FIXME what to do?
    }
  
    return root;
  }
  
  virtual YAML::Node saveEdgeInData(toponav_core::TopoMap::Edge const *edge) override {
    auto const& inData = getExitData(edge);
    YAML::Node root;
  
    if(!(root["exit"] = inData)) {
      ROS_ERROR_STREAM("FlatParser: cannot save exit pose of edge '" << edge->getName() << "' type since type is unknown");
      // FIXME what to do?
    }
  
    return root;
  }
};

}

