#pragma once

#include <toponav_core/interfaces/EdgePlanningInterface.h>
#include <toponav_ros/interfaces/EdgeStorageInterface.h>
#include <toponav_ros/interfaces/EdgeVisualizationInterface.h>
#include <toponav_ros/utils/NamedModuleContainer.h>

namespace toponav_ros {
class EdgeModuleContainer: public toponav_core::EdgePlanningInterface, public EdgeStorageInterface, public EdgeVisualizationInterface {
public:
	typedef std::shared_ptr<EdgeModuleContainer> Ptr;
	
	inline void addPlanningModule(std::string type, toponav_core::EdgePlanningInterfacePtr const &plugin) {
		planningContainer.addModule(type, plugin);
	}
	
	inline void addStorageModule(std::string type, EdgeStorageInterfacePtr const &plugin) {
		storageContainer.addModule(type, plugin);
	}
	
	inline void addVisualizationModule(std::string type, EdgeVisualizationPluginPtr const &plugin) {
		vizContainer.addModule(type, plugin);
	}
  
  void initApproachData(const toponav_core::TopoMap::Edge *edge, toponav_core::AbstractApproachExitData *approachData) override;
  
  void initExitData(const toponav_core::TopoMap::Edge *edge, toponav_core::AbstractApproachExitData *exitData) override;
  
  std::pair<double, toponav_core::LocalPositionConstPtr>
  computeTransitionCost(const toponav_core::TopoMap::Edge *edge, const toponav_core::LocalPosition &approachPose, boost::any *pathData) override;
  
  virtual void beginParsing(toponav_core::TopoMap *map, YAML::Node const &parserConfig) override;
	virtual void endParsing() override;
	
	virtual void parseEdgeData(YAML::Node const &config, toponav_core::TopoMap::Edge *edge) override;
	
	virtual YAML::Node beginSaving(toponav_core::TopoMap *map) override;
	
	virtual void endSaving() override;
	
	virtual YAML::Node saveEdgeData(toponav_core::TopoMap::Edge const *edge) override;
	
	virtual void initializeVisualization(MapEditor *editor) override;
	
	virtual void beginMapVisualization() override;
	
	virtual void endMapVisualization() override;
	
	virtual void visualizeEdge(const toponav_core::TopoMap::Edge *edge) override;
	
	virtual void appendTransitionToPlan(nav_msgs::Path *pathPtr, std::string transitionType,
										boost::any const &pathData) override;
	
	virtual bool areEdgesCoupled(const toponav_core::TopoMap::Edge *edgeA, const toponav_core::TopoMap::Edge *edgeB) override;
	
	void parseEdgeOutData(YAML::Node const &config, toponav_core::TopoMap::Edge *edge) override;
	
	void parseEdgeInData(YAML::Node const &config, toponav_core::TopoMap::Edge *edge) override;
	
	YAML::Node saveEdgeOutData(toponav_core::TopoMap::Edge const *edge) override;
	
	YAML::Node saveEdgeInData(toponav_core::TopoMap::Edge const *edge) override;
	
	double getHeuristics(toponav_core::TopoMap::Edge const *edge) override;
  
  std::string getModuleType(const toponav_core::TopoMap::Edge *edge) override;
	
	bool isEdgeSaveable(const toponav_core::TopoMap::Edge *edge) const override;

private:
	NamedModuleContainer<toponav_core::EdgePlanningInterface> planningContainer;
	NamedModuleContainer<EdgeStorageInterface> storageContainer;
	NamedModuleContainer<EdgeVisualizationInterface> vizContainer;
};

typedef EdgeModuleContainer::Ptr EdgeModuleContainerPtr;

} // namespace topo_nav
