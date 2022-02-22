#pragma once

#include <arips_navigation/utils/FloorStepTracker.h>
#include <arips_navigation/StepEdgeModule.h>


class StepModuleFromTracker: public toponav_ros::EdgeVisualizationInterface, public toponav_ros::PlanningContextHolder {
    FloorStepTracker& mTracker;
    toponav_ros::StepEdgeModule& mStepModule;

    void parseStepsFromTracker();

public:
    StepModuleFromTracker(toponav_ros::PlanningContext& context,
                          FloorStepTracker& tracker,
                          toponav_ros::StepEdgeModule& stepModule);

    void visualizeEdge(const toponav_core::TopoMap::Edge* edge) override;
    void initializeVisualization(toponav_ros::MapEditor* editor) override;
};