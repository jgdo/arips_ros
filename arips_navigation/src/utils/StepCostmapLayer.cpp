#pragma once

#include "arips_navigation/utils/StepCostmapLayer.h"

#include <arips_navigation/StepEdgeModule.h>
#include <arips_navigation/utils/transforms.h>

StepCostmapLayer::StepCostmapLayer(toponav_core::TopoMapPtr topoMap) : mTopoMap{std::move(topoMap)} {}
void StepCostmapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                   int max_i, int max_j) {

    static constexpr auto stepWidth = 0.05;

    auto toPoint = [](const tf2::Vector3& p) {
        geometry_msgs::Point msg;
        msg.x = p.x();
        msg.y = p.y();
        msg.z = p.z();
        return msg;
    };

    const auto& mapData = toponav_ros::StepEdgeModule::getMapData(mTopoMap.get());
    for (const auto& step : mapData.steps) {

        const auto off =
            tf2::quatRotate(createQuaternionFromYaw(M_PI_2), step.second.end - step.second.start)
                .normalized() *
            stepWidth*0.5;

        std::vector<geometry_msgs::Point> polygon{
            toPoint(step.second.start + off),
            toPoint(step.second.start - off),
            toPoint(step.second.end - off),
            toPoint(step.second.end + off),
        };

        master_grid.setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);
    }
}
