#include "arips_navigation/utils/StepCostmapLayer.h"

#include <arips_navigation/StepEdgeModule.h>
#include <arips_navigation/utils/transforms.h>

StepCostmapLayer::StepCostmapLayer(SemanticMapTracker& mapTracker) : mMapTracker{mapTracker} {
    enabled_ = true;
    name_ = "arips_navigation/steps";
}

void StepCostmapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                    double* min_y, double* max_x, double* max_y) {
    mLastPolygons.clear();
    if (!enabled_) {
        return;
    }

    static constexpr auto stepWidth = 0.05;

    auto toPoint = [](const tf2::Vector3& p) {
        geometry_msgs::Point msg;
        msg.x = p.x();
        msg.y = p.y();
        msg.z = p.z();
        return msg;
    };

    const auto semanticMap = mMapTracker.getLastSemanticMap();
    //ROS_INFO_STREAM(
    //    "StepCostmapLayer plugin updateBounds: num doors: " << semanticMap.doors.size());

    mLastPolygons.reserve(semanticMap.doors.size());
    for (const auto& door : semanticMap.doors) {
        const tf2::Vector3 start{door.pivot.x, door.pivot.y, 0.0};
        const tf2::Vector3 end{door.extent.x, door.extent.y, 0.0};

        const auto off =
            tf2::quatRotate(createQuaternionFromYaw(M_PI_2), end - start).normalized() * stepWidth *
            0.5;

        std::vector<geometry_msgs::Point> polygon{
            toPoint(start + off),
            toPoint(start - off),
            toPoint(end - off),
            toPoint(end + off),
        };

        for (const auto& p : polygon) {
            *min_x = std::min(*min_x, p.x);
            *min_y = std::min(*min_y, p.y);
            *max_x = std::max(*max_x, p.x);
            *max_y = std::max(*max_y, p.y);
        }

        mLastPolygons.push_back(std::move(polygon));
    }
}

void StepCostmapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                   int max_i, int max_j) {

    if (!enabled_) {
        return;
    }

    // ROS_INFO_STREAM("StepCostmapLayer plugin updateCosts: num polygons: " << mLastPolygons.size());

    for (const auto& polygon : mLastPolygons) {

        master_grid.setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);
    }
}
