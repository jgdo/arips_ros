#include "arips_navigation/utils/StepCostmapLayer.h"

#include <arips_navigation/StepEdgeModule.h>
#include <arips_navigation/utils/transforms.h>

StepCostmapLayer::StepCostmapLayer(SemanticMapTracker& mapTracker) : mMapTracker{mapTracker} {
    enabled_ = true;

}
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

    const auto semanticMap = mMapTracker.getLastSemanticMap();
    // ROS_INFO_STREAM("StepCostmapLayer plugin update: num doors: " << semanticMap.doors.size());

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

        master_grid.setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);
    }
}
