// Map client subscribing to semantic map and creating a topo map suitable for navigation

#pragma once

#include <memory>
#include <optional>

#include <arips_navigation/path_planning/Costmap.h>
#include <arips_navigation/path_planning/Locomotion.h>
#include <arips_navigation/topo/TopoPath.h>

#include <arips_semantic_map_msgs/SemanticMap.h>


class SemanticTopoPlanner {
public:
    SemanticTopoPlanner(Locomotion& locomotion);
    ~SemanticTopoPlanner();

    std::optional<TopoPath> plan(const Costmap& costmap,
                                 const arips_semantic_map_msgs::SemanticMap& semanticMap,
                                 Pose2D start, Pose2D goal);

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};
