#pragma once

#include <memory>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <toponav_ros/RosContext.h>

#include <toponav_core/interfaces/NodePlanningInterface.h>
#include <toponav_ros/interfaces/MapPoseInterface.h>

#include <arips_navigation/utils/ApproachExit3D.h>
#include <arips_navigation/utils/MoveGoalProperties.h>
#include <arips_navigation/utils/NodeModuleHelperBase.h>
#include <toponav_ros/utils/CostsProfileHolderBase.h>

#include <costmap_2d/costmap_2d_ros.h>

namespace toponav_ros {

struct FlatGroundModule;

template <> struct BaseTraits<FlatGroundModule> {
    class CostsPlanner {
    public:
        typedef std::shared_ptr<CostsPlanner> Ptr;

        inline virtual ~CostsPlanner() {}

        virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                              ApproachExit3DPtr const& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan,
                              double* costs = nullptr,
                              tf2::Stamped<tf2::Transform>* actualApproachPose = nullptr) = 0;

        virtual costmap_2d::Costmap2DROS& getMap() = 0;
    };

    typedef CostsPlanner::Ptr CostsPlannerPtr;

    typedef Eigen::Matrix<toponav_core::TopoMap::Node*, Eigen::Dynamic, Eigen::Dynamic> NodeMatrix;

    struct MapData {
        double time_factor = 1.0;
        double energy_factor = 1.0;
        double distance_factor = 1.0;

        CostsPlanner* planner;
        NodeMatrix nodeMatrix;
    };

    struct NodeData {
        int x, y;
    };
};

class FlatGroundModule : public RosContextHolder,
                         public toponav_core::NodePlanningInterface,
                         public MapPoseInterface,
                         public NodeModuleHelperBase<FlatGroundModule>,
                         public BaseTraits<FlatGroundModule>,
                         public CostsProfileHolderBase {
public:
    static const std::string className;

    typedef std::shared_ptr<FlatGroundModule> Ptr;

    struct PositionData : public toponav_core::LocalPosition {
        PositionData(tf2::Stamped<tf2::Transform> const& pose) : pose(pose) {}
        tf2::Stamped<tf2::Transform> pose;

        MoveGoalProperties goal_properties;
    };

    typedef std::shared_ptr<PositionData> PositionDataPtr;

    template <typename... Arg> Ptr static create(Arg&&... arg) {
        struct EnableMakeShared : public FlatGroundModule {
            EnableMakeShared(Arg&&... arg) : FlatGroundModule(std::forward<Arg>(arg)...) {}
        };

        return std::make_shared<EnableMakeShared>(std::forward<Arg>(arg)...);
    }

    static const PositionData&
    getPositionDataFromGlobal(toponav_core::GlobalPosition const& global_position);

    /**
     * @brief initNodeData
     * @param node
     * @param x x pixel position in map image
     * @param y y pixel position in map image (inverted on costmap)
     */
    static void initNodeData(toponav_core::TopoMap::Node* node, int x, int y);

    static size_t regionGrow(toponav_core::TopoMap::Node* node,
                             const costmap_2d::Costmap2D* costmap, int x, int y, NodeMatrix* mat);

    /**
     * Find GlobalPose from tf 3d pose by iterating over all costmap layers and checking if z lies
     * on map height and x/y is within costmap range.
     *
     * @param pose
     * @param map
     * @return empty global pos if not found
     */
    virtual std::pair<toponav_core::GlobalPosition, tf2::Stamped<tf2::Transform>>
    findGlobalPose(tf2::Stamped<tf2::Transform> const& pose, const toponav_core::TopoMap& map);

    virtual tf2::Stamped<tf2::Transform>
    getMapPoseFromPosition(toponav_core::GlobalPosition const& pos) override;

    /**
     *
     * @param map
     * @param mapName
     * @param nodeType
     * @param minNumCells 0 means no restriction
     */
    static void segmentAllNodes(toponav_core::TopoMap* map,
                                std::string const& nodeType, size_t minNumCells);

    static void combineNodeMatrix(NodeMatrix& mat, const NodeMatrix& other);

    std::pair<double, toponav_core::LocalPositionConstPtr>
    computeCostsOnRegion(const toponav_core::TopoMap::Node* node,
                         toponav_core::LocalPosition const& start,
                         toponav_core::AbstractApproachExitData const& end,
                         toponav_core::AbstractPathData* pathData) override;

    void convertGlobalPoseToApproachExitData(
        const toponav_core::GlobalPosition& pos,
        toponav_core::AbstractApproachExitData* approachData) override;

    double getHeuristics(toponav_core::TopoMap::Node const* region,
                         toponav_core::AbstractApproachExitData const& startData,
                         toponav_core::AbstractApproachExitData const& endData) override;

    std::string getModuleType(const toponav_core::TopoMap::Node* node) override;

protected:
    using RosContextHolder::RosContextHolder;

    double getCosts(MapData const& data, double distance);
};

typedef FlatGroundModule::Ptr FlatGroundModulePtr;

} // namespace toponav_ros
