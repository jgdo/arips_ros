#pragma once

#include <arips_navigation/FlatGroundModule.h>
#include <arips_navigation/utils/ApproachExit3DEdgeModuleBase.h>
#include <toponav_core/interfaces/EdgePlanningInterface.h>
#include <toponav_ros/interfaces/EdgeStorageInterface.h>
#include <toponav_ros/interfaces/EdgeVisualizationInterface.h>
#include <toponav_ros/interfaces/MapEditorModule.h>

#include <tf/transform_listener.h>
#include <toponav_ros/utils/CostsProfileHolderBase.h>

#include "utils/ApproachExitVisualizer.h"

#include <arips_navigation/path_planning/PlanningMath.h>

namespace toponav_ros {

using FloorStep2d = std::array<Point2d, 2>;

class StepEdgeModule;

template <> struct BaseTraits<StepEdgeModule> : public ApproachExit3DEdgeModuleBaseTraits {
    struct CostSettings {
        double time_costs = 1;
        double energy_costs = 1;
        double distance_costs = 1;
    };

    /**
     * Contains information about a single step
     */
    struct StepInfo {
        virtual ~StepInfo() = default;

        StepInfo(const tf2::Stamped<tf2::Vector3>& start, const tf2::Stamped<tf2::Vector3>& end,
                 std::string name, CostSettings upCosts, CostSettings downCosts,
                 CostSettings crossOverCosts, CostSettings crossGravelCosts)
            : start{start}, end{end}, name{std::move(name)}, upCosts{upCosts}, downCosts{downCosts},
              crossOverCosts{crossOverCosts}, crossGravelCosts{crossGravelCosts} {}

        tf2::Stamped<tf2::Vector3> start, end;
        std::string name;
        CostSettings upCosts, downCosts, crossOverCosts, crossGravelCosts;

        std::shared_ptr<interactive_markers::MenuHandler> menuHandler;
    };

    struct TrackedStepInfo : public StepInfo {
        TrackedStepInfo(const tf2::Stamped<tf2::Vector3>& start,
                        const tf2::Stamped<tf2::Vector3>& end, std::string name,
                        CostSettings upCosts, CostSettings downCosts, CostSettings crossOverCosts,
                        CostSettings crossGravelCosts)
            : StepInfo{start,           end, std::move(name), upCosts, downCosts, crossOverCosts,
                       crossGravelCosts} {}

        size_t count = 1;

        static constexpr float GAMMA = 0.9f;
        static constexpr int MAX_COUNT = 100;

        static auto dist(const Point2d& lhs, const Point2d& rhs) { return (lhs - rhs).norm(); }

        static bool isClose(const Point2d& lhs, const Point2d& rhs, float tolerance) {
            return (lhs - rhs).norm() < tolerance;
        }

        FloorStep2d getStep2d() const {
            return FloorStep2d{Point2d{start.x(), start.y()}, Point2d{end.x(), end.y()}};
        }

        bool isWithinTolerance(const FloorStep2d& obs, float tolerance) const {
            const auto step = getStep2d();
            return (isClose(obs.at(0), step.at(0), tolerance) &&
                    isClose(obs.at(1), step.at(1), tolerance)) ||
                   (isClose(obs.at(0), step.at(1), tolerance) &&
                    isClose(obs.at(1), step.at(0), tolerance));
        }

        using M = std::pair<int, int>;
        M bestMatch(const FloorStep2d& other) const {
            const auto step = getStep2d();

            std::pair<int, int> result{0, 0};
            double best = -1;

            for (int i = 0; i < step.size(); i++) {
                for (int j = 0; j < other.size(); j++) {
                    const auto d = dist(step.at(i), other.at(j));
                    if (best < 0 || d < best) {
                        best = d;
                        result = {i, j};
                    }
                }
            }

            return result;
        }

        void track(const FloorStep2d& obs) {
            const auto best = bestMatch(obs);

            if ((best == M{0, 0}) || (best == M{1, 1})) {
                // 0 close to 0
                combine(obs);
            } else {

                combine({obs.at(1), obs.at(0)});
            }
        }

        void combine(const FloorStep2d& other) {
            tf2::Stamped<tf2::Vector3>* thisSteps[2] = {&start, &end};
            for (size_t i = 0; i < other.size(); i++) {
                thisSteps[i]->setX(thisSteps[i]->x() * GAMMA + other[i].x() * (1.0F - GAMMA));
                thisSteps[i]->setY(thisSteps[i]->y() * GAMMA + other[i].y() * (1.0F - GAMMA));
            }

            if (count < MAX_COUNT) {
                count++;
            }
        }
    };

    /**
     * For usage as map user data
     */
    struct MapStepData {
        // TODO find out why shared instead of unique required
        std::map<std::string, std::shared_ptr<StepInfo>> steps;
        CostSettings defaultUpCosts, defaultDownCosts, defaultCrossOverCosts,
            defaultCrossGravelCosts;
    };

    /**
     * For usage as edge user data
     */
    struct EdgeStepData {
        std::string stepName;
        enum StepType { UP, DOWN, CROSS_OVER, CROSS_GRAVEL } type;

        typedef std::shared_ptr<interactive_markers::MenuHandler> MenuHandlerPtr;
        MenuHandlerPtr menu_handler;
    };

    typedef EdgeStepData EdgeData;
    typedef MapStepData MapData;
};

class StepEdgeModule : public ApproachExit3DEdgeModuleBase<StepEdgeModule, FlatGroundModule>,
                       public ApproachExitVisualizer,
                       public MapEditorModule,
                       public BaseTraits<StepEdgeModule>,
                       public CostsProfileHolderBase {
public:
    typedef std::shared_ptr<StepEdgeModule> Ptr;

    static const std::string className;

    StepEdgeModule(PlanningContext& context, bool dynamicStepsFromDetection);

    virtual void beginParsing(toponav_core::TopoMap* map, YAML::Node const& parserConfig) override;

    virtual void endParsing() override;

    virtual void parseEdgeData(YAML::Node const& config,
                               toponav_core::TopoMap::Edge* edge) override;

    virtual std::pair<double, toponav_core::LocalPositionConstPtr>
    computeTransitionCost(const toponav_core::TopoMap::Edge* edge,
                          const toponav_core::LocalPosition& approachPose,
                          boost::any* pathData) override;

    virtual void initializeVisualization(MapEditor* editor) override;

    virtual void beginMapVisualization() override;

    virtual void endMapVisualization() override;

    virtual void visualizeEdge(const toponav_core::TopoMap::Edge* edge) override;

    virtual void appendTransitionToPlan(nav_msgs::Path* pathPtr, std::string transitionType,
                                        boost::any const& pathData) override;

    virtual void activate() override;
    virtual void deactivate() override;

    virtual void poseCallback(geometry_msgs::PoseStamped const& pose) override;

    static void createNewMapStep(toponav_core::TopoMap* map, std::string name,
                                 tf2::Stamped<tf2::Vector3> const& start,
                                 const tf2::Stamped<tf2::Vector3>& end);

    static void initEdgeData(toponav_core::TopoMap::Edge* edge, std::string stepName,
                             EdgeStepData::StepType stepType);

    virtual YAML::Node beginSaving(toponav_core::TopoMap* map) override;

    virtual YAML::Node saveEdgeData(toponav_core::TopoMap::Edge const* edge) override;

    virtual bool areEdgesCoupled(const toponav_core::TopoMap::Edge* edgeA,
                                 const toponav_core::TopoMap::Edge* edgeB) override;

    double getHeuristics(toponav_core::TopoMap::Edge const* edge) override;

    std::string getModuleType(const toponav_core::TopoMap::Edge* edge) override;

    virtual void parseEdgeInData(YAML::Node const& config,
                                 toponav_core::TopoMap::Edge* edge) override;

    virtual YAML::Node saveEdgeInData(toponav_core::TopoMap::Edge const* edge) override;

    /**
     * Update exit data of edge and the src/dst node according to 3d center and exit position
     * @param edge
     */
    virtual void onEdgeApproachChanged(const toponav_core::TopoMap::Edge* edge) override;

private:
    enum {
        IDLE,
        CREATING_EDGE_APPROACH,
        CREATING_EDGE_EXIT,
        CREATING_MAP_STEP,
        WAITING_STEP_SELECTION
    } vizState = IDLE;

    bool mDynamicStepsFromDetection;

    // for parsing
    toponav_core::TopoMap* currentMap = nullptr;
    MapStepData currentStepData;

    // for viz/editor
    std::string selectedStepName;
    tf2::Stamped<tf2::Transform> lastApproachPose;
    toponav_core::TopoMap::Node* lastApproachNode = nullptr;

    ros::Subscriber mFloorStepSub;

    static constexpr float TRACK_TOLERANCE_M = 0.5f;

    void createMapStepCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    void selectMapStepCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    void createEdgeCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    void setEdgeUpDown(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                       const toponav_core::TopoMap::Edge* data);

    void setApproachFromStep(const toponav_core::TopoMap::Edge* data);

    void visualizeMapStep(StepInfo* stepInfo);

    void processMapStepCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                          StepInfo* stepInfo, int type);

    void deleteStepCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                      StepInfo* stepInfo);

    void
    createBothStepEdgesCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                          StepInfo* stepInfo);

    void createBothStepEdges(StepInfo* stepInfo);

    tf2::Stamped<tf2::Transform>
    getExitPoseFromApproach(StepInfo const& stepInfo,
                            tf2::Stamped<tf2::Transform> const& approachPose);

    double getCosts(CostSettings const&);

    inline static const CostSettings& selectCosts(const StepInfo& entry,
                                                  EdgeStepData::StepType type) {
        return type == EdgeStepData::UP
                   ? entry.upCosts
                   : (type == EdgeStepData::DOWN
                          ? entry.downCosts
                          : (type == EdgeStepData::CROSS_OVER ? entry.crossOverCosts
                                                              : entry.crossGravelCosts));
    }

    void onFloorStepCb(const geometry_msgs::PolygonStamped& msg);

    void trackStep(const FloorStep2d& obsStep, ros::Time stamp);
    void clearArea(const std::vector<Point2d>& points);
};

typedef StepEdgeModule::Ptr StepEdgeModulePtr;

} // namespace toponav_ros
