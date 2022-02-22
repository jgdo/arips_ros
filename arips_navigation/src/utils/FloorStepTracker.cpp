#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

#include <arips_navigation/path_planning/PlanningMath.h>
#include <arips_navigation/utils/FloorStepTracker.h>
#include <arips_navigation/utils/transforms.h>

struct TrackedStep {
    DetectedFloorStep step;
    size_t count = 1;

    static constexpr float GAMMA = 0.9f;
    static constexpr int MAX_COUNT = 100;

    static auto dist(const Point2d& lhs, const Point2d& rhs) { return (lhs - rhs).norm(); }

    static bool isClose(const Point2d& lhs, const Point2d& rhs, float tolerance) {
        return (lhs - rhs).norm() < tolerance;
    }

    bool isWithinTolerance(const DetectedFloorStep& obs, float tolerance) const {
        return (isClose(obs.at(0), step.at(0), tolerance) &&
                isClose(obs.at(1), step.at(1), tolerance)) ||
               (isClose(obs.at(0), step.at(1), tolerance) &&
                isClose(obs.at(1), step.at(0), tolerance));
    }

    using M = std::pair<int, int>;
    M bestMatch(const DetectedFloorStep& other) const {
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

    void track(const DetectedFloorStep& obs) {
        const auto best = bestMatch(obs);

        if ((best == M{0, 0}) || (best == M{1, 1})) {
            // 0 close to 0
            combine(obs);
        } else {

            combine({obs.at(1), obs.at(0)});
        }
    }

    void combine(const DetectedFloorStep& other) {
        for (size_t i = 0; i < step.size(); i++) {
            step[i] = step[i] * GAMMA + other[i] * (1.0F - GAMMA);
        }

        if (count < MAX_COUNT) {
            count++;
        }
    }
};

namespace {

struct PimplBase {
    const tf2_ros::Buffer& mTf;
    std::string mGlobalFrame;
    ros::Subscriber mFloorStepSub;
    ros::Publisher mMapPub;

    static constexpr float TRACK_TOLERANCE_M = 0.5f;

    explicit PimplBase(const tf2_ros::Buffer& tf, std::string globalFrame)
        : mTf{tf}, mGlobalFrame{std::move(globalFrame)} {
        ros::NodeHandle nh;
        mFloorStepSub = nh.subscribe("floor_step", 10, &PimplBase::onFloorStepCb, this);
        mMapPub = nh.advertise<visualization_msgs::Marker>("floor_step_map", 1);
    }

    void onFloorStepCb(const geometry_msgs::PolygonStamped& msg) {
        const auto trans = tryLookupTransform(mTf, mGlobalFrame, msg.header.frame_id);
        if (!trans) {
            return;
        }

        auto toP2d = [&trans](const geometry_msgs::Point32& p) {
            const auto pTrans = (*trans)(tf2::Vector3{p.x, p.y, p.z});
            return Point2d{pTrans.x(), pTrans.y()};
        };

        if (msg.polygon.points.size() == 4) {
            clearArea({toP2d(msg.polygon.points.at(0)), toP2d(msg.polygon.points.at(1)),
                       toP2d(msg.polygon.points.at(2)), toP2d(msg.polygon.points.at(3))});
        } else if (msg.polygon.points.size() == 6) {
            const DetectedFloorStep s{toP2d(msg.polygon.points.at(4)),
                                      toP2d(msg.polygon.points.at(5))};
            trackStep(s);
        } else {
            ROS_WARN_STREAM("FloorStepTracker received polygon with size "
                            << msg.polygon.points.size() << ", but expected 4 or 6. Ignoring.");
            return;
        }

        visualizeSteps();
    }

    virtual void trackStep(const DetectedFloorStep& obsStep) = 0;
    virtual void clearArea(const std::vector<Point2d>& points) = 0;
    virtual void visualizeSteps() = 0;

    template <class Container> void visualizeSteps(const Container& allSteps) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = mGlobalFrame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "floor_step_map";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        auto point = [](const Point2d& p) {
            geometry_msgs::Point msg;
            msg.x = p.x();
            msg.y = p.y();
            return msg;
        };

        for (const auto& s : allSteps) {
            if (s.count > 5) {
                marker.points.push_back(point(s.step.at(0)));
                marker.points.push_back(point(s.step.at(1)));
            }
        }

        mMapPub.publish(marker);
    }

    static bool polygonContains(const std::vector<Point2d>& points, Point2d test) {
        // https://stackoverflow.com/questions/8721406/how-to-determine-if-a-point-is-inside-a-2d-convex-polygon
        bool result = false;
        for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
            if ((points[i].y() > test.y()) != (points[j].y() > test.y()) &&
                (test.x() < (points[j].x() - points[i].x()) * (test.y() - points[i].y()) /
                                    (points[j].y() - points[i].y()) +
                                points[i].x())) {
                result = !result;
            }
        }
        return result;
    }
};

} // namespace

struct FloorStepTracker::Pimpl : public PimplBase {
    std::vector<TrackedStep> mAllSteps;

    explicit Pimpl(tf2_ros::Buffer& tf, std::string globalFrame,
                   const std::vector<DetectedFloorStep>& initialSteps)
        : PimplBase(tf, std::move(globalFrame)) {
        for (const auto& s : initialSteps) {
            mAllSteps.push_back({s, 10});
        }
    }

    void trackStep(const DetectedFloorStep& obsStep) override {
        for (auto& step : mAllSteps) {
            if (step.isWithinTolerance(obsStep, TRACK_TOLERANCE_M)) {
                step.track(obsStep);
                return;
            }
        }

        mAllSteps.push_back({obsStep});
    }

    void clearArea(const std::vector<Point2d>& points) override {
        for (auto& step : mAllSteps) {
            if (polygonContains(points, step.step[0]) && polygonContains(points, step.step[1])) {
                if (step.count > 0) {
                    step.count--;
                }
                ROS_INFO_STREAM("Clearning step ... " << step.count);
            }
        }

        mAllSteps.erase(std::remove_if(mAllSteps.begin(), mAllSteps.end(),
                                       [&](const auto& step) { return step.count == 0; }),
                        mAllSteps.end());
    }

    void visualizeSteps() override { PimplBase::visualizeSteps(mAllSteps); }
};

FloorStepTracker::FloorStepTracker(tf2_ros::Buffer& tf, std::string globalFrame,
                                   const std::vector<DetectedFloorStep>& initialSteps)
    : mPimpl{std::make_unique<Pimpl>(tf, std::move(globalFrame), initialSteps)} {}

std::vector<DetectedFloorStep> FloorStepTracker::allSteps() const {
    std::vector<DetectedFloorStep> steps;
    steps.reserve(mPimpl->mAllSteps.size());
    for (const auto& s : mPimpl->mAllSteps) {
        steps.push_back(s.step);
    }

    return steps;
}
void FloorStepTracker::visualizeSteps() { mPimpl->visualizeSteps(); }

FloorStepTracker::~FloorStepTracker() = default;

struct SingleFloorStepTracker::Pimpl: public PimplBase {
    std::optional<std::array<TrackedStep, 1>> theStep;

    using PimplBase::PimplBase;

    void trackStep(const DetectedFloorStep& obsStep) override {
        if (theStep) {
            if ((*theStep)[0].isWithinTolerance(obsStep, TRACK_TOLERANCE_M)) {
                (*theStep)[0].track(obsStep);
            }
        }
    }

    void clearArea(const std::vector<Point2d>& points) override {
        if (theStep) {
            if (polygonContains(points, (*theStep)[0].step[0]) && polygonContains(points, (*theStep)[0].step[1])) {
                if ((*theStep)[0].count > 0) {
                    (*theStep)[0].count--;
                }
                ROS_INFO_STREAM("Clearning step ... " << (*theStep)[0].count);
            }
        }
    }

    void visualizeSteps() override {
        if(theStep) {
            PimplBase::visualizeSteps(*theStep);
        }
    }
};

SingleFloorStepTracker::SingleFloorStepTracker(const tf2_ros::Buffer& tf, std::string globalFrame)
     :mPimpl{std::make_unique<Pimpl>(tf, std::move(globalFrame))}
{

}

SingleFloorStepTracker::~SingleFloorStepTracker() = default;

void SingleFloorStepTracker::track(const std::optional<DetectedFloorStep>& step) {
    if(step) {
        mPimpl->theStep = { TrackedStep{*step, 5} };
    } else {
        mPimpl->theStep = std::nullopt;
    }
}

std::optional<DetectedFloorStep> SingleFloorStepTracker::trackedStepPosition() const {
    if(mPimpl->theStep && mPimpl->theStep->at(0).count > 0) {
        return mPimpl->theStep->at(0).step;
    } else {
        return std::nullopt;
    }
}

void SingleFloorStepTracker::visualizeStep() {
    mPimpl->visualizeSteps();
}
