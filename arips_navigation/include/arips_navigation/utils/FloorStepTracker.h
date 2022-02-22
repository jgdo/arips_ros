#pragma once

#include <memory>
#include <string>

#include <tf2_ros/buffer.h>

#include <arips_navigation/path_planning/PlanningMath.h>

using DetectedFloorStep = std::array<Point2d, 2>;

class FloorStepTracker {
public:
    FloorStepTracker(tf2_ros::Buffer& tf, std::string globalFrame,
                     const std::vector<DetectedFloorStep>& initialSteps = {});
    ~FloorStepTracker();

    std::vector<DetectedFloorStep> allSteps() const;

    void visualizeSteps();

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};

class SingleFloorStepTracker {
public:
    SingleFloorStepTracker(const tf2_ros::Buffer& tf, std::string globalFrame);
    ~SingleFloorStepTracker();


    void track(const std::optional<DetectedFloorStep>& step);
    std::optional<DetectedFloorStep> trackedStepPosition() const;

    void visualizeStep();

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};

