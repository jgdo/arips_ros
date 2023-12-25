#include <arips_navigation/CrossFloorStep.h>
#include <arips_navigation/NavigationContext.h>
#include <arips_navigation/utils/FloorStepTracker.h>

struct DemoNode {
    const ros::Rate mControlRate{10};

    DemoNode() {
        ros::NodeHandle nh;
        loopTimer = nh.createTimer(mControlRate, &DemoNode::doLoop, this);
    }

    void doLoop(const ros::TimerEvent& ev) {
        if(crossStep.isActive()) {
            crossStep.runCycle();
        } else if(!tracker.allSteps().empty()) {
            crossStep.activate(tracker.allSteps().at(0));
        } else {
            ROS_INFO("Waiting for a step to appear...");
        }
    }

    NavigationContext navigationContext;
    CrossFloorStep crossStep{navigationContext};
    FloorStepTracker tracker{navigationContext.tf,
                            navigationContext.localCostmap.getGlobalFrameID()};

    ros::Timer loopTimer;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_cross_step");
    ros::NodeHandle nh;

    DemoNode demo;

    while (ros::ok()) {
        ros::spin();
    }
}