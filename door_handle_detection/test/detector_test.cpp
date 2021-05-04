#include "DoorHandleDetector.h"

#include <gtest/gtest.h>
// #include <torch/torch.h>

#include <ros/ros.h>

TEST(DoorHandleDetector, detect) {
    DoorHandleDetector detector;

    cv::Mat image = cv::imread("/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/with_handle/frame_1619717262_996409107.jpg");
    const auto doorHandle = detector.detect(image);
    DoorHandleDetector::annotateDetected(image, doorHandle);

    cv::imshow("Detected", image);
    cv::waitKey(0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
