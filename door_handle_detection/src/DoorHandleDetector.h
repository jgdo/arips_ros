#pragma once

#include <string>
#include <optional>


#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

struct DoorHandleLocation {
    cv::Point2i handleStart;
    cv::Point2i handleEnd;
};

class DoorHandleDetector {
public:
    explicit DoorHandleDetector(const std::string& modelPath);

    std::optional<DoorHandleLocation> detect(const cv::Mat& image);

    static void annotateDetected(cv::Mat& image, const std::optional<DoorHandleLocation>& doorHandle);

private:

    cv::dnn::Net mDetectionNet;
    // cv::Mat1f mGridXX, mGridYY;
};



