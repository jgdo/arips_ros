#include "DoorHandleDetector.h"

#include <ros/ros.h>

// see https://answers.opencv.org/question/11788/is-there-a-meshgrid-function-in-opencv/?sort=votes
static void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv,
                     cv::Mat1f &X, cv::Mat1f &Y) {
    cv::repeat(xgv.reshape(1, 1), ygv.total(), 1, X);
    cv::repeat(ygv.reshape(1, 1).t(), 1, xgv.total(), Y);
}

std::vector<float> linspace(float start, float end, int steps) {
    assert(steps > 1);

    std::vector<float> data;
    data.reserve(steps);

    const float step = (end - start) / (steps - 1);
    float f = start;
    for (int i = 0; i < steps; i++, f += step) {
        data.push_back(f);
    }
    return data;
}

// helper function (maybe that goes somehow easier)
static void createMeshgrid(cv::Mat1f &X, cv::Mat1f &Y, cv::Size2i size) {
    meshgrid(cv::Mat(linspace(-1.0F, 1.0F, size.width)),
             cv::Mat(linspace(-1.0F, 1.0F, size.height)), X, Y);
}

static int denormalizeCoords(float n, int size) {
    int x = static_cast<int>((n + 1) / 2 * size);
    return x;
}


DoorHandleDetector::DoorHandleDetector()
    : ModelPath {"/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/pytorch/models/heatmap.onnx"}
{
    mDetectionNet = cv::dnn::readNetFromONNX(ModelPath);
    createMeshgrid(mGridXX, mGridYY, {320, 240});
}

std::optional<DoorHandleLocation> DoorHandleDetector::detect(const cv::Mat &image) {
    cv::Mat floatImage;
    image.convertTo(floatImage, CV_32F);
    floatImage /= 255.0F;

    cv::Mat dnnInput({3, 240, 320}, CV_32F);

    for (int y = 0; y < 240; y++) {
        for (int x = 0; x < 320; x++) {
            dnnInput.at<float>(0, y, x) = floatImage.at<cv::Vec3f>(y, x)[0];
            dnnInput.at<float>(1, y, x) = floatImage.at<cv::Vec3f>(y, x)[1];
            dnnInput.at<float>(2, y, x) = floatImage.at<cv::Vec3f>(y, x)[2];
            //dnnInput.at<float>(3, y, x) = mGridXX.at<float>(y, x);
            //dnnInput.at<float>(4, y, x) = mGridYY.at<float>(y, x);
        }
    }

    dnnInput = dnnInput.reshape(0, {1, 3, 240, 320});

    mDetectionNet.setInput(dnnInput);
    const cv::Mat labels = mDetectionNet.forward();

    ROS_INFO_STREAM("Labels size: " << labels.size);

    std::vector<cv::Mat> outputs;
    cv::dnn::imagesFromBlob(labels, outputs);

    assert(outputs.size() == 1);

    const cv::Mat& output = outputs.at(0);
    std::vector<cv::Mat> heatmaps;
    cv::split(output, heatmaps);
    assert(heatmaps.size() == 2);

    double handleStartMax, handleEndMax;
    cv::Point2i handleStart, handleEnd;
    cv::minMaxLoc(heatmaps.at(0), NULL, &handleStartMax, NULL, &handleStart );
    cv::minMaxLoc(heatmaps.at(1), NULL, &handleEndMax, NULL, &handleEnd );

    if(handleStartMax > 0.5 && handleEndMax > 0.5) {
        return DoorHandleLocation {handleStart, handleEnd};
    } else {
        return std::nullopt;
    }
}

void DoorHandleDetector::annotateDetected(cv::Mat &image, const std::optional<DoorHandleLocation> &doorHandle) {
    if (doorHandle) {
        cv::circle(image, doorHandle->handleStart, 3, {0, 0, 255}, -1);
        cv::circle(image, doorHandle->handleEnd, 3, {0, 255, 0}, -1);
    } else {
        cv::circle(image, {image.cols / 2, image.rows / 2},
                   50, {0, 255, 255});
    }
}


