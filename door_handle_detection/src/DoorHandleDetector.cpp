#include "DoorHandleDetector.h"

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

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


DoorHandleDetector::DoorHandleDetector(const std::string& modelPath)
{
    mDetectionNet = cv::dnn::readNetFromONNX(modelPath);
    // createMeshgrid(mGridXX, mGridYY, {320, 240});
}

void non_maxima_suppression(const cv::Mat& image, cv::Mat& mask, bool remove_plateaus) {
  // find pixels that are equal to the local neighborhood not maximum (including 'plateaus')
  cv::dilate(image, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, {35, 35}));
  cv::compare(image, mask, mask, cv::CMP_GE);

  // optionally filter out pixels that are equal to the local minimum ('plateaus')
  if (remove_plateaus) {
    cv::Mat non_plateau_mask;
    cv::erode(image, non_plateau_mask, cv::Mat());
    cv::compare(image, non_plateau_mask, non_plateau_mask, cv::CMP_GT);
    cv::bitwise_and(mask, non_plateau_mask, mask);
  }
}

std::optional<DoorHandleLocation> DoorHandleDetector::detect(const cv::Mat &image) {
    cv::Mat floatImage;
    image.convertTo(floatImage, CV_32F);
    floatImage /= 255.0F;

    cv::Mat dnnInput({3, image.rows, image.cols}, CV_32F);

    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            dnnInput.at<float>(0, y, x) = floatImage.at<cv::Vec3f>(y, x)[0];
            dnnInput.at<float>(1, y, x) = floatImage.at<cv::Vec3f>(y, x)[1];
            dnnInput.at<float>(2, y, x) = floatImage.at<cv::Vec3f>(y, x)[2];
            //dnnInput.at<float>(3, y, x) = mGridXX.at<float>(y, x);
            //dnnInput.at<float>(4, y, x) = mGridYY.at<float>(y, x);
        }
    }

    dnnInput = dnnInput.reshape(0, {1, 3, image.rows, image.cols});

    mDetectionNet.setInput(dnnInput);
    const cv::Mat labels = mDetectionNet.forward();

    std::vector<cv::Mat> outputs;
    cv::dnn::imagesFromBlob(labels, outputs);

    assert(outputs.size() == 1);

    const cv::Mat& output = outputs.at(0);
    std::vector<cv::Mat> heatmaps;
    cv::split(output, heatmaps);

    if(heatmaps.size() == 1) {
      const auto& heatmap = heatmaps.at(0);

      cv::Mat mask = heatmap.clone();
      non_maxima_suppression(heatmap, mask, false);


      cv::imshow("mask", mask);
      cv::imshow("heatmap", heatmap);
      cv::waitKey(1);


      std::vector<cv::Point2i> points;

      for(int y = 0; y < mask.rows; y++) {
        for(int x = 0; x < mask.cols; x++) {
          if(mask.at<uint8_t>(y, x) > 127 && heatmap.at<float>(y,x) > 0.4) {
            points.emplace_back(x, y);
          }
        }
      }

      if(points.size() == 2) {
        return DoorHandleLocation{points.at(0), points.at(1)};
      } else {
        return std::nullopt;
      }
    }
    else if(heatmaps.size() == 2) {
      double handleStartMax, handleEndMax;
      cv::Point2i handleStart, handleEnd;
      cv::minMaxLoc(heatmaps.at(0), NULL, &handleStartMax, NULL, &handleStart);
      cv::minMaxLoc(heatmaps.at(1), NULL, &handleEndMax, NULL, &handleEnd);

      if (handleStartMax > 0.5 && handleEndMax > 0.5) {
        return DoorHandleLocation{handleStart, handleEnd};
      } else {
        return std::nullopt;
      }
    } else {
      assert(false);
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


