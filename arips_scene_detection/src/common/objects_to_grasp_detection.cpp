#include "objects_to_grasp_detection.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>

inline tf2::Vector3 eigen2tf(const Eigen::Vector3f& v)
{
  return { v.x(), v.y(), v.z() };
}

inline Eigen::Vector3f tf2eigen(const tf2::Vector3& v)
{
  return { float(v.x()), float(v.y()), float(v.z()) };
}

template <typename PointT>
pcl::Indices selectOutsideDistance(const pcl::SampleConsensusModelPlane<PointT>& model_plane,
                                   const Eigen::VectorXf& model_coefficients,
                                   const double threshold, bool nearerOnly)
{
  pcl::Indices result;

  // Needs a valid set of model coefficients
  if (!model_plane.isModelValid(model_coefficients))
  {
    PCL_ERROR("[pcl::SampleConsensusModelPlane::selectWithinDistance] Given model is invalid!\n");
    return result;
  }

  result.reserve(model_plane.indices_->size());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (std::size_t i = 0; i < model_plane.indices_->size(); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f pt((*model_plane.input_)[(*model_plane.indices_)[i]].x,
                       (*model_plane.input_)[(*model_plane.indices_)[i]].y,
                       (*model_plane.input_)[(*model_plane.indices_)[i]].z, 1.0f);

    float distance = model_coefficients.dot(pt);
    if (!nearerOnly)
    {
      distance = std::abs(distance);
    }

    if (distance > threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      result.push_back((*model_plane.indices_)[i]);
    }
  }

  return result;
}

static Eigen::Vector4f getPlaneCoefficientsToCamera(const std::vector<float>& coefficients)
{
  Eigen::Vector4f coeff(coefficients.data());
  if (coeff[2] > 0)
  {
    coeff *= -1;  // reverse of pointing away from camera
  }

  return coeff;
}

std::pair<cv::Mat, cv::Mat> projectPointsOntoImage(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                                                   const pcl::Indices& indices,
                                                   const tf2::Transform& transform,
                                                   const cv::Rect2f& targetArea, float resolution,
                                                   float heightResolution)
{
  const auto resolutionInverse = 1.0 / resolution;
  const auto heightResolutionInverse = 1.0 / heightResolution;
  const int imageWidth = std::ceil(targetArea.size().width * resolutionInverse);
  const int imageHeight = std::ceil(targetArea.size().height * resolutionInverse);

  cv::Mat image(imageHeight, imageWidth, CV_8UC1, cv::Scalar::all(0));
  cv::Mat indexImage(imageHeight, imageWidth, CV_32SC1, cv::Scalar::all(0));

  for (auto index : indices)
  {
    const auto& originalPoint = pointcloud[index];
    const auto transformedPoint =
        transform(tf2::Vector3(originalPoint.x, originalPoint.y, originalPoint.z));

    const auto xImg = int((transformedPoint.x() - targetArea.x) * resolutionInverse);
    const auto yImg = int((transformedPoint.y() - targetArea.y) * resolutionInverse);

    if (xImg >= 0 && xImg < imageWidth && yImg >= 0 && yImg < imageHeight)
    {
      const uint8_t valImg =
          std::clamp<int>(int(transformedPoint.z() * heightResolutionInverse), 0, 255);
      image.at<uint8_t>(yImg, xImg) = valImg;
      indexImage.at<int32_t>(yImg, xImg) = index;
    }
  }

  cv::imshow("projectedImageOrig", image);
  cv::waitKey(1);

  cv::Mat element5 = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, { 5, 5 });
  cv::morphologyEx(image, image, cv::MorphTypes::MORPH_CLOSE, element5);

  cv::Mat element3 = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, { 5, 5 });
  cv::morphologyEx(image, image, cv::MorphTypes::MORPH_OPEN, element3);

  return { std::move(image), std::move(indexImage) };
}

tf2::Transform calcTransformFloorToCamera(const tf2::Transform& baseToCamera,
                                          const Eigen::VectorXf& floorModelCoefficients)
{
  const auto planeNormal = eigen2tf(floorModelCoefficients.head<3>());
  const float distanceToPlane =
      floorModelCoefficients.w() + planeNormal.dot(baseToCamera.getOrigin());
  const auto floorOrigin = baseToCamera.getOrigin() - (planeNormal * distanceToPlane);

  const auto baseToCameraRot = baseToCamera.getBasis();
  const auto floorY = planeNormal.cross(baseToCameraRot.getColumn(0)).normalized();
  const auto floorX = floorY.cross(planeNormal).normalized();

  // clang-format off
  const tf2::Matrix3x3 floorRotationMatrix(floorX.x(), floorY.x(), planeNormal.x(),
                                           floorX.y(), floorY.y(), planeNormal.y(),
                                           floorX.z(), floorY.z(), planeNormal.z());
  // clang-format on

  return tf2::Transform{ floorRotationMatrix, floorOrigin };
}

// projectedImage: U8C1
// indexImage: S32C1, same size as indexImage, values are indices to colorImage
// colorImage: U8C3 rgb image
// contour: inside projectedImage and indexImage
static std_msgs::ColorRGBA extractObjectColor(const cv::Mat& projectedImage,
                                              const cv::Mat& indexImage, const cv::Mat& colorImage,
                                              const std::vector<cv::Point>& contour)
{
  const auto boundingBox = cv::boundingRect(contour);
  pcl::CentroidPoint<pcl::RGB> colorCentroid;
  for (int y = boundingBox.y; y < boundingBox.y + boundingBox.height; y++)
  {
    for (int x = boundingBox.x; x < boundingBox.x + boundingBox.width; x++)
    {
      if (projectedImage.at<uint8_t>(y, x) > 0)
      {
        const auto& pointColor = colorImage.at<cv::Vec3b>(indexImage.at<int32_t>(y, x));
        colorCentroid.add(pcl::RGB(pointColor[2], pointColor[1], pointColor[0]));
      }
    }
  }
  pcl::RGB objectColor;
  colorCentroid.get(objectColor);

  std_msgs::ColorRGBA res;
  res.a = 1.0;
  res.r = objectColor.r / 255.f;
  res.g = objectColor.g / 255.f;
  res.b = objectColor.b / 255.f;
  return res;
}

ObjectSegmentationOutput detectObjectsInScene(const ObjectSegmentationInput& input,
                                              visualization_msgs::MarkerArray* markerArray)
{
  // filter indices in dense cloud according to plane coefficients
  const auto planeCoefficients = getPlaneCoefficientsToCamera(input.modelCoefficients.values);
  const pcl::SampleConsensusModelPlane<pcl::PointXYZ> modelPlane(input.pointcloud);
  const auto outsideGroundIndices =
      selectOutsideDistance(modelPlane, planeCoefficients, 0.01, true);

  const auto floorToCamera =
      calcTransformFloorToCamera(input.cameraToBase.inverse(), planeCoefficients);

  if (markerArray)
  {
    visualization_msgs::Marker marker;
    marker.header = pcl_conversions::fromPCL(input.pointcloud->header);
    marker.ns = "plane";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.color.r = 1;
    marker.color.a = 1;

    std_msgs::ColorRGBA color;
    geometry_msgs::Point pointMsg;

    color.r = 1;
    color.g = 1;
    color.b = 1;
    color.a = 1;
    tf2::toMsg(floorToCamera(tf2::Vector3{ 0, 0, 0 }), pointMsg);
    marker.points.emplace_back(pointMsg);
    marker.colors.emplace_back(color);

    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    tf2::toMsg(floorToCamera(tf2::Vector3{ 0.1, 0, 0 }), pointMsg);
    marker.points.emplace_back(pointMsg);
    marker.colors.emplace_back(color);

    color.r = 0;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    tf2::toMsg(floorToCamera(tf2::Vector3{ 0, 0.1, 0 }), pointMsg);
    marker.points.emplace_back(pointMsg);
    marker.colors.emplace_back(color);

    color.r = 0;
    color.g = 0;
    color.b = 1;
    color.a = 1;
    tf2::toMsg(floorToCamera(tf2::Vector3{ 0, 0, 0.1 }), pointMsg);
    marker.points.emplace_back(pointMsg);
    marker.colors.emplace_back(color);

    markerArray->markers.emplace_back(std::move(marker));
  }

  constexpr auto projectionResolution = 0.001;
  const auto projectionAreaXY = cv::Rect2f{ 0.05, -0.3, 0.3, 0.6 };

  const auto [projectedImage, indexImage] =
      projectPointsOntoImage(*input.pointcloud, outsideGroundIndices, floorToCamera.inverse(),
                             projectionAreaXY, projectionResolution, projectionResolution);

  std::vector<std::vector<cv::Point>> contours;
  findContours(projectedImage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat labeledMask;
  cv::cvtColor(projectedImage, labeledMask, cv::COLOR_GRAY2BGR);

  std::vector<std::vector<cv::Point>> hull(contours.size());
  for (size_t i = 0; i < contours.size(); i++)
  {
    convexHull(contours[i], hull[i]);
  }

  ObjectSegmentationOutput result;

  for (auto& contour : contours)
  {
    const cv::Scalar drawColor = { 0, 0,
                                   255 };  // cv::Scalar( rand()%256, rand()%256, rand()%256 );
    // drawContours( labeledMask, contours, (int)i, color );
    // drawContours( labeledMask, hull, (int)i, color, 2 );
    const auto rect = cv::minAreaRect(contour);

    // choose size and angle such that width is always bigger than height
    const cv::Size2f realSize{
      static_cast<float>(std::max(rect.size.width, rect.size.height) * projectionResolution),
      static_cast<float>(std::min(rect.size.width, rect.size.height) * projectionResolution)
    };
    const auto correctedAngle =
        (rect.size.width >= rect.size.height ? rect.angle : rect.angle + 90.0) / 180 * M_PI;

    // skip too big or too small objects
    if (realSize.area() < input.minObjectArea || realSize.area() > input.maxObjectArea)
    {
      continue;
    }

    cv::Point2f vertices2f[4];
    rect.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    std::vector<cv::Point> rectPoints(vertices2f, vertices2f + 4);
    cv::polylines(labeledMask, rectPoints, true, drawColor, 2);

    const auto objCenterXY = rect.center * projectionResolution + projectionAreaXY.tl();
    const auto objHeight = projectedImage.at<uint8_t>(rect.center) * projectionResolution;

    tf2::Stamped<tf2::Transform> objTrans;
    objTrans.setOrigin({ objCenterXY.x, objCenterXY.y, objHeight / 2 });

    objTrans.setRotation(tf2::Quaternion(tf2::Vector3{ 0, 0, 1 }, correctedAngle));

    // angle is pointing along the longer side (-> x), but we need to make sure
    // that x is always pointing away from arm
    if ((input.cameraToBase.getRotation() * floorToCamera.getRotation() * objTrans.getRotation() *
         tf2::Vector3(1, 0, 0))
            .x() > 0.0)
    {
      // rotate by 180° if x would point towards arm
      objTrans.setRotation(tf2::Quaternion(tf2::Vector3{ 0, 0, 1 }, correctedAngle + M_PI));
    }

    ObjectInformation info;
    info.position = { floorToCamera * objTrans,
                      pcl_conversions::fromPCL(input.pointcloud->header.stamp),
                      input.pointcloud->header.frame_id };
    info.size = { realSize.width, realSize.height, objHeight };
    info.meanColor = extractObjectColor(projectedImage, indexImage, input.colorImage, contour);

    result.detectedObjects.emplace_back(std::move(info));
  }

  cv::imshow("projectedImageMorphed", labeledMask);
  cv::waitKey(1);

  if (markerArray)
  {
    // first delete old markers
    {
      visualization_msgs::Marker marker;
      marker.header = pcl_conversions::fromPCL(input.pointcloud->header);
      marker.ns = "objects_to_grasp";
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::DELETEALL;
      markerArray->markers.push_back(marker);
    }

    for (const auto& obj : result.detectedObjects)
    {
      visualization_msgs::Marker marker;
      marker.header = pcl_conversions::fromPCL(input.pointcloud->header);
      marker.ns = "objects_to_grasp";
      marker.id = markerArray->markers.size();
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.scale = tf2::toMsg(obj.size);
      tf2::toMsg(obj.position, marker.pose);
      marker.color = obj.meanColor;

      markerArray->markers.push_back(marker);

      marker.id = markerArray->markers.size();
      marker.type = visualization_msgs::Marker::ARROW;
      marker.scale = tf2::toMsg(tf2::Vector3(obj.size.x() + 0.015, 0.005, 0.01));
      markerArray->markers.push_back(marker);
    }
  }

  return result;
}