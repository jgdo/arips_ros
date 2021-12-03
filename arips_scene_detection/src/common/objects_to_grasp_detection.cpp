#include "objects_to_grasp_detection.h"

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>

inline tf2::Vector3 eigen2tf(const Eigen::Vector3f& v)
{
  return { v.x(), v.y(), v.z() };
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

cv::Mat projectPointsOntoImage(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                               const pcl::Indices& indices, const tf2::Transform& transform,
                               const cv::Rect2f& targetArea, float resolution,
                               float heightResolution)
{
  const auto resolutionInverse = 1.0 / resolution;
  const auto heightResolutionInverse = 1.0 / heightResolution;
  const int imageWidth = std::ceil(targetArea.size().width * resolutionInverse);
  const int imageHeight = std::ceil(targetArea.size().height * resolutionInverse);

  cv::Mat image(imageHeight, imageWidth, CV_8UC1, cv::Scalar::all(0));
  for (auto index : indices)
  {
    const auto& originalPoint = pointcloud[index];
    const auto transformedPoint =
        transform(tf2::Vector3(originalPoint.x, originalPoint.y, originalPoint.z));

    const int xImg = (transformedPoint.x() - targetArea.x) * resolutionInverse;
    const int yImg = (transformedPoint.y() - targetArea.y) * resolutionInverse;

    if (xImg >= 0 && xImg < imageWidth && yImg >= 0 && yImg < imageHeight)
    {
      const uint8_t valImg =
          std::clamp<int>((transformedPoint.z() + 0.2) * heightResolutionInverse, 0, 255);
      image.at<uint8_t>(yImg, xImg) = valImg;
    }
  }

  cv::imshow("projectedImageOrig", image);
  cv::waitKey(1);

  cv::Mat element5 = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, { 5, 5 });
  cv::morphologyEx(image, image, cv::MorphTypes::MORPH_CLOSE, element5);

  cv::Mat element3 = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, { 5, 5 });
  cv::morphologyEx(image, image, cv::MorphTypes::MORPH_OPEN, element3);

  /*
  // cluster contours in stencil image
  cv::Mat labels, stats, centroids;
  int label_count = cv::connectedComponentsWithStats(image, labels, stats, centroids, 8);

  cv::Mat labeledMask;
  cv::cvtColor(image, labeledMask, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < label_count; i++)
  {
    int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    int y = stats.at<int>(i, cv::CC_STAT_TOP);
    int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    int area = stats.at<int>(i, cv::CC_STAT_AREA);

      cv::rectangle(labeledMask, { x, y }, { x + w - 1, y + h - 1 }, { 0, 0, 255 }, 1);
  }

   */

  std::vector<std::vector<cv::Point>> contours;
  findContours(image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat labeledMask;
  cv::cvtColor(image, labeledMask, cv::COLOR_GRAY2BGR);

  std::vector<std::vector<cv::Point>> hull(contours.size());
  for (size_t i = 0; i < contours.size(); i++)
  {
    convexHull(contours[i], hull[i]);
  }

  for (size_t i = 0; i < contours.size(); i++)
  {
    cv::Scalar color = { 0, 0, 255 };  // cv::Scalar( rand()%256, rand()%256, rand()%256 );
    // drawContours( labeledMask, contours, (int)i, color );
    // drawContours( labeledMask, hull, (int)i, color, 2 );
    const auto rect = cv::minAreaRect(contours[i]);



    cv::Point2f vertices2f[4];
    rect.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    std::vector<cv::Point> rectPoints;
    for(int i = 0; i < 4; ++i)
    {
      rectPoints.push_back(vertices2f[i]);
    }

    cv::polylines( labeledMask, rectPoints, true, color, 2 );
  }

  cv::imshow("projectedImageMorphed", labeledMask);
  cv::waitKey(1);

  return image;
}

ObjectSegmentationOutput detectObjectsInScene(const ObjectSegmentationInput& input,
                                              visualization_msgs::MarkerArray* markerArray)
{
  // filter indices in dense cloud according to plane coefficients
  const auto planeCoefficients = getPlaneCoefficientsToCamera(input.modelCoefficients.values);
  const pcl::SampleConsensusModelPlane<pcl::PointXYZ> modelPlane(input.pointcloud);
  const auto outsideGroundIndices =
      selectOutsideDistance(modelPlane, planeCoefficients, 0.01, true);

  const auto projectedImage =
      projectPointsOntoImage(*input.pointcloud, outsideGroundIndices, input.cameraToBase,
                             cv::Rect2f{ 0.05, -0.3, 0.3, 0.6 }, 0.001, 0.0001);

  // create stencil image from indices
  cv::Mat imgMask =
      cv::Mat(cv::Size{ input.depthImage.cols, input.depthImage.rows }, CV_8U, cv::Scalar(0));

  for (auto i : outsideGroundIndices)
  {
    imgMask.data[i] = 255;
  }

  cv::Mat element3 = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, { 3, 3 });
  cv::morphologyEx(imgMask, imgMask, cv::MorphTypes::MORPH_OPEN, element3);

  cv::Mat element5 = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, { 5, 5 });
  cv::morphologyEx(imgMask, imgMask, cv::MorphTypes::MORPH_CLOSE, element5);

  // cluster contours in stencil image
  cv::Mat labels, stats, centroids;
  int label_count = cv::connectedComponentsWithStats(imgMask, labels, stats, centroids, 8);

  // extract object point indices for every counter
  cv::Mat labeledMask;
  cv::cvtColor(imgMask, labeledMask, cv::COLOR_GRAY2BGR);

  std::vector<std::pair<pcl::Indices, pcl::RGB>> allObjectIndices;

  for (int i = 0; i < label_count; i++)
  {
    int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    int y = stats.at<int>(i, cv::CC_STAT_TOP);
    int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    int area = stats.at<int>(i, cv::CC_STAT_AREA);

    // skip too big or too small objects
    if (w >= input.minObjectWidth && w <= input.maxObjectWidth && h >= input.minObjectHeight &&
        h <= input.maxObjectHeight)
    {
      cv::rectangle(labeledMask, { x, y }, { x + w - 1, y + h - 1 }, { 0, 0, 255 }, 3);

      pcl::CentroidPoint<pcl::RGB> colorCentroid;
      std::vector<int> objectIndices;
      const cv::Rect2i objectRect(x, y, w, h);

      // find object's indices by going through all outliers and checking if they are inside object
      // rect.
      for (const int inlierIndex : outsideGroundIndices)
      {
        const unsigned int inlierX = inlierIndex % input.pointcloud->width;
        const unsigned int inlierY = inlierIndex / input.pointcloud->width;
        const auto pt = cv::Point2i(inlierX, inlierY);

        if (objectRect.contains(pt))
        {
          objectIndices.push_back(inlierIndex);
          const auto color = input.colorImage.at<cv::Vec3b>(pt);
          colorCentroid.add(pcl::RGB(color[2], color[1], color[0]));
        }
      }

      pcl::RGB color;
      colorCentroid.get(color);
      allObjectIndices.emplace_back(std::move(objectIndices), color);
    }
  }

  cv::imshow("labeledMask", labeledMask);
  cv::waitKey(1);

  if (markerArray)
  {
    visualization_msgs::Marker marker;
    marker.header = pcl_conversions::fromPCL(input.pointcloud->header);
    marker.ns = "objects_to_grasp_segmented";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;

    std_msgs::ColorRGBA color;
    color.r = 1;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 0.5;

    for (const auto& objectIndices : allObjectIndices)
    {
      for (auto index : objectIndices.first)
      {
        const auto& point = input.pointcloud->at(index);
        geometry_msgs::Point pointMsg;
        pointMsg.x = point.x;
        pointMsg.y = point.y;
        pointMsg.z = point.z;
        marker.points.emplace_back(pointMsg);
        marker.colors.emplace_back(color);
      }
    }

    markerArray->markers.emplace_back(std::move(marker));
  }

  // project points onto plane
  ObjectSegmentationOutput result;

  if (markerArray)
  {
    visualization_msgs::Marker marker;
    marker.header = pcl_conversions::fromPCL(input.pointcloud->header);
    marker.ns = "objects_to_grasp_segmented_projected";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;

    std_msgs::ColorRGBA color;
    color.g = 1;
    color.a = 1;
    // create some base vectors for the plane, didn't check if its right- or left-handed
    const auto planeZ = planeCoefficients.head<3>();
    const auto someX = Eigen::Vector3f{ 1.0, 0, 0 };
    const auto baseX = planeZ.cross(someX).normalized();
    const auto baseY = planeZ.cross(baseX).normalized();
    const auto planeOrigin = planeZ * -planeCoefficients.w();

    for (const auto& objectIndicesColor : allObjectIndices)
    {
      const auto& objectIndices = objectIndicesColor.first;
      const auto objectColor = objectIndicesColor.second;

      pcl::CentroidPoint<pcl::PointXYZ> centroid3d;
      Eigen::Matrix<float, Eigen::Dynamic, 2> points2d(objectIndices.size(), 2);
      size_t pointsIndex = 0;

      float topHeight = -1000;

      for (auto index : objectIndices)
      {
        const auto& p = input.pointcloud->at(index);
        const Eigen::Vector2f p2d{ p.getVector3fMap().dot(baseX), p.getVector3fMap().dot(baseY) };
        const auto projectedOnPlane = baseX * p2d.x() + baseY * p2d.y() + planeOrigin;

        const auto heightAbovePlane = p.getVector3fMap().dot(planeZ);
        topHeight = std::max(topHeight, heightAbovePlane);

        centroid3d.add(p);
        points2d.row(pointsIndex++) = p2d;

        geometry_msgs::Point pointMsg;
        pointMsg.x = projectedOnPlane.x();
        pointMsg.y = projectedOnPlane.y();
        pointMsg.z = projectedOnPlane.z();
        marker.points.emplace_back(pointMsg);
        marker.colors.emplace_back(color);
      }

      pcl::PointXYZ objectCenterPCL;
      centroid3d.get(objectCenterPCL);

      if (topHeight > input.maxCenterHeightAboveFloor)
      {
        // object is too high to consider
        continue;
      }

      const auto centerTfInBase = input.cameraToBase(eigen2tf(objectCenterPCL.getVector3fMap()));
      if (centerTfInBase.length() > input.maxRadiusAroundBase)
      {
        continue;
      }

      const auto getRotation = [](tf2::Matrix3x3 const& rot) {
        tf2::Quaternion q;
        rot.getRotation(q);

        return Eigen::Quaterniond{ q.w(), q.x(), q.y(), q.z() }.cast<float>();
      };

      /*

      // get orientation
      Eigen::Affine3f cameraToBase {getRotation(input.cameraToBase.getBasis())};
      cameraToBase.translation() =
          Eigen::Vector3d{ input.cameraToBase.getOrigin().x(), input.cameraToBase.getOrigin().z(),
                           input.cameraToBase.getOrigin().z() }.cast<float>();

      float bestWidth = 1000;
      float bestAngle = 0;

      for(float angle = -M_PI_2; angle < M_PI_2; angle += M_PI_2/4)
      {
        const auto preRot = Eigen::AngleAxisf(angle, Eigen::Vector3f{0,0,1});

        float minX = 1000, maxX = -1000;
        for (auto index : objectIndices)
        {
          const auto& p = input.pointcloud->at(index);
          auto pBase = preRot*(cameraToBase * p.getVector3fMap());

          minX = std::min(minX, pBase.x());
          maxX = std::max(maxX, pBase.x());
        }
        const auto maxWidth = std::abs(maxX - minX);

        if(maxWidth < bestWidth) {
          bestWidth = maxWidth;
          bestAngle = angle;
        }
      }

      ROS_INFO_STREAM("Best angle: " << bestAngle << " width " << bestWidth);

       */

      // perform PCA
      // https://stackoverflow.com/questions/15138634/eigen-is-there-an-inbuilt-way-to-calculate-sample-covariance
      const auto center2d = points2d.colwise().mean();
      Eigen::MatrixXf centered = points2d.rowwise() - center2d;
      Eigen::MatrixXf cov = (centered.adjoint() * centered) / float(points2d.rows() - 1);
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

      const auto mainDirection =
          eig.eigenvectors().col(1).x() * baseX + eig.eigenvectors().col(1).y() * baseY;
      const auto objectCenter = center2d.x() * baseX + center2d.y() * baseY + planeZ * topHeight;

      {
        visualization_msgs::Marker marker;
        marker.header = pcl_conversions::fromPCL(input.pointcloud->header);
        marker.ns = "objects_to_grasp_segmented_arrows";
        marker.id = &objectIndicesColor - allObjectIndices.data();
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.002;
        marker.scale.y = 0.004;
        marker.color.r = 1;
        marker.color.a = 1;

        const auto arrow = mainDirection * 0.1 + objectCenter;
        geometry_msgs::Point pointMsg;
        pointMsg.x = objectCenter.x();
        pointMsg.y = objectCenter.y();
        pointMsg.z = objectCenter.z();
        marker.points.emplace_back(pointMsg);
        pointMsg.x = arrow.x();
        pointMsg.y = arrow.y();
        pointMsg.z = arrow.z();
        marker.points.emplace_back(pointMsg);

        markerArray->markers.emplace_back(std::move(marker));
      }

      ObjectInformation object;
      object.position.setOrigin(tf2::Vector3(objectCenter.x(), objectCenter.y(), objectCenter.z()));
      object.meanColor.r = objectColor.r / 255.0f;
      object.meanColor.g = objectColor.g / 255.0f;
      object.meanColor.b = objectColor.b / 255.0f;
      object.meanColor.a = 1.0;

      result.detectedObjects.emplace_back(std::move(object));
    }

    markerArray->markers.emplace_back(std::move(marker));
  }

  return result;
}