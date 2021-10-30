#include "objects_to_grasp_detection.h"

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

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

ObjectSegmentationOutput detectObjectsInScene(const ObjectSegmentationInput& input,
                                              visualization_msgs::MarkerArray* markerArray)
{
  // filter indices in dense cloud according to plane coefficients
  const auto planeCoefficients = getPlaneCoefficientsToCamera(input.modelCoefficients.values);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> modelPlane(input.pointcloud);
  const auto outsideGroundIndices =
      selectOutsideDistance(modelPlane, planeCoefficients, 0.01, true);

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

  std::vector<pcl::Indices> allObjectIndices;

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

      std::vector<int> objectIndices;
      const cv::Rect2i objectRect(x, y, w, h);

      // find object's indices by going through all outliers and checking if they are inside object
      // rect.
      for (const int inlierIndex : outsideGroundIndices)
      {
        const unsigned int inlierX = inlierIndex % input.pointcloud->width;
        const unsigned int inlierY = inlierIndex / input.pointcloud->width;

        if (objectRect.contains(cv::Point2i(inlierX, inlierY)))
        {
          objectIndices.push_back(inlierIndex);
        }
      }

      allObjectIndices.emplace_back(std::move(objectIndices));
    }
  }

  cv::imshow("labeledMask", labeledMask);
  cv::waitKey(3);

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
      for (auto index : objectIndices)
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

    for (const auto& objectIndices : allObjectIndices)
    {
      pcl::CentroidPoint<pcl::PointXYZ> centroid3d;
      Eigen::Matrix<float, Eigen::Dynamic, 2> points2d(objectIndices.size(), 2);
      size_t pointsIndex = 0;

      float topZ = -1000;

      for (auto index : objectIndices)
      {
        const auto& p = input.pointcloud->at(index);
        const Eigen::Vector2f p2d {p.getVector3fMap().dot(baseX), p.getVector3fMap().dot(baseY)};
        const auto projected = baseX * p2d.x() +
                               baseY * p2d.y() + planeOrigin;

        const auto zOnPlane = p.getVector3fMap().dot(planeZ);
        topZ = std::max(topZ, zOnPlane);

        centroid3d.add(pcl::PointXYZ{ projected.x(), projected.y(), projected.z() });
        points2d.row(pointsIndex++) = p2d;

        geometry_msgs::Point pointMsg;
        pointMsg.x = projected.x();
        pointMsg.y = projected.y();
        pointMsg.z = projected.z();
        marker.points.emplace_back(pointMsg);
        marker.colors.emplace_back(color);
      }

      pcl::PointXYZ objectCenterPCL;
      centroid3d.get(objectCenterPCL);

      // perform PCA
      // https://stackoverflow.com/questions/15138634/eigen-is-there-an-inbuilt-way-to-calculate-sample-covariance
      const auto center2d = points2d.colwise().mean();
      Eigen::MatrixXf centered = points2d.rowwise() - center2d;
      Eigen::MatrixXf cov = (centered.adjoint() * centered) / float(points2d.rows() - 1);
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);



      const auto mainDirection = eig.eigenvectors().col(1).x() * baseX + eig.eigenvectors().col(1).y() * baseY;
      const auto objectCenter = center2d.x() * baseX + center2d.y() * baseY + planeZ * topZ;

      {
        visualization_msgs::Marker marker;
        marker.header = pcl_conversions::fromPCL(input.pointcloud->header);
        marker.ns = "objects_to_grasp_segmented_arrows";
        marker.id = &objectIndices - allObjectIndices.data();
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
      result.detectedObjects.emplace_back(std::move(object));
    }

    markerArray->markers.emplace_back(std::move(marker));
  }

  return result;
}