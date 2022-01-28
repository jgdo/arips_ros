#include <arips_navigation/CostsPlanners.h>
#include <arips_navigation/utils/ApproachLineArea.h>
#include <arips_navigation/utils/FixedPosition.h>

#include "arips_navigation/path_planning/Costmap2dView.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace toponav_ros;

std::optional<double>
AripsFlatPlanner::computeCosts(const geometry_msgs::PoseStamped& start,
                               ApproachExit3DPtr const& goal,
                               tf2::Stamped<tf2::Transform>* actualApproachPose,
                               std::vector<geometry_msgs::PoseStamped>* path) {
    const auto frame = mContext.globalCostmap.getGlobalFrameID();
    if (start.header.frame_id != frame) {
        ROS_ERROR_STREAM("Start passed to GridMapCostsPlanner must be in frame "
                         << frame << ", but is instead in frame " << start.header.frame_id);
        return false;
    }

    geometry_msgs::PoseStamped goalMsg;
    tf2::toMsg(goal->getCenter(), goalMsg);
    goalMsg = mContext.tf.transform(goalMsg, frame);

    const auto start2d = Pose2D::fromMsg(start.pose);
    const auto potmap = mLocomotion.makePlan(Costmap2dView{mContext.globalCostmap, start2d},
                                             start2d, Pose2D::fromMsg(goalMsg.pose));

    if (!potmap) {
        return {};
    }

    if (actualApproachPose) {
        tf2::fromMsg(goalMsg, *actualApproachPose);
    }

    if (path) {
        path->clear();
        const auto path2d = potmap->traceCurrentPath(start2d);
        geometry_msgs::PoseStamped pathPoint;
        pathPoint.header.frame_id = frame;
        pathPoint.header.stamp = ros::Time::now();

        for (const auto& p2d : path2d) {
            pathPoint.pose = p2d.toPoseMsg();
            // header stays same
            path->emplace_back(pathPoint);
        }
    }

    return potmap->atPos(potmap->goal().point);
}

#if 0
/**
 *
 * @param path_in
 * @param path_out must be different than &path_in !!! Header will be empty
 */
static void smoothPath(std::vector<geometry_msgs::PoseStamped> const& path_in, const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped>* path_out) {
  // convert to eigen
  deque_vec3 in_poses;
  Eigen::Vector3d v_eigen;
  for(auto& e: path_in) {
    tf2::pointMsgToEigen(e.pose.position, v_eigen);
    in_poses.push_back(v_eigen);
  }
  
  // do smooting
  MotionParameters params;
  params.y_symmetry = false; // TODO really?
  Pathsmoother3D smooth(false, &params);
  Eigen::Quaterniond start_q, end_q;
  tf2::quaternionMsgToEigen(start.pose.orientation, start_q);
  tf2::quaternionMsgToEigen(path_in.back().pose.orientation, end_q);
  vector_vec3 out_poses;
  vector_quat out_rots;
  smooth.smooth(in_poses, start_q, end_q, out_poses, out_rots, false);
  
  // convert result
  path_out->clear();
  path_out->reserve(out_poses.size());
  // std_msgs::Header const& hdr = path_in.front().header;
  auto v_iter = out_poses.begin();
  auto r_iter = out_rots.begin();
  for(; v_iter != out_poses.end() && r_iter != out_rots.end(); ++v_iter, ++r_iter) {
    path_out->emplace_back();
    // path_out->back().header = hdr;
    tf2::pointEigenToMsg(*v_iter, path_out->back().pose.position);
    tf2::quaternionEigenToMsg(*r_iter, path_out->back().pose.orientation);
  }
}
#endif
