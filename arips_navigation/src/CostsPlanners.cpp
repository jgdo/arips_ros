#include <arips_navigation/CostsPlanners.h>
#include <arips_navigation/utils/ApproachLineArea.h>
#include <arips_navigation/utils/FixedPosition.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

toponav_ros::AripsFlatPlanner::AripsFlatPlanner(
    costmap_2d::Costmap2DROS& costmap,
    navfn::NavfnROS& planner,
    tf2_ros::Buffer& tfBuffer)
    : _costmap(costmap), _planner(planner), mTfBuffer{tfBuffer} {}

bool toponav_ros::AripsFlatPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                              ApproachExit3DPtr const& goal,
                                              std::vector<geometry_msgs::PoseStamped>& final_plan,
                                              double* costs,
                                              tf2::Stamped<tf2::Transform>* actualApproachPose) {
    std::string frame = _costmap.getGlobalFrameID();
    if (start.header.frame_id != frame) {
        ROS_ERROR_STREAM("Start passed to GridMapCostsPlanner must be in frame "
                         << frame << ", but is instead in frame " << start.header.frame_id);
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> initial_plan;

    bool ok = false;

    if (ApproachLineAreaConstPtr approachLine =
            std::dynamic_pointer_cast<const ApproachLineArea>(goal)) {
        ROS_WARN_STREAM("NavfnCostsPlanner only suppoert planning to center of ApproachLineArea.");
        geometry_msgs::PoseStamped goalMsg;
        tf2::toMsg(approachLine->getCenter(), goalMsg);

        if (goalMsg.header.frame_id != frame) {
            ROS_ERROR_STREAM("Goal passed to GridMapCostsPlanner must be in frame "
                             << frame << ", but is instead in frame " << goalMsg.header.frame_id);
            return false;
        }

        ok = _planner.makePlan(start, goalMsg, initial_plan);
    } else if (FixedPositionConstPtr fixedPose =
                   std::dynamic_pointer_cast<const FixedPosition>(goal)) {
        geometry_msgs::PoseStamped goalMsg;
        tf2::toMsg(fixedPose->getCenter(), goalMsg);

        const auto goalMsgMap = mTfBuffer.transform(goalMsg, frame);
        ok = _planner.makePlan(start, goalMsgMap, initial_plan);
    } else {
        ROS_ERROR_STREAM("GridMapCostsPlanner cannot handle goal type '" << typeid(*goal).name()
                                                                         << "'");
        return false;
    }

    if (ok) {
        float costs_f = 0;
        final_plan = initial_plan;
        // TODO smoothPath(initial_plan, start, &final_plan);

        double robot_width = 0.3; // TODO real value

        tf2::Stamped<tf2::Transform> lastPose, currentPos;
        lastPose.frame_id_ = std::string();
        tf2::fromMsg(start, lastPose);

        for (auto& pose : final_plan) {
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = frame;

            // add rotation to costs
            tf2::fromMsg(pose, currentPos);
            tf2::Vector3 d = currentPos.getOrigin() - lastPose.getOrigin();
            costs_f +=
                std::abs(lastPose.getRotation().angleShortestPath(currentPos.getRotation())) *
                robot_width / ((d.length() + robot_width) / robot_width) * 2;
            costs_f += d.length();
            lastPose = currentPos;
        }

        std::cout << " " << costs_f << std::endl;
        if (costs)
            *costs = costs_f;

        if (actualApproachPose)
            *actualApproachPose = lastPose;
    }

    return ok;
}

costmap_2d::Costmap2DROS & toponav_ros::AripsFlatPlanner::getMap() { return _costmap; }

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
