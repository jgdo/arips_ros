#include <arips_arm_node/uniform_sample_filter.h>
#include <arips_arm_msgs/TrajectoryBufferCommand.h>
#include <arips_arm_msgs/MotionState.h>

class TrajectoryBufferHandler {
public:
    TrajectoryBufferHandler() {
        mBufferPublisher = nh.advertise<arips_arm_msgs::TrajectoryBufferCommand>("traj_buffer_command", 1, false);
    }

    void setNewTrajectory(trajectory_msgs::JointTrajectory const& traj) {
        mCurrentTrajectory = traj;

        publishPoints(0);
    }

    void updateBuffer(arips_arm_msgs::TrajectoryState const& state) {
        if(state.numPointsInBuffer < state.bufferCapacity * 3 / 4) {
            publishPoints(state.controlCycleCount);
        }
    }

private:
    const size_t MSG_BUF_SIZE = arips_arm_msgs::TrajectoryBufferCommand::_traj_points_type::size();

    ros::NodeHandle nh;
    ros::Publisher mBufferPublisher;

    trajectory_msgs::JointTrajectory mCurrentTrajectory;

    void publishPoints(size_t startIndex) {
        startIndex = std::min(startIndex, mCurrentTrajectory.points.size());
        size_t size = std::min(mCurrentTrajectory.points.size()-startIndex, MSG_BUF_SIZE);

        arips_arm_msgs::TrajectoryBufferCommandPtr bufCmd = boost::make_shared<arips_arm_msgs::TrajectoryBufferCommand>();
        bufCmd->start_index = startIndex;
        bufCmd->size = size;
        for(size_t i = 0; i < size; i++) {
            for(size_t j = 0; j < arips_arm_msgs::TrajectoryPoint::_positions_type::size(); j++) {
                bufCmd->traj_points.at(i).positions.at(j) = mCurrentTrajectory.points.at(startIndex + i).positions.at(j);
                bufCmd->traj_points.at(i).velocities.at(j) = mCurrentTrajectory.points.at(startIndex + i).velocities.at(j);
                bufCmd->traj_points.at(i).accelerations.at(j) = mCurrentTrajectory.points.at(startIndex + i).accelerations.at(j);
            }
        }
        mBufferPublisher.publish(bufCmd);
    }
};

class AripsArmNode {
public:
    AripsArmNode() {
        filter.configure();
        pub = nh.advertise<trajectory_msgs::JointTrajectory>("sampled_trajectory", 1, false);
        sub = nh.subscribe("trajectory", 1, &AripsArmNode::trajectoryCb, this);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    industrial_trajectory_filters::UniformSampleFilter filter;

    void trajectoryCb(const trajectory_msgs::JointTrajectory& trajOrig) {
        trajectory_msgs::JointTrajectory trajSampled;
        if(filter.update(trajOrig, trajSampled)) {
            pub.publish(trajSampled);
        }
    }
};

int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "arips_arm_node");

    AripsArmNode aan;
    ros::spin();
}