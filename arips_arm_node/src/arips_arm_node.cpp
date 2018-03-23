#include <arips_arm_node/uniform_sample_filter.h>
#include <arips_arm_msgs/TrajectoryBufferCommand.h>
#include <arips_arm_msgs/MotionState.h>
#include <arips_arm_msgs/MotionCommand.h>

#include <sensor_msgs/JointState.h>

template<typename>
struct array_size;

template<typename T, size_t N>
struct array_size<boost::array<T,N> > {
static size_t constexpr size = N;
};

static const size_t NUM_JOINTS = array_size<arips_arm_msgs::MotionState::_jointStates_type>::size;

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

        if(startIndex == 0) {
            bufCmd->size = mCurrentTrajectory.points.size();
        } else {
            bufCmd->size = size;
        }

        for(size_t i = 0; i < size; i++) {
            for(size_t j = 0; j < arips_arm_msgs::TrajectoryPoint::_goals_type::size(); j++) {
                bufCmd->traj_points.at(i).goals.at(j).position = mCurrentTrajectory.points.at(startIndex + i).positions.at(j);
                bufCmd->traj_points.at(i).goals.at(j).velocity = mCurrentTrajectory.points.at(startIndex + i).velocities.at(j);
                bufCmd->traj_points.at(i).goals.at(j).acceleration = mCurrentTrajectory.points.at(startIndex + i).accelerations.at(j);
            }
        }
        mBufferPublisher.publish(bufCmd);
    }
};

class AripsArmNode {
public:
    struct State {
        State(AripsArmNode* armNode): mArmNode(armNode) {}
        virtual ~State() {}

        virtual void enterState() {}
        virtual void onTick() = 0;
        virtual void onMotionState(const arips_arm_msgs::MotionState& msg) = 0;

    protected:
        AripsArmNode* mArmNode;
    };

    // robot is stopped
    struct StateIdle: public State {
        using State::State;

        virtual void enterState() override  {
            ROS_INFO_STREAM("Entering IDLE state");
        }

        void onTick() override {

        }

        void onMotionState(const arips_arm_msgs::MotionState &msg) override {

        }
    };

    // trajectory was requested, waiting for stop and traj. buffer is filled
    struct StateTrajWait: public State {
        using State::State;

        virtual void enterState() override  {
            ROS_INFO_STREAM("Entering TRAJ WAIT state");
        }

        void onTick() override {

        }

        void onMotionState(const arips_arm_msgs::MotionState &msg) override {
            if(msg.mode == arips_arm_msgs::MotionState::M_TRAJECTORY) {
                mArmNode->switchState(mArmNode->mStateTrajExec);
            }
        }
    };

    // trajectory is being executed
    struct StateTrajExec: public State {
        using State::State;

        virtual void enterState() override  {
            ROS_INFO_STREAM("Entering TRAJ EXEC state");
        }

        void onTick() override {
        }

        void onMotionState(const arips_arm_msgs::MotionState &msg) override {
            if(msg.mode == arips_arm_msgs::MotionState::M_BREAK) {
                mArmNode->switchState(mArmNode->mStateIdle);
            } else {
                mArmNode->mTrajBuffHandler.updateBuffer(msg.trajState);
            }
        }
    };

    AripsArmNode():
        mStateIdle(this),
        mStateTrajWait(this),
        mStateTrajExec(this)
    {
        filter.configure();
        pub = nh.advertise<trajectory_msgs::JointTrajectory>("sampled_trajectory", 1, false);
        mMotionCmdPub = nh.advertise<arips_arm_msgs::MotionCommand>("motion_command", 1, false);
        mJointStatePub = nh.advertise<sensor_msgs::JointState>("joint_states", 1, false);
        mTrajectorySub = nh.subscribe("trajectory", 1, &AripsArmNode::trajectoryCb, this);
        mMotionStateSub = nh.subscribe("motion_state", 1, &AripsArmNode::motionStateCb, this);

        mTimer = nh.createTimer(ros::Duration(0.3), &AripsArmNode::timerCb, this);
        switchState(mStateIdle);
    }
private:
    ros::NodeHandle nh;
    StateIdle mStateIdle;
    StateTrajWait mStateTrajWait;
    StateTrajExec mStateTrajExec;
    State* mCurrentState = nullptr;

    TrajectoryBufferHandler mTrajBuffHandler;
    ros::Subscriber mTrajectorySub, mMotionStateSub;
    ros::Publisher pub;
    ros::Publisher mMotionCmdPub;
    ros::Publisher mJointStatePub;
    industrial_trajectory_filters::UniformSampleFilter filter;
    arips_arm_msgs::MotionState mLastMotionState;

    ros::Timer mTimer;

    void trajectoryCb(const trajectory_msgs::JointTrajectory& trajOrig) {
        trajectory_msgs::JointTrajectory trajSampled;
        if(filter.update(trajOrig, trajSampled)) {
            pub.publish(trajSampled);
            mTrajBuffHandler.setNewTrajectory(trajSampled);
            arips_arm_msgs::MotionCommand cmd;
            cmd.command = arips_arm_msgs::MotionCommand::CMD_START_TRAJECTORY;
            mMotionCmdPub.publish(cmd);
            switchState(mStateTrajWait);
        }
    }

    void motionStateCb(const arips_arm_msgs::MotionState& msg) {
        if(mCurrentState)
            mCurrentState->onMotionState(msg);

        sensor_msgs::JointState jointState;

        jointState.header.stamp = ros::Time::now();

        jointState.name = { "joint1", "joint2", "joint3", "joint4", "joint5" };

        jointState.position.resize(NUM_JOINTS);
        jointState.velocity.resize(NUM_JOINTS);
        jointState.effort.resize(NUM_JOINTS);

        for(size_t i = 0; i < NUM_JOINTS; i++) {
            jointState.position.at(i) = msg.jointStates.at(i).position;
            jointState.velocity.at(i) = msg.jointStates.at(i).velocity;
            jointState.effort.at(i) = msg.jointStates.at(i).torque;
        }

        mJointStatePub.publish(jointState);
    }

    void timerCb(const ros::TimerEvent& event)
    {
        if(mCurrentState)
            mCurrentState->onTick();
    }

    void switchState(State& state) {
        mCurrentState = &state;
        mCurrentState->enterState();
    }
};

int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "arips_arm_node");

    AripsArmNode aan;
    ros::spin();
}