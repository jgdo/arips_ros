#include <arips_arm_node/uniform_sample_filter.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/Float32.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>

#include "SCServo.h"

static const size_t NUM_JOINTS = 5;

class RosSCSServo {
public:
    RosSCSServo(SCServo* servo, std::string const& ns):
            mServo(servo)
    {
        ros::NodeHandle pnh("~" + ns);
        ros::NodeHandle nh(ns);

        pnh.param("id", mID, -1);

        if(mID < 1 || mID > 254) {
            throw std::range_error("RosSCSServo: servo ID must be in [1..254], is " + std::to_string(mID));
        }

        if(!pnh.getParam("angles", mAngleDegTable) || !pnh.getParam("raws", mRawTable) || mAngleDegTable.size() != mRawTable.size() || mAngleDegTable.size() < 2) {
            ROS_WARN_STREAM("Using default angle table for SCS servo '" << ns << "' (ID: " << mID << ").");

            mAngleDegTable = { -120.0F, 120.0F};
            mRawTable = {100, 1023-100};
        }

        pnh.param("angle_offset", mAngleOffset_deg, 0.0F);

        pnh.param("angle_factor", mAngleFactor, 1.0F);

        setByteRegFromParam(pnh, P_CCW_DEAD, "ccw_dead");
        setByteRegFromParam(pnh, P_CW_DEAD, "cw_dead");

        setByteRegFromParam(pnh, P_COMPLIANCE_P, "kp");
        setByteRegFromParam(pnh, P_COMPLIANCE_I, "ki");
        setByteRegFromParam(pnh, P_INTEGRAL_LIMIT_L, "isum_max");

        mPosPub = nh.advertise<std_msgs::Float32>("pos_deg", 3);
        mPosSub = nh.subscribe("setpoint_deg", 1, &RosSCSServo::onSetposReceived, this);
    }

    float updateState() {
        // todo read servo
        auto raw = mServo->ReadPos(mID);

        float pos = rawToDegree(raw);
        std_msgs::Float32 msg;
        msg.data = pos;
        mPosPub.publish(msg);
        return pos;
    }

    void setPos_deg(float pos) {
        mSetPos = pos;
        mServo->WritePos(mID, degreeToRaw(mSetPos), 500);
    }

private:
    SCServo* mServo = nullptr;

    int mID = -1;

    std::vector<float> mAngleDegTable; // must contain at least 2 entries
    std::vector<int> mRawTable; // same size as mRawTable

    ros::Subscriber mPosSub;
    ros::Publisher mPosPub;

    float mAngleOffset_deg = 0.0F;
    float mAngleFactor = 1.0F;

    float mSetPos = 0.0F;


    void onSetposReceived(const std_msgs::Float32& msg) {
        setPos_deg(msg.data);
    }

    template <class Tin, class Tout>
    static Tout convertAngle(Tin value, std::vector<Tin> const& sourceTable, std::vector<Tout> const& dstTable) {
        size_t idx = 0;
        for(idx = 0; idx < sourceTable.size()-2; idx++) {
            if(value < sourceTable.at(idx+1))
                break;
        }

        float alpha = (float)(value - sourceTable.at(idx))/(float)(sourceTable.at(idx+1) - sourceTable.at(idx));
        return dstTable.at(idx) * (1.0f - alpha) + dstTable.at(idx+1) * alpha;
    };

    int degreeToRaw(float deg) const {
        return convertAngle((deg + mAngleOffset_deg) * mAngleFactor, mAngleDegTable, mRawTable);
    }

    float rawToDegree(int raw) const {
        return mAngleFactor * convertAngle(raw, mRawTable, mAngleDegTable) - mAngleOffset_deg;
    }

    void setByteRegFromParam(ros::NodeHandle& pnh, int reg, const std::string& name) {
        int value;
        if(pnh.getParam(name, value)) {
            mServo->writeByte(mID, reg, value);
        }
    }
};

class AripsArmNode {
public:
    AripsArmNode():
        mJointTrajActionServer(nh, "/arips_arm_controller/follow_joint_trajectory", false),
        mGripperActionServer(nh, "/arips_gripper_controller/gripper_action", false)
    {
        mServos.reserve(NUM_JOINTS+1);
        for(int i = 0; i < NUM_JOINTS+1; i++) {
            mServos.emplace_back(&mServoDevice, "servo_" + std::to_string(i));
        }

        filter.configure();
        pub = nh.advertise<trajectory_msgs::JointTrajectory>("sampled_trajectory", 3, false);
        mJointStatePub = nh.advertise<sensor_msgs::JointState>("joint_states", 3, false);

        mTimer = nh.createTimer(ros::Duration(0.05), &AripsArmNode::timerCb, this);

        mJointTrajActionServer.registerGoalCallback(boost::bind(&AripsArmNode::trajectoryActionGoalCB, this));
        mJointTrajActionServer.registerPreemptCallback(boost::bind(&AripsArmNode::trajectoryActionPreemptCB, this));

        mJointTrajActionServer.start();

        mGripperActionServer.registerGoalCallback(boost::bind(&AripsArmNode::gripperActionGoalCB, this));
        // mGripperActionServer.registerPreemptCallback(boost::bind(&AripsArmNode::gripperActionPreemptCB, this));

        mGripperActionServer.start();
    }
private:
    ros::NodeHandle nh;

    SCServo mServoDevice;

    std::vector<RosSCSServo> mServos;

    ros::Publisher pub, mJointStatePub;

    industrial_trajectory_filters::UniformSampleFilter filter;

    ros::Timer mTimer;

    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> mJointTrajActionServer;
    control_msgs::FollowJointTrajectoryGoalConstPtr mCurrentTrajGoal;

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> mGripperActionServer;

    /**
     * Next points index of mSampledTrajectory
     * Inactive if negative, otherwise [0, traj size], if == traj size, this means the last trajectory point was send and trajectory will be finished at next cycle
     */
    int mNextTrajectoryExecutionIndex = -1;

    trajectory_msgs::JointTrajectory mSampledTrajectory;

    void startNewSampledTrajectory(const trajectory_msgs::JointTrajectory& trajSampled) {
        mSampledTrajectory = trajSampled;
        mNextTrajectoryExecutionIndex = 0;
        pub.publish(trajSampled);
    }

    void timerCb(const ros::TimerEvent& event)
    {
        if(mJointTrajActionServer.isActive() && mNextTrajectoryExecutionIndex >= 0) {
            if(mNextTrajectoryExecutionIndex < mSampledTrajectory.points.size()) {
                ROS_INFO_STREAM("Sending trajectory point " << mNextTrajectoryExecutionIndex);
                auto const& point = mSampledTrajectory.points.at(mNextTrajectoryExecutionIndex);
                for(size_t i = 0; i < point.positions.size(); i++) {
                    mServos.at(i).setPos_deg(point.positions.at(i) *180.0F / M_PI);
                }
                mNextTrajectoryExecutionIndex++;

            } else {
                ROS_INFO_STREAM("Sending trajectory finished");
                // trajectory finished
                control_msgs::FollowJointTrajectoryResult res;
                res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
                mJointTrajActionServer.setSucceeded(res);
                mNextTrajectoryExecutionIndex = -1;
            }
        }

        sensor_msgs::JointState jointState;

        jointState.header.stamp = ros::Time::now();

        jointState.name = { "joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint"};

        jointState.position.resize(NUM_JOINTS + 1);
        jointState.velocity.resize(NUM_JOINTS + 1);
        jointState.effort.resize(NUM_JOINTS + 1);


        for(size_t i = 0; i < NUM_JOINTS; i++) {
            jointState.position.at(i) = mServos.at(i).updateState() * M_PI / 180.0F;
        }

        jointState.position.at(NUM_JOINTS) = -mServos.at(NUM_JOINTS).updateState();

        mJointStatePub.publish(jointState);
    }

    void trajectoryActionGoalCB() {
        mCurrentTrajGoal = mJointTrajActionServer.acceptNewGoal();

        trajectory_msgs::JointTrajectory trajSampled;
        if(filter.update(mCurrentTrajGoal->trajectory, trajSampled)) {
            ROS_INFO_STREAM("Starting new trajectory");
            startNewSampledTrajectory(trajSampled);
        } else {
            ROS_INFO_STREAM("Trajectory resampling failed, aborting trajectory goal");
            control_msgs::FollowJointTrajectoryResult res;
            res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
            res.error_string = ros::this_node::getName() + ": Aborted goal since could not resample the trajectory.";
            mJointTrajActionServer.setAborted(res, res.error_string);
            cancelCurrentTrajectory();
        }
    }

    void trajectoryActionPreemptCB() {
        ROS_INFO_STREAM("trajectoryActionPreemptCB().");
        cancelCurrentTrajectory();
        mJointTrajActionServer.setAborted();
    }

    void gripperActionGoalCB() {
        auto gripperGoal = mGripperActionServer.acceptNewGoal();

        // startNewGripperTrajectory(mCurrentGripperGoal->command.position);

        mServos.at(NUM_JOINTS).setPos_deg(-gripperGoal->command.position);

        ROS_INFO_STREAM("gripperActionGoalCB() " << gripperGoal->command.position);

        control_msgs::GripperCommandResult res;
        res.position = gripperGoal->command.position;
        res.reached_goal = true;
        mGripperActionServer.setSucceeded(res);
    }

    void cancelCurrentTrajectory() {
        mNextTrajectoryExecutionIndex = -1;
    }
};

int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "arips_arm_node");

    AripsArmNode aan;
    ros::spin();
}
