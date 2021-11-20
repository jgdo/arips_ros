#include <arips_arm_node/uniform_sample_filter.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>

#include "SCServo.h"

static const size_t NUM_JOINTS = 5;

class RosSCSServo
{
public:
  RosSCSServo(SCServo* servo, std::string const& ns) : mServo(servo)
  {
    ros::NodeHandle pnh("~" + ns);
    ros::NodeHandle nh(ns);

    pnh.param("id", mID, -1);

    if (mID < 1 || mID > 254)
    {
      throw std::range_error("RosSCSServo: servo ID must be in [1..254], is " + std::to_string(mID));
    }

    if (!pnh.getParam("angles", mAngleDegTable) || !pnh.getParam("raws", mRawTable) ||
        mAngleDegTable.size() != mRawTable.size() || mAngleDegTable.size() < 2)
    {
      ROS_WARN_STREAM("Using default angle table for SCS servo '" << ns << "' (ID: " << mID << ").");

      mAngleDegTable = { -120.0F, 120.0F };
      mRawTable = { 100, 1023 - 100 };
    }

    pnh.param("angle_offset", mAngleOffset_deg, 0.0F);

    pnh.param("angle_factor", mAngleFactor, 1.0F);

    try
    {
      setByteRegFromParam(pnh, P_CCW_DEAD, "ccw_dead");
      setByteRegFromParam(pnh, P_CW_DEAD, "cw_dead");

      setByteRegFromParam(pnh, P_COMPLIANCE_P, "kp");
      setByteRegFromParam(pnh, P_COMPLIANCE_I, "ki");
      setWordRegFromParam(pnh, P_INTEGRAL_LIMIT_L, "isum_max");

      setWordRegFromParam(pnh, P_MAX_TORQUE_L, "max_torque");
    }
    catch (std::exception const& e)
    {
      ROS_ERROR_STREAM(e.what());
    }

    mPosPub = nh.advertise<std_msgs::Float32>("pos_deg", 3);
    mPosSub = nh.subscribe("setpoint_deg", 1, &RosSCSServo::onSetposReceived, this);
    mRawPosSub = nh.subscribe("setpoint_raw", 1, &RosSCSServo::onSetposRawReceived, this);
  }

  float updateState()
  {
    auto raw = mServo->ReadPos(mID);
    float pos = rawToDegree(raw);
    mLastPose = pos;
    std_msgs::Float32 msg;
    msg.data = pos;
    mPosPub.publish(msg);
    return pos;
  }

  float getLastPose() const
  {
    return mLastPose;
  }

  static u16 calcSpeedPerStep(double radPerSec)
  {
    const double stepRadian = 300.0 / 1024.0 * M_PI / 180.0;
    const u16 speedPerStep = std::round(std::abs(radPerSec) / stepRadian);
    return speedPerStep;
  }

  void setPos_deg(float degree, u16 timeMs = 500, u16 speedPerStep = 0)
  {
    setPos_raw(std::round(degreeToRaw(degree)), timeMs, speedPerStep);
  }

  void setPos_raw(int raw, u16 timeMs = 500, u16 speedPerStep = 0)
  {
    try
    {
      mServo->WritePos(mID, raw, timeMs, speedPerStep);
    }
    catch (std::exception const& e)
    {
      ROS_ERROR_STREAM(e.what());
    }
  }

  int getId() const
  {
    return mID;
  }

private:
  SCServo* mServo = nullptr;

  int mID = -1;

  std::vector<float> mAngleDegTable;  // must contain at least 2 entries
  std::vector<int> mRawTable;         // same size as mRawTable

  ros::Subscriber mPosSub, mRawPosSub;
  ros::Publisher mPosPub;

  float mLastPose = 0;

  float mAngleOffset_deg = 0.0F;
  float mAngleFactor = 1.0F;

  void onSetposReceived(const std_msgs::Float32& msg)
  {
    setPos_deg(msg.data, 0, RosSCSServo::calcSpeedPerStep(0.5));
  }

  void onSetposRawReceived(const std_msgs::Int32& msg)
  {
    setPos_raw(msg.data);
  }

  template <class Tin, class Tout>
  static Tout convertAngle(Tin value, std::vector<Tin> const& sourceTable, std::vector<Tout> const& dstTable)
  {
    size_t idx = 0;
    for (idx = 0; idx < sourceTable.size() - 2; idx++)
    {
      if (value < sourceTable.at(idx + 1))
        break;
    }

    float alpha = (float)(value - sourceTable.at(idx)) / (float)(sourceTable.at(idx + 1) - sourceTable.at(idx));
    return dstTable.at(idx) * (1.0f - alpha) + dstTable.at(idx + 1) * alpha;
  };

  int degreeToRaw(float deg) const
  {
    return convertAngle((deg + mAngleOffset_deg) * mAngleFactor, mAngleDegTable, mRawTable);
  }

  float rawToDegree(int raw) const
  {
    return mAngleFactor * convertAngle(raw, mRawTable, mAngleDegTable) - mAngleOffset_deg;
  }

  void setByteRegFromParam(ros::NodeHandle& pnh, int reg, const std::string& name)
  {
    int value;
    if (pnh.getParam(name, value))
    {
      mServo->writeByte(mID, reg, value);
    }
  }

  void setWordRegFromParam(ros::NodeHandle& pnh, int reg, const std::string& name)
  {
    int value;
    if (pnh.getParam(name, value))
    {
      mServo->writeWord(mID, reg, value);
    }
  }
};

class AripsArmNode
{
public:
  static constexpr auto ControlDeltaT = 0.2;

  AripsArmNode()
    : mServoDevice(pnh.param<int>("baud", 500000), QString::fromStdString(pnh.param<std::string>("port",
                                                                                                 "/dev/"
                                                                                                 "scs215")))
    , mJointTrajActionServer(nh, "/arips_arm_controller/follow_joint_trajectory", false)
    , mGripperActionServer(nh, "/arips_gripper_controller/gripper_action", false)
  {
    mServos.reserve(NUM_JOINTS + 3);
    for (int i = 0; i < NUM_JOINTS + 3; i++)
    {
      mServos.emplace_back(&mServoDevice, "servo_" + std::to_string(i));
    }

    mUniformSampler.configure();
    pub = nh.advertise<trajectory_msgs::JointTrajectory>("sampled_trajectory", 3, false);
    mJointStatePub = nh.advertise<sensor_msgs::JointState>("joint_states", 3, false);

    mTimer = nh.createTimer(ros::Duration(ControlDeltaT), &AripsArmNode::timerCb, this);

    mJointTrajActionServer.registerGoalCallback([this] { trajectoryActionGoalCB(); });
    mJointTrajActionServer.registerPreemptCallback([this] { trajectoryActionPreemptCB(); });

    mJointTrajActionServer.start();

    mGripperActionServer.registerGoalCallback([this] { gripperActionGoalCB(); });
    // mGripperActionServer.registerPreemptCallback(boost::bind(&AripsArmNode::gripperActionPreemptCB,
    // this));

    mGripperActionServer.start();

    mKinectTiltSub = nh.subscribe("set_kinect_tilt_deg", 1, &AripsArmNode::kinectTiltCb, this);
    mKinectTiltPub = nh.advertise<std_msgs::Float32>("current_kinect_tilt_deg", 1);

    mChangeBaudSub = nh.subscribe("change_servo_baud", 1, &AripsArmNode::changeBaudCb, this);

    if (ros::NodeHandle("~").param<bool>("zero_tilt", false))
    {
      mKinectAngleFactor = 0.0F;
    }
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh{ "~" };

  SCServo mServoDevice;

  std::vector<RosSCSServo> mServos;

  ros::Publisher pub, mJointStatePub;

  industrial_trajectory_filters::UniformSampleFilter mUniformSampler;

  ros::Timer mTimer;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> mJointTrajActionServer;
  control_msgs::FollowJointTrajectoryGoalConstPtr mCurrentTrajGoal;

  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> mGripperActionServer;

  ros::Subscriber mKinectTiltSub;
  ros::Publisher mKinectTiltPub;

  ros::Subscriber mChangeBaudSub;

  float mKinectAngleFactor = M_PI / 180.0f;

  /**
   * Next points index of mSampledTrajectory
   * Inactive if negative, otherwise [0, traj size], if == traj size, this means
   * the last trajectory point was send and trajectory will be finished at next
   * cycle
   */
  int mNextTrajectoryExecutionIndex = -1;

  trajectory_msgs::JointTrajectory mSampledTrajectory;

  tf::TransformListener mTFListener;
  tf::TransformBroadcaster mTFBroadcaster;

  sensor_msgs::JointState mJointState;

  float mListKinectCorrectionAngle = 0.0;

  void startNewSampledTrajectory(const trajectory_msgs::JointTrajectory& trajSampled)
  {
    mSampledTrajectory = trajSampled;
    mNextTrajectoryExecutionIndex = 0;
    pub.publish(trajSampled);
  }

  void timerCb(const ros::TimerEvent& event)
  {
    if (mJointTrajActionServer.isActive() && mNextTrajectoryExecutionIndex >= 0)
    {
      if (mNextTrajectoryExecutionIndex < mSampledTrajectory.points.size())
      {
        ROS_INFO_STREAM("Sending trajectory point " << mNextTrajectoryExecutionIndex);
        auto const& point = mSampledTrajectory.points.at(mNextTrajectoryExecutionIndex);

        auto getNextTrajectoryStepDuration = [&] {
          return mNextTrajectoryExecutionIndex > 0 ?
                     (mSampledTrajectory.points.at(mNextTrajectoryExecutionIndex).time_from_start -
                      mSampledTrajectory.points.at(mNextTrajectoryExecutionIndex - 1).time_from_start) :
                     point.time_from_start;
        };

        const auto deltaT = getNextTrajectoryStepDuration().toSec();

        for (size_t i = 0; i < point.positions.size(); i++)
        {
          // const auto currentPos = mJointState.position.at(i);
          const auto setpointPos = point.positions.at(i);

          // const auto deltaPos = angles::normalize_angle(setpointPos - currentPos);
          // const auto speed = std::abs(deltaPos / deltaT);

          mServos.at(i).setPos_deg(setpointPos * 180.0F / M_PI,
                                   std::round(deltaT * 1000.0 * 1.2) /*, RosSCSServo::calcSpeedPerStep(speed * 0.8) */);
        }
        mNextTrajectoryExecutionIndex++;
      }
      else
      {
        ROS_INFO_STREAM("Sending trajectory finished");
        // trajectory finished
        control_msgs::FollowJointTrajectoryResult res;
        res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        mJointTrajActionServer.setSucceeded(res);
        mNextTrajectoryExecutionIndex = -1;
      }
    }

    mJointState.header.stamp = ros::Time::now();

    mJointState.name = { "joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint" };

    mJointState.position.resize(NUM_JOINTS + 1);
    mJointState.velocity.resize(NUM_JOINTS + 1);
    mJointState.effort.resize(NUM_JOINTS + 1);

    for (size_t i = 0; i < NUM_JOINTS; i++)
    {
      try
      {
        mJointState.position.at(i) = mServos.at(i).updateState() * M_PI / 180.0F;
      }
      catch (const std::runtime_error& err)
      {
        ROS_WARN_STREAM_ONCE("error when reading servo " << i << ": " << err.what());
      }
    }

    try
    {
      mJointState.position.at(NUM_JOINTS) = mServos.at(NUM_JOINTS).updateState();
    }
    catch (const std::runtime_error& err)
    {
      ROS_WARN_STREAM_ONCE(err.what());
    }

    for (size_t i = NUM_JOINTS + 1; i < mServos.size(); i++)
    {
      try
      {
        mServos.at(i).updateState();
      }
      catch (const std::runtime_error& err)
      {
        ROS_WARN_STREAM_ONCE(err.what());
      }
    }

    mJointStatePub.publish(mJointState);

    try
    {
      tf::Transform transform;

      bool useCorrected = false;

      try
      {
        tf::StampedTransform T_base_gtmarker, T_marker_kinect;

        mTFListener.lookupTransform("/marker_robot", "/kinect_link", ros::Time(0), T_marker_kinect);
        mTFListener.lookupTransform("/kinect_base", "/corrected_robot_marker", ros::Time(0), T_base_gtmarker);
        if ((ros::Time::now() - T_marker_kinect.stamp_).toSec() < 1.0)
        {
          transform = T_base_gtmarker * T_marker_kinect;

          useCorrected = true;

          ROS_DEBUG_STREAM("Using corrected kinect pose");
        }
      }
      catch (const tf::TransformException& ex)
      {
      }

      const float kinectAngle_deg = mServos.at(NUM_JOINTS + 1).getLastPose();
      if (!useCorrected)
      {
        // publish kinect tf
        const float kinectAngle_rad = kinectAngle_deg * mKinectAngleFactor;

        transform.setRotation(tf::createQuaternionFromRPY(0, kinectAngle_rad, 0));
        transform.setOrigin(transform * tf::Vector3(0.02, 0, 0.025));
      }

      std_msgs::Float32 kinectAngle_msg;
      kinectAngle_msg.data = kinectAngle_deg;
      mKinectTiltPub.publish(kinectAngle_msg);

      mTFBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/kinect_base", "/kinect_link"));
    }
    catch (const std::runtime_error& err)
    {
      ROS_WARN_STREAM("Updating kinect pose: " << err.what());
    }
  }

  void trajectoryActionGoalCB()
  {
    mCurrentTrajGoal = mJointTrajActionServer.acceptNewGoal();

    trajectory_msgs::JointTrajectory trajSampled;
    if (mUniformSampler.update(mCurrentTrajGoal->trajectory, trajSampled))
    {
      ROS_INFO_STREAM("Starting new trajectory");
      startNewSampledTrajectory(trajSampled);
    }
    else
    {
      ROS_INFO_STREAM("Trajectory resampling failed, aborting trajectory goal");
      control_msgs::FollowJointTrajectoryResult res;
      res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      res.error_string = ros::this_node::getName() + ": Aborted goal since could not resample the trajectory.";
      mJointTrajActionServer.setAborted(res, res.error_string);
      cancelCurrentTrajectory();
    }
  }

  void trajectoryActionPreemptCB()
  {
    ROS_INFO_STREAM("trajectoryActionPreemptCB().");
    cancelCurrentTrajectory();
    mJointTrajActionServer.setAborted();
  }

  void gripperActionGoalCB()
  {
    auto gripperGoal = mGripperActionServer.acceptNewGoal();

    // startNewGripperTrajectory(mCurrentGripperGoal->command.position);

    mServos.at(NUM_JOINTS).setPos_deg(gripperGoal->command.position);

    ROS_INFO_STREAM("gripperActionGoalCB() " << gripperGoal->command.position);

    control_msgs::GripperCommandResult res;
    res.position = gripperGoal->command.position;
    res.reached_goal = true;
    mGripperActionServer.setSucceeded(res);
  }

  void cancelCurrentTrajectory()
  {
    mNextTrajectoryExecutionIndex = -1;
  }

  void kinectTiltCb(const std_msgs::Float32& msg)
  {
    mServos.at(NUM_JOINTS + 1).setPos_deg(msg.data, 0, RosSCSServo::calcSpeedPerStep(0.5));
  }

  void changeBaudCb(const std_msgs::Int32& msg)
  {
    std::map<int, int> baudToCode{
      { 1000000, 0 }, { 500000, 1 }, { 250000, 2 }, { 128000, 3 },
      { 115200, 4 },  { 76800, 5 },  { 57600, 6 },  { 38400, 7 },
    };

    auto code = baudToCode.find(msg.data);
    if (code == baudToCode.end())
    {
      ROS_ERROR_STREAM("Could not set servo baud rate to " << msg.data << ": baud rate not supported");
    }

    for (auto& servo : mServos)
    {
      mServoDevice.writeByte(servo.getId(), P_LOCK, 0);
      mServoDevice.writeByte(servo.getId(), P_BAUD_RATE, code->second);
    }

    mServoDevice.setBaud(msg.data);

    for (auto& servo : mServos)
    {
      mServoDevice.writeByte(servo.getId(), P_LOCK, 1);
    }
  }

  void onKinectCorrectionAngle(const std_msgs::Float32& msg)
  {
    mListKinectCorrectionAngle += msg.data * 0.1F;
  }
};

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "arips_arm_node");

  AripsArmNode aan;
  ros::spin();
}
