#include "Simulator.h"

#include <arips_navigation/utils/DriveDirectionIndicator.h>
#include <arips_navigation/utils/transforms.h>

#include <geometry_msgs/Twist.h>
#include <interactive_markers/interactive_marker_server.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

struct Simulator::Pimpl {
    const ros::Rate mControlRate{50.0};

    tf2_ros::Buffer& mTfBuffer;
    ros::Subscriber mCmdVelSub;
    ros::Publisher mOdomPub;
    ros::Timer mControlLoopTimer;

    DriveDirectionIndicator mSetpointIndicator {"setpoint_indicator", 0.2};

    geometry_msgs::Twist mLastTwist;
    int mTwistTimeoutCount =
        0; // will be decremented on each control cycle, reset twist when no feedbeck received
    nav_msgs::Odometry mRobotPose;

    tf2_ros::TransformBroadcaster mTfBroadcaster;

    interactive_markers::InteractiveMarkerServer mMarkerServer{"simulation_marker"};

    explicit Pimpl(tf2_ros::Buffer& tf) : mTfBuffer{tf} {
        ros::NodeHandle nh;
        mRobotPose.header.frame_id = "map";
        mRobotPose.child_frame_id = "arips_base";
        mRobotPose.pose.pose.orientation.w = 1; // start at 0

        mOdomPub = nh.advertise<nav_msgs::Odometry>("odom", 5);
        mCmdVelSub = nh.subscribe("cmd_vel", 100, &Pimpl::onCmdVel, this);
        mControlLoopTimer = nh.createTimer(mControlRate, &Pimpl::onRunControlLoop, this);

        setupMarkers();
    }

    void setupMarkers() {
        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.header.stamp = ros::Time::now();
        int_marker.name = "robot_sim_marker";
        int_marker.description = "Simulated robot position";
        int_marker.pose.position.z = 0.1;
        int_marker.pose.orientation.w = 1;

        visualization_msgs::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        {
            visualization_msgs::Marker box_marker;
            box_marker.type = visualization_msgs::Marker::CYLINDER;
            box_marker.scale.x = 0.4;
            box_marker.scale.y = 0.4;
            box_marker.scale.z = 0.2;
            box_marker.color.r = 0.584;
            box_marker.color.g = 0.365;
            box_marker.color.b = 0.0;
            box_marker.color.a = 0.5;
            box_marker.pose.orientation.w = 1;
            box_control.markers.push_back(box_marker);
        }

        {
            visualization_msgs::Marker box_marker;
            box_marker.type = visualization_msgs::Marker::ARROW;
            box_marker.scale.x = 0.2;
            box_marker.scale.y = 0.02;
            box_marker.scale.z = 0.02;
            box_marker.color.r = 1.0;
            box_marker.color.g = 0.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 1.0;
            box_marker.pose.position.z = 0.1;
            box_marker.pose.orientation.w = 1;
            box_control.markers.push_back(box_marker);
        }

        // add the control to the interactive marker
        int_marker.controls.push_back(box_control);

        {
            // create a control which will move the box
            // this control does not contain any markers,
            // which will cause RViz to insert two arrows
            visualization_msgs::InteractiveMarkerControl control;
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;

            // add the control to the interactive marker
            int_marker.controls.push_back(control);
        }

        // add the interactive marker to our collection &
        // tell the server to call processFeedback() when feedback arrives for it
        mMarkerServer.insert(int_marker, [this](auto&& PH1) {
            processMarkerFeedback(std::forward<decltype(PH1)>(PH1));
        });

        // 'commit' changes and send to all clients
        mMarkerServer.applyChanges();
    }

    void onCmdVel(geometry_msgs::Twist const& cmd_vel) {
        mLastTwist = cmd_vel;
        mTwistTimeoutCount = static_cast<int>(0.5 / mControlRate.expectedCycleTime().toSec());
    }

    void onRunControlLoop(const ros::TimerEvent& ev) {
        // reset velocity after timeout
        if (mTwistTimeoutCount > 0) {
            mTwistTimeoutCount--;
        } else {
            mLastTwist = geometry_msgs::Twist{};
        }

        mSetpointIndicator.indicate(mLastTwist);

        // see also http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
        double x = mRobotPose.pose.pose.position.x;
        double y = mRobotPose.pose.pose.position.y;
        double th = getYawFromQuaternion(mRobotPose.pose.pose.orientation);

        const double dt = mControlRate.expectedCycleTime().toSec();
        const double vx = mLastTwist.linear.x;
        const double vy = mLastTwist.linear.y; // usually 0
        const double vth = mLastTwist.angular.z;

        const double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        const double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        const double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        mRobotPose.header.stamp = ev.current_expired;
        mRobotPose.pose.pose.position.x = x;
        mRobotPose.pose.pose.position.y = y;
        mRobotPose.pose.pose.orientation = createQuaternionMsgFromYaw(th);
        mRobotPose.twist.twist = mLastTwist;
        // TODO: set covariance
        mOdomPub.publish(mRobotPose);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header = mRobotPose.header;
        transformStamped.child_frame_id = mRobotPose.child_frame_id;
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation = mRobotPose.pose.pose.orientation;

        mTfBroadcaster.sendTransform(transformStamped);

        auto markerPose = mRobotPose.pose.pose;
        markerPose.position.z = 0.1;
        mMarkerServer.setPose("robot_sim_marker", markerPose, mRobotPose.header);
        mMarkerServer.applyChanges();
    }

    void
    processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
            mRobotPose.pose.pose.position.x = feedback->pose.position.x;
            mRobotPose.pose.pose.position.y = feedback->pose.position.y;
            mRobotPose.pose.pose.orientation = feedback->pose.orientation;
        }
    }
};

Simulator::Simulator(tf2_ros::Buffer& tf) : mPimpl{std::make_unique<Pimpl>(tf)} {}

Simulator::~Simulator() = default;