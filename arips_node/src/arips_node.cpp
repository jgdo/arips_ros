#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>

#include "sysarips.h"
#include <boost/thread.hpp>

#include <arips_msgs/SensorState.h>
#include <arips_msgs/ServoPosition.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/recursive_mutex.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign.hpp>

#define KINECT_SERVO_NUM 7

#define ARIPS_CMD_VEL_TOPIC "/cmd_vel"
#define ARIPS_RESET_POS_TOPIC "reset_pos"
#define ARIPS_ODOMETRYSTATE_TOPIC "/odom"
#define ARIPS_SETSERVOPOS_TOPIC "setservo"
#define ARIPS_SERVOSTATE_TOPIC "servostate"
#define ARIPS_KINECT_TILT_TOPIC "kinect_tilt"

#define ARIPS_SENSORSTATE_TOPIC "/sensor_state"

#define LOCK_ARIPS() boost::recursive_mutex::scoped_lock _lock(aripsMutex)

using namespace std;

SysArips arips;

boost::recursive_mutex aripsMutex;

#define TIMER_CYCLE	0.050 	// in s, min 5, not working with smaller values

void SysAripsThread() {
	while(ros::ok())
		arips.ReadInput();
}

ros::Publisher sensorPublisher;
ros::Publisher odometryPublisher;
ros::Publisher servoPublisher;

double kinect_fact, kinect_offset;

void CmdVelCallback(const geometry_msgs::Twist& msg) {
	LOCK_ARIPS();
	
	float x = msg.linear.x;
	float z = msg.angular.z * arips.GetWheelDist() / 2.0f;
	
	float left = (x - z);
	float right = (x + z);
	
	// cout << "CmdVelCallback " << left << " " << right << endl;
	
	arips.DriveAt(left, right);
}


void ResetPosCallback(const std_msgs::String&) {
	cout << "resetting pos" << endl;
	LOCK_ARIPS();
	arips.ResetDrivenDistanceAndPos();
}

void KinectTiltCallback(const std_msgs::Float32 &angle) {
	LOCK_ARIPS();
	float pos[1];
	pos[0] = (kinect_offset + angle.data * kinect_fact + 0.5);
	arips.SetServos((1 << KINECT_SERVO_NUM), pos, 1);
}

void AripsTimerCallback(const ros::TimerEvent& event) {
	static int count = 0;
	
	LOCK_ARIPS();
	// System.out.println("Run run run " + timerCount);
	
	arips.UpdatePosition();
	
	if (--count <= 0) {
		
		
		count = 2;
		// if (sensorPublisher.hasSubscribers())
		arips.RefreshAdc();
	}
	
}

void NewADCDataAvailable() {
	LOCK_ARIPS();
	
	arips_msgs::SensorState st;
	for(int i = 0; i < 8; i++)
		st.ADC.push_back(arips.GetAdc(i));
	
	sensorPublisher.publish(st);
}

void NewDistanceDataAvailable() {
	LOCK_ARIPS();
	
	static tf::TransformBroadcaster br;
	
	tf::Transform transform;
	// transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	// transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0));
	// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/"));
	
	transform.setOrigin(tf::Vector3(arips.getPosX(), arips.getPosY(), 0.0));
	transform.setRotation(tf::createQuaternionFromYaw(arips.getAngle()));
	
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_link"));
	
	Rad angle = arips.getAngle();
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
	
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/base_link";
	
	static tf::Vector3 prevPos;
	static Rad prevAngle;
	static ros::Time lastTime;
	
	odom.pose.pose.position.x = arips.getPosX();
	odom.pose.pose.position.y = arips.getPosY();
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	
	tf::Vector3 pos = tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0);
	
	ros::Time now = ros::Time::now();
	
	Rad angleDiff = (angle - prevAngle);
	angleDiff.Normalize();
	
	double dt = (now - lastTime).toSec();
	
	odom.twist.twist.linear.x = (pos - prevPos).length() / dt;
	odom.twist.twist.angular.z = angleDiff / dt;
	
	prevPos = pos;
	prevAngle = angle;
	lastTime = now;
	
	
odom.pose.covariance[0]  = 0.01;
odom.pose.covariance[7]  = 0.01;
odom.pose.covariance[14] = 99999;
odom.pose.covariance[21] = 99999;
odom.pose.covariance[28] = 99999;
odom.pose.covariance[35] = 0.01;

odom.twist.covariance = odom.pose.covariance;

/*

	odom.pose.covariance =
		boost::assign::list_of(5e-3)(0)(0)(0)(0)(0)(0)(5e-3)(0)(0)(0)(0)(0)(0)(5e6)(0)(0)(0)(0)(0)(0)(5e6)(0)(0)(0)(0)(0)(0)(5e6)(0)(0)(
					0)(0)(0)(0)(5e3);
	
	odom.twist.covariance =
			boost::assign::list_of	(0.1)(0)(0)(0)(0)(0)
									(0)(0.1)(0)(0)(0)(0)
									(0)(0)(0.1)(0)(0)(0)
									(0)(0)(0)(0.1)(0)(0)
									(0)(0)(0)(0)(0.1)(0)
									(0)(0)(0)(0)(0)(0.1);
*/
	odometryPublisher.publish(odom);
}


void SendServoTransform(const ros::TimerEvent&) {
	static tf::TransformBroadcaster br;
	
	LOCK_ARIPS();
	
	tf::Transform transform;
	
	double angle = (arips.GetServos()[KINECT_SERVO_NUM] - kinect_offset) / kinect_fact;
	
	transform.setOrigin(tf::Vector3(-0.14, 0, 0.74));
	transform.setRotation(tf::createQuaternionFromRPY(0, -angle, 0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/camera_link"));
	
	transform.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/laser_frame"));
}

void OnAripsServoChanged() {
	LOCK_ARIPS();
	
	static arips_msgs::ServoPosition pos;
	pos.mask = 0xFF;
	
	for (int i = 0; i < 8; i++) {
		float s = arips.GetServos()[i];
		
		if (i == KINECT_SERVO_NUM)
			s = (s - kinect_offset) / kinect_fact;
		
		pos.positions.push_back(s);
	}
	
	servoPublisher.publish(pos);
}

void SetServoCallback(const arips_msgs::ServoPosition& msg) {
	LOCK_ARIPS();
	
	int size = msg.positions.size();
	float pos[size];
	for(int i = 0; i < size; i++)
		pos[i] = msg.positions[i];
	
	arips.SetServos(msg.mask, pos, size);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "arips_node");

	if (arips.Open() < 0) {
		cout << "Failed to connect to motor control" << endl;
		return -1;
	}
	
	ros::NodeHandle n("arips");
	
	odometryPublisher = n.advertise<nav_msgs::Odometry>(ARIPS_ODOMETRYSTATE_TOPIC, 1);
	servoPublisher = n.advertise<arips_msgs::ServoPosition>(ARIPS_SERVOSTATE_TOPIC, 1);
	sensorPublisher = n.advertise<arips_msgs::SensorState>(ARIPS_SENSORSTATE_TOPIC, 1);
	
	
	n.param("kinect_fact", kinect_fact, -134.0);
	n.param("kinect_offset", kinect_offset, 108.0);
	
	float pos[1];
	pos[0] = (kinect_offset + 0 * kinect_fact + 0.5);
	arips.SetServos((1 << KINECT_SERVO_NUM), pos, 1);
	
	
	ros::Subscriber cmdVelSub = n.subscribe(ARIPS_CMD_VEL_TOPIC, 5, CmdVelCallback);
	ros::Subscriber resetPosSub = n.subscribe(ARIPS_RESET_POS_TOPIC, 5, ResetPosCallback);
	ros::Subscriber kinectTiltSub = n.subscribe(ARIPS_KINECT_TILT_TOPIC, 5, KinectTiltCallback);
	ros::Subscriber setServoSub = n.subscribe(ARIPS_SETSERVOPOS_TOPIC, 5, SetServoCallback);
	
	arips.AddOnAdc(boost::bind(NewADCDataAvailable));
	arips.AddOnNewPostition(boost::bind(NewDistanceDataAvailable));
	arips.AddOnServoChanged(boost::bind(OnAripsServoChanged));
	
	boost::thread aripsThread(SysAripsThread);
	
	ros::Timer servoTransformTimer = n.createTimer(ros::Duration(0.1), SendServoTransform);
	
	ros::Timer aripsTimer = n.createTimer(ros::Duration(TIMER_CYCLE), AripsTimerCallback);
	
	
	while (ros::ok()) 
		ros::spin();
}


