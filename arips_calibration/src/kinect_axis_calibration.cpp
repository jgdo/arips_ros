#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/Empty.h>

std::ostream& operator<<(std::ostream& out, tf::Vector3 const& v) {
    return out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
}

std::ostream& operator<<(std::ostream& out, tf::Quaternion const& v) {
    return out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w() << ")";
}


class KinectAxisCalibration {
public:
    KinectAxisCalibration() {
        mSignalSub =  mNode.subscribe("/calc_transform_signal", 1, &KinectAxisCalibration::onComputeSignal, this);

        mTransformListenTimer = mNode.createTimer(ros::Duration(0.3), &KinectAxisCalibration::onTransformTimerCb, this);
    }

    void calcRotationAxis(const tf::Transform &transformKinectMarkerA, const tf::Transform &transformKinectMarkerB) {
        ROS_INFO("####################");

        const tf::Transform baseMarker(tf::Matrix3x3::getIdentity(), tf::Vector3{0.41, 0.0, 0});

        const tf::Transform baseKinectA{baseMarker * transformKinectMarkerA.inverse()};
        const tf::Transform baseKinectB{baseMarker * transformKinectMarkerB.inverse()};

        ROS_INFO_STREAM("baseKinectA origin: " << baseKinectA.getOrigin());
        ROS_INFO_STREAM("baseKinectA rotation: " << baseKinectA.getRotation());
        ROS_INFO_STREAM("baseKinectB origin: " << baseKinectB.getOrigin());
        ROS_INFO_STREAM("baseKinectB rotation: " << baseKinectB.getRotation());

        tf::Transform kinectAtoB{baseKinectA.inverseTimes(baseKinectB)};

        const auto axisVector{-kinectAtoB.getRotation().getAxis()};
        float angle = kinectAtoB.getRotation().getAngleShortestPath();

        ROS_INFO_STREAM("rotation axis vector: " << axisVector);
        ROS_INFO_STREAM("angle: " << angle);

        ROS_INFO_STREAM("transform origin: " << kinectAtoB.getOrigin());

        tf::Vector3 planeNormalA = axisVector.cross(tf::Vector3(1, 0, 0)).normalized();
        tf::Vector3 planeNormalB = axisVector.cross(planeNormalA).normalized();

        ROS_INFO_STREAM("planeNormalA: " << planeNormalA);
        ROS_INFO_STREAM("planeNormalB: " << planeNormalB);

        const float sizeIncFactor = sin(angle / 2.0) * 2;

        const float distBetween = kinectAtoB.getOrigin().length();

        ROS_INFO_STREAM("distBetween: " << distBetween);

        const float distToAxis = distBetween / sizeIncFactor;

        ROS_INFO_STREAM("distToAxis: " << distToAxis);

        const float x2d = planeNormalA.dot(kinectAtoB.getOrigin());
        const float y2d = planeNormalB.dot(kinectAtoB.getOrigin());

        float xyLen = sqrt(x2d * x2d + y2d * y2d);

        const float xNorm = y2d / xyLen;
        const float yNorm = -x2d / xyLen;

        const float triangleHeight = cos(angle / 2.0) * distToAxis;

        const float xAxis = (x2d / 2.0) + xNorm * triangleHeight;
        const float yAxis = (y2d / 2.0) + yNorm * triangleHeight;

        tf::Vector3 axisOffsetKinect(planeNormalA * xAxis + planeNormalB * yAxis);

        tf::Vector3 axisOffsetBase = baseKinectA * axisOffsetKinect;

        ROS_INFO_STREAM("Axis offset [base]: " << axisOffsetBase);
    }

    void onComputeSignal(const std_msgs::Empty&) {
        mAveragingRotation.setValue(0, 0, 0, 0);
        mAveragingOrigin.setValue(0, 0, 0);
        mAveragingCount = 0;
    }

    void onTransformTimerCb(const ros::TimerEvent&) {
        if (mAveragingCount >= cAveragingNum) {
            tf::Transform transform((mAveragingRotation/mAveragingCount).normalized(), mAveragingOrigin / mAveragingCount);

            mAveragingRotation.setValue(0, 0, 0, 0);
            mAveragingOrigin.setValue(0, 0, 0);

            if(mHasA) {
                mTransformB = transform;

                calcRotationAxis(mTransformA, mTransformB);
            } else {
                mTransformA = transform;
                ROS_INFO("Got transform A, waiting next signal ...");
            }

            mHasA = !mHasA;

            mAveragingCount = -1;
        } else if(mAveragingCount >= 0) {
            try{
                tf::StampedTransform transform;
                mTransfromListener.lookupTransform("/kinect_link", "/ar_marker_1",
                                                   ros::Time(0), transform);


                mAveragingOrigin += transform.getOrigin();
                mAveragingRotation += transform.getRotation();

                ROS_INFO("Got average sample");
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }

            mAveragingCount++;
        }
    }



    static constexpr auto cAveragingNum = 1;

    ros::NodeHandle mNode;

    ros::Timer mTransformListenTimer;

    bool mHasA = false;
    ros::Subscriber mSignalSub;
    tf::TransformListener mTransfromListener;

    tf::Transform mTransformA, mTransformB;

    tf::Quaternion mAveragingRotation;
    tf::Vector3 mAveragingOrigin{0,0,0};

    int mAveragingCount = -1;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
//
//    static constexpr float chosenAngle = 0.25;
//
//    const auto originalRotation {tf::createQuaternionFromRPY(0, 0.3, 0) * tf::createQuaternionFromRPY(0.00, 0, 0)};
//
//    tf::Vector3 originalOffset(0, 0, 0.3);
//
//    tf::Transform aToB = tf::Transform(tf::Matrix3x3::getIdentity(), originalOffset) * tf::Transform(tf::createQuaternionFromRPY(0, chosenAngle, 0))
//            * tf::Transform(tf::Matrix3x3::getIdentity(), -originalOffset);
    KinectAxisCalibration calibration;

    while (ros::ok()){
        ros::spin();
    }
};

