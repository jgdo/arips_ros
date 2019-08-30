#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <Eigen/Dense>

std::ostream& operator<<(std::ostream& out, tf::Vector3 const& v) {
    return out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
}

std::ostream& operator<<(std::ostream& out, tf::Quaternion const& v) {
    return out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w() << ")";
}

// see https://github.com/oleg-alexandrov/projects/blob/master/eigen/Kabsch.cpp

Eigen::Affine3d Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out) {

  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = 1.0; // dist_out/dist_in;
  out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);

  return A;
}

class KinectAxisCalibration {
public:
    KinectAxisCalibration() {
        mSignalSub =  mNode.subscribe("/calc_transform_signal", 1, &KinectAxisCalibration::onComputeSignal, this);

        mTransformListenTimer = mNode.createTimer(ros::Duration(0.3), &KinectAxisCalibration::onTransformTimerCb, this);
    }

    void onComputeSignal(const std_msgs::Empty&) {
        mAveragingRotation.setValue(0, 0, 0, 0);
        mAveragingOrigin.setValue(0, 0, 0);
        mAveragingCount = 0;
    }

    void setMatrix(Eigen::Matrix3Xd& mat, int i, tf::Vector3 const& d) {
        mat.col(i) = Eigen::Vector3d(d.x(), d.y(), d.z());
    }
    
    void printEigen(const Eigen::Vector3d& v) {
        std::cout << v.x() << " " << v.y() << " " << v.z() << std::endl;
    }
    
    
    
    void onTransformTimerCb(const ros::TimerEvent&) {
        try{
            tf::StampedTransform transform1, transform2, transform3, transform4;
            mTransfromListener.lookupTransform("/kinect_link", "/ar_marker_10",
                                                ros::Time(0), transform1);

            mTransfromListener.lookupTransform("/kinect_link", "/ar_marker_11",
                                                ros::Time(0), transform2);
            
            mTransfromListener.lookupTransform("/kinect_link", "/ar_marker_12",
                                                ros::Time(0), transform3);
            
            mTransfromListener.lookupTransform("/kinect_link", "/ar_marker_13",
                                                ros::Time(0), transform3);
            
            Eigen::Matrix3Xd matIn(3, 4);
            setMatrix(matIn, 0, transform1.getOrigin());
            setMatrix(matIn, 1, transform2.getOrigin());
            setMatrix(matIn, 2, transform3.getOrigin());
            setMatrix(matIn, 3, transform4.getOrigin());
            
            
            Eigen::Matrix3Xd matOut(3, 4);
            setMatrix(matOut, 0, tf::Vector3(0.28, -0.2, 0.01));
            setMatrix(matOut, 1, tf::Vector3(0.28, 0.2, 0.001));
            setMatrix(matOut, 2, tf::Vector3(0, 0.2, -0.001));
            setMatrix(matOut, 3, tf::Vector3(0, -0.2, 0));
            
            Eigen::Affine3d affine = Find3DAffineTransform(matIn, matOut);
            
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(affine.translation().x(), affine.translation().y(), affine.translation().z()) );
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            mTransformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/kinect_link", "/marker_floor"));
            
            ROS_INFO("Publishing trans");
        }
        
        catch (tf::TransformException ex){
            // ROS_ERROR("%s",ex.what());
        }
    }



    static constexpr auto cAveragingNum = 1;

    ros::NodeHandle mNode;

    ros::Timer mTransformListenTimer;

    bool mHasA = false;
    ros::Subscriber mSignalSub;
    tf::TransformListener mTransfromListener;
    tf::TransformBroadcaster mTransformBroadcaster;

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

