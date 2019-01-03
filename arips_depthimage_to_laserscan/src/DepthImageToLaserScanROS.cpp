/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#include <depthimage_to_laserscan/DepthImageToLaserScanROS.h>

#include <visualization_msgs/Marker.h>

using namespace depthimage_to_laserscan;

DepthImageToLaserScanROS::DepthImageToLaserScanROS(ros::NodeHandle& n, ros::NodeHandle& pnh):pnh_(pnh), it_(n), srv_(pnh) {
    boost::mutex::scoped_lock lock(connect_mutex_);

    // Dynamic Reconfigure
    dynamic_reconfigure::Server<depthimage_to_laserscan::DepthConfig>::CallbackType f;
    f = boost::bind(&DepthImageToLaserScanROS::reconfigureCb, this, _1, _2);
    srv_.setCallback(f);
    
    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0);

    // Lazy subscription to depth image topic
    pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&DepthImageToLaserScanROS::connectCb, this, _1), boost::bind(&DepthImageToLaserScanROS::disconnectCb, this, _1));
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS(){
    sub_.shutdown();
}

void DepthImageToLaserScanROS::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                       const sensor_msgs::CameraInfoConstPtr& info_msg){
    static int frame_count = 0;
    frame_count++;
    if(frame_count < frame_skip_) {
        // std::cout << "Skipping frame" << std::endl;
        return;
    } else {
        frame_count = 0;
    }

    tf::StampedTransform transform;
    try
    {
        tfListener_.lookupTransform(dtl_.get_output_frame(), depth_msg->header.frame_id, ros::Time(0), transform);
        sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(depth_msg, info_msg, transform);
        pub_.publish(scan_msg);
        
        /*
        cv::Vec3f normal, offset;
        dtl_.getLastPlane(normal, offset);
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = dtl_.get_output_frame();
        marker.header.stamp = ros::Time();
        marker.ns = "groundplane_estimate";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points.resize(2);
        marker.points[0].x = 1;
        marker.points[0].y = 0;
        marker.points[0].z = 0;
        marker.points[1].x = 1+ normal(0);
        marker.points[1].y =  normal(1);
        marker.points[1].z =  normal(2);
        marker.scale.x = 0.05;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish( marker ); */
    }
    catch (std::runtime_error& e)
    {
        ROS_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
    }
}

void DepthImageToLaserScanROS::connectCb(const ros::SingleSubscriberPublisher& pub) {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (!sub_ && pub_.getNumSubscribers() > 0) {
        ROS_DEBUG("Connecting to depth topic.");
        image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
        sub_ = it_.subscribeCamera("image", 10, &DepthImageToLaserScanROS::depthCb, this, hints);
    }
}

void DepthImageToLaserScanROS::disconnectCb(const ros::SingleSubscriberPublisher& pub) {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() == 0) {
        ROS_DEBUG("Unsubscribing from depth topic.");
        sub_.shutdown();
    }
}

void DepthImageToLaserScanROS::reconfigureCb(depthimage_to_laserscan::DepthConfig& config, uint32_t level){
    dtl_.set_scan_time(config.scan_time);
    dtl_.set_range_limits(config.range_min, config.range_max);
    dtl_.set_scan_height(config.scan_height_min, config.scan_height_max);
    dtl_.set_output_frame(config.output_frame_id);
    frame_skip_ = config.frame_skip;
}
