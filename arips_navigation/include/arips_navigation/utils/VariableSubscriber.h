#pragma once

#include <ros/ros.h>

template <class T>
class VariableSubscriber {
public:
    T msg;
    bool receivedOnce = false;

    VariableSubscriber(std::string const& topic, size_t queue_size) {
        ros::NodeHandle nh;
        mSub = nh.subscribe(topic, queue_size, &VariableSubscriber::callback, this);
    }

protected:
    ros::Subscriber mSub;

    void callback(const T& receivedMsg) {
        msg = receivedMsg;
        receivedOnce = true;
    }
};