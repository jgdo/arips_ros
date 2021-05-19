#pragma once

#include <memory>

#include <tf2_ros/buffer.h>

class Simulator {
public:
    explicit Simulator(tf2_ros::Buffer& tf);
    ~Simulator();

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};
