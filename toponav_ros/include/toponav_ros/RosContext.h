#pragma once

#include <memory>
#include <tf/transform_listener.h>


namespace toponav_ros {

struct RosContext {
	tf2_ros::Buffer* tfBuffer = nullptr;
	std::string globalFrame;
};

class RosContextHolder {
public:
	inline RosContextHolder(RosContext& context): _context(context) {}
	
protected:
	RosContext& _context;
};

} // namespace topo_nav

