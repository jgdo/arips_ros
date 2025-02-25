#include <ros/node_handle.h>

#include <arips_navigation/topo_nav/SemanticMapTracker.h>

struct SemanticMapTracker::Pimpl {
    explicit Pimpl() {
        ros::NodeHandle nh;

        mSub = nh.subscribe("semantic_map", 1, &Pimpl::onNewSemanticMap, this);
    }

    void onNewSemanticMap(const arips_semantic_map_msgs::SemanticMap& map) {
        ROS_INFO("Received new semantic map");
        mMap = map;
    }

    ros::Subscriber mSub;
    arips_semantic_map_msgs::SemanticMap mMap;
};

SemanticMapTracker::SemanticMapTracker()
    : mPimpl{std::make_unique<Pimpl>()}

{}

SemanticMapTracker::~SemanticMapTracker() = default;

arips_semantic_map_msgs::SemanticMap SemanticMapTracker::getLastSemanticMap() const {
    return mPimpl->mMap;
}
