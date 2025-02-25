#pragma once

#include <memory>

#include  <arips_semantic_map_msgs/SemanticMap.h>

class SemanticMapTracker {
public:
    explicit  SemanticMapTracker();
    ~SemanticMapTracker();

    arips_semantic_map_msgs::SemanticMap getLastSemanticMap() const;

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};