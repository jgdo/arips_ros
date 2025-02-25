#include <costmap_2d/layer.h>

#include <arips_navigation/topo_nav/SemanticMapTracker.h>

class StepCostmapLayer: public costmap_2d::Layer {
public:
    explicit StepCostmapLayer(SemanticMapTracker& mapTracker);

    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                     int max_j) override;

private:
    SemanticMapTracker& mMapTracker;
};
