#include <costmap_2d/layer.h>

#include <toponav_core/TopoMap.h>

class StepCostmapLayer: public costmap_2d::Layer {
public:
    explicit StepCostmapLayer(toponav_core::TopoMapPtr topoMap);

    void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                     int max_j) override;

private:
    toponav_core::TopoMapPtr mTopoMap;
};
