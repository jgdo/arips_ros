#include <arips_navigation/path_planning/NavMap.h>

const std::string& ComposedNavMap::frameId() const { return mCostmap->frameId(); }

std::optional<double> ComposedNavMap::goalDistance(const Vector2d& point) const {
    return mPotentialMap->at(point);
}

std::optional<uint8_t> ComposedNavMap::cost(const Vector2d& point) const {
    return mCostmap->at(point);
}

const CostFunction& ComposedNavMap::costFunction() const { return mPotentialMap->costFunction(); }

double ComposedNavMap::lowestResolution() const {
    return std::min(mPotentialMap->resolution(), mCostmap->resolution());
}
