#pragma once

#include <toponav_ros/interfaces/CostsProfileInterface.h>
#include "CostProfileCalculator.h"

namespace toponav_ros {

class CostsProfileHolderBase: public CostsProfileInterface {
public:
  virtual void setCostsProfile(std::string const& profile) override { costs_comp_.setProfile(profile); };

protected:
  CostProfileCalculator costs_comp_;
};
  
} // namespace topo_nav
