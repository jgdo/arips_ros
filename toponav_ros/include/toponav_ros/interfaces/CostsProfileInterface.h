#pragma once

#include <string>
#include <memory>

namespace toponav_ros {

class CostsProfileInterface {
public:
  typedef std::shared_ptr<CostsProfileInterface> Ptr;
  
  virtual void setCostsProfile(std::string const& profile) { /*does nothing per default*/ };
};

typedef CostsProfileInterface::Ptr CostsProfileInterfacePtr;
  
} // namespace topo_nav
