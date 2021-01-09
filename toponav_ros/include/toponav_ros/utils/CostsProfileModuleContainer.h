#pragma  once

#include <vector>
#include <toponav_ros/interfaces/CostsProfileInterface.h>

namespace toponav_ros {

class CostsProfileModuleContainer: public CostsProfileInterface {
public:
  inline void addModule(const CostsProfileInterfacePtr& module) {
    modules_.push_back(module);
  }
  
  void setCostsProfile(std::string const &profile) override {
    for(auto& e: modules_) {
      e->setCostsProfile(profile);
    }
  }
  
protected:
  std::vector<CostsProfileInterfacePtr> modules_;
};
  
  
  
} // namespace topo_nav
