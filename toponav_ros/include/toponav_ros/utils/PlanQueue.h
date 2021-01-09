#pragma once

#include <toponav_core/TopoPath.h>

#include <map>
#include <list>


namespace toponav_ros {

class PlanQueue {
public:
  static const int MAX_SIZE = 50;
  
  /**
   *
   * @param path
   * @return plan id. 0 means invalid
   */
  uint32_t addPath(const toponav_core::TopoPath& path);
  uint32_t addPath(toponav_core::TopoPath&& path);
  
  /**
   * Throws exception if pathID invalid.
   * @param pathID 
   * @return Reference is valid until next modification call to the queue
   */
  toponav_core::TopoPath& getPath(uint32_t pathID);
  
  /**
   * Get the lastly added path. Throws exception if path queue is emtpy.
   * @return Reference is valid until next modification call to the queue
   */
  toponav_core::TopoPath& getLastPath();
  
private:
  typedef std::map<uint32_t, toponav_core::TopoPath> TPathMap;
  
  uint32_t nextID = 1; // start with a valid id
  
  TPathMap pathMap;
  std::list<TPathMap::iterator> pathQueue;
  
  void ensureFreeSlot();
};
  
} // namespace topo_nav
