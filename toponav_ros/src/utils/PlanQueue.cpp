#include "toponav_ros/utils/PlanQueue.h"


namespace toponav_ros {

using namespace toponav_core;

uint32_t PlanQueue::addPath(const TopoPath &path) {
  ensureFreeSlot();
  auto iter = pathMap.emplace(nextID++, path).first;
  pathQueue.emplace_front(iter);
  return iter->first;
}

uint32_t PlanQueue::addPath(TopoPath &&path) {
  ensureFreeSlot();
  auto iter = pathMap.emplace(nextID++, path).first;
  pathQueue.emplace_front(iter);
  return iter->first;
}

TopoPath &PlanQueue::getPath(uint32_t pathID) {
  return pathMap.at(pathID);
}

void PlanQueue::ensureFreeSlot() {
  if(pathMap.size() >= MAX_SIZE) {
    pathMap.erase(pathQueue.back());
    pathQueue.pop_back();
  }
}

TopoPath &PlanQueue::getLastPath() {
  if(pathQueue.empty())
    throw std::range_error("Path queue is empty");
  
  return pathQueue.front()->second;
}
  
} // namespace topo_nav
