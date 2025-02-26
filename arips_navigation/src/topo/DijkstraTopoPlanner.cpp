#include <arips_navigation/topo/DijkstraTopoPlanner.h>

#include <arips_navigation/topo/MapTransformer.h>

#include <geometry_msgs/PoseArray.h>

#include <ctime>

static double get_cpu_time() { return (double)clock() / CLOCKS_PER_SEC; }

DijkstraTopoPlanner::DijkstraTopoPlanner(RoomPlanningInterfacePtr roomPlanner,
                                         DoorPlanningInterfacePtr doorPlanner,
                                         DijkstraTopoPlanner::Config const& config)
    : _config(config), nodePlanner(std::move(roomPlanner)), edgePlanner(std::move(doorPlanner)) {}

std::optional<TopoPath> DijkstraTopoPlanner::plan(TopoMap const* topoMap, TopoPose2D start,
                                                  TopoPose2D end,
                                                  PlanningStatistics* statisticsOutput) {
    double now = get_cpu_time();
    initLocalMap(topoMap, start, end);
    double initTime = get_cpu_time() - now;
    ROS_INFO_STREAM("local map init took " << initTime);

    static ros::NodeHandle nh;
    static auto pub = nh.advertise<geometry_msgs::PoseArray>("planning_map_poses", 1);

    // geometry_msgs::PoseArray mapPoses;
    // mapPoses.header.stamp = ros::Time::now();
    // mapPoses.header.frame_id = "global";
    //
    // for (const auto& n: localMap.getNodes()) {
    //     mapPoses.poses.push_back(n->pose.toPoseMsg());
    // }
    // pub.publish(mapPoses);
    // ros::spinOnce();

    now = get_cpu_time();
    while (localMap.getEndNode()->state == LocalNode::DISCOVERED && !mainQueue.empty()) {
        LocalNode* u = getAndRemoveLowestCostNode();

        // costs are only estimated based on heuristics, so compute real costs and insert it back
        if (!u->predMap.begin()->second.areCostsReal()) {
            LocalEdge* edge = u->predMap.begin()->second.predEdge;
            u->predMap.erase(u->predMap.begin()); // erase this entry since updateCostAndPred() will
                                                  // generate a new one with real costs
            if (updateCostAndPred(edge, true))
                mainQueue.insert(u);
            continue;
        }

        u->state = LocalNode::VISITED;

        // std::cout << "visiting node " << u << " " << u->minCostsFromStart() << std::endl;

        // stop if it is guaranteed that no better path exists
        if (u->minCostsFromStart() > localMap.getEndNode()->minCostsFromStart())
            break;

        for (auto e : u->edges) {
            if (mayVisitEdge(e)) {
                bool ok = updateCostAndPred(e, !_config.useRegionCostsHeuristic);
                if (ok && mainQueue.find(e->b) == mainQueue.end()) {
                    mainQueue.insert(e->b);
                }
            }
        }
    }
    double searchTime = get_cpu_time() - now;
    ROS_INFO_STREAM("search took " << searchTime);

    // localMap.saveToDot("after.dot");

    if (localMap.getEndNode()->state != LocalNode::VISITED) {
        ROS_INFO_STREAM("No topo plan found");
        return {};
    }

    std::list<LocalNode*> nodePath;

    std::cout << "pred costs: ";
    for (auto& e : localMap.getEndNode()->predMap) {
        std::cout << e.first << " ";
    }
    std::cout << std::endl;

    // inverse direction such that nodePath contains nodes  ]start...end]
    // notice that the start node IS NOT in the nodePath!
    for (LocalNode* n = localMap.getEndNode(); n != localMap.getStartNode();
         n = n->predMap.begin()->second.predEdge->a)
        nodePath.push_front(n);

    TopoPath plan;

    plan.totalCosts = localMap.getEndNode()->predMap.begin()->second.costFromStart;
    plan.pathElements.clear();

    for (LocalNode* n : nodePath) {
        auto& predEntry = n->predMap.begin()->second;

        if (predEntry.predEdge->isSameRegion()) {
            if (!predEntry.predEdge->pathPoints) {
                ROS_ERROR_STREAM("No path points found for path segment");
                return {};
            }

            plan.pathElements.push_back(std::make_unique<TopoPath::Movement>(
                TopoPose2D(predEntry.predEdge->topoNode,
                           predEntry.predEdge->a->predMap.begin()->second.localPosition),
                TopoPose2D(predEntry.predEdge->topoNode, predEntry.localPosition),
                predEntry.costFromStart - predEntry.predEdge->a->minCostsFromStart(),
                *predEntry.predEdge->pathPoints));
        } else {
            plan.pathElements.push_back(std::make_unique<TopoPath::Transition>(
                TopoPose2D(predEntry.predEdge->a->region,
                           predEntry.predEdge->a->predMap.begin()->second.localPosition),
                TopoPose2D(predEntry.predEdge->b->region, predEntry.localPosition),
                predEntry.costFromStart - predEntry.predEdge->a->minCostsFromStart(),
                predEntry.predEdge->topoEdge));
        }
    }

    if (statisticsOutput) {
        statisticsOutput->computeRegionCostsCalls = regionCostsCount;
        statisticsOutput->computeTransitionCostsCalls = transitionCostsCount;

        statisticsOutput->finalPlanningTime = initTime + searchTime;
    }

    ROS_INFO_STREAM("Found topo plan of size "
                    << plan.pathElements.size() << " and costs "
                    << localMap.getEndNode()->predMap.begin()->second.costFromStart);

    ROS_INFO_STREAM("region cost calls: " << regionCostsCount.first << " | "
                                          << regionCostsCount.second);

    ROS_INFO_STREAM("transition cost calls: " << transitionCostsCount.first << " | "
                                              << transitionCostsCount.second);

    return plan;
}

void DijkstraTopoPlanner::initLocalMap(TopoMap const* topoMap, TopoPose2D start, TopoPose2D end) {
    mainQueue.clear();
    regionCostsCount = {};
    transitionCostsCount = {};

    if (topoMap->getVersionCount() != localMap.getLastTopoMapVersion()) {
        MapTransformer::createLocalMap(
            *topoMap, localMap, edgePlanner,
            _config.prePathDiscovery ? LocalNode::UNKNOWN : LocalNode::DISCOVERED,
            _config.useBackDijkstraHeuristic ? LocalNode::infCosts : 0.0);
        ROS_INFO("Topo map changed, re-creating planning map");
    } else {
        // mark everything as discovered if no pre path discovery
        localMap.resetStartEndPaths(_config.prePathDiscovery ? LocalNode::UNKNOWN
                                                             : LocalNode::DISCOVERED,
                                    _config.useBackDijkstraHeuristic ? LocalNode::infCosts : 0.0);
        ROS_INFO("Re-using planning map for Dijkstra");
    }

    localMap.setStartNode(start.room, _config.useBackDijkstraHeuristic ? LocalNode::infCosts : 0);
    localMap.getStartNode()->predMap.emplace(
        std::piecewise_construct, std::make_tuple(0),
        std::make_tuple(start.pose, 0, nullptr)); // first node has costs 0 and no pred edge
    localMap.setEndNode(end.room, _config.useBackDijkstraHeuristic ? LocalNode::infCosts : 0);

    localMap.getStartNode()->pose = start.pose;
    localMap.getEndNode()->pose = end.pose;

    if (_config.prePathDiscovery) {
        bfsFromGoal();
    } else {
        localMap.getStartNode()->state = LocalNode::DISCOVERED;
        localMap.getEndNode()->state = LocalNode::DISCOVERED;
    }

    // localMap.saveToDot("before.dot");

    if (localMap.getStartNode()->state == LocalNode::DISCOVERED) {
        if (_config.useBackDijkstraHeuristic) {
            doBackDijkstraHeuristic();
        }

        mainQueue.insert(localMap.getStartNode());
    }
}

bool DijkstraTopoPlanner::updateCostAndPred(DijkstraTopoPlanner::LocalEdge* v, bool evaluateReal) {
    if (!evaluateReal && v->costsState == LocalEdge::UNKNOWN) {
        calcEdgeHeuristics(v);
    } else if (evaluateReal) {
        if (v->isTransitionEdge()) { // transition edge -> recompute exit pose and costs based on
                                     // current approach pose
            auto maybeCosts = edgePlanner->computeTransitionCost(
                v->topoEdge, v->a->predMap.begin()->second.localPosition);

            v->cost = maybeCosts.value_or(std::numeric_limits<double>::max());

            if (!maybeCosts) {
                ROS_WARN_STREAM("DijkstraTopoPlanner: cost for edge '"
                                << v->topoEdge->getName() << "' was computed to nullopt, ignoring");
                return false;
            }

            transitionCostsCount.second++;
        } else {
            auto [maybeCosts, pathPoints] = nodePlanner->computeCostsOnRegion(
                v->b->region, v->a->predMap.begin()->second.localPosition, v->b->pose);
            v->pathPoints = pathPoints;

            v->cost = maybeCosts.value_or(std::numeric_limits<double>::max());

            if (!maybeCosts) {
                ROS_WARN_STREAM("DijkstraTopoPlanner: computeCostsOnRegion '"
                                << v->b->region->getName() << "' returned nullopt costs, ignoring");
                return false;
            }

            regionCostsCount.second++; // TODO: really ignore failed calls?
        }

        v->costsState = LocalEdge::REAL;
    }

    double alternateCost = v->a->minCostsFromStart() + v->cost;
    v->b->predMap.emplace(std::piecewise_construct, std::make_tuple(alternateCost),
                          std::make_tuple(v->b->pose, alternateCost, v));
    std::cout << "updating edge " << v << " " << v->cost << std::endl;
    return true;
}

void DijkstraTopoPlanner::calcEdgeHeuristics(DijkstraTopoPlanner::LocalEdge* e) {
    if (e->isSameRegion()) {
        e->cost = nodePlanner->getHeuristics(e->a->pose, e->b->pose);
        regionCostsCount.first++;
    } else {
        e->cost = edgePlanner->getHeuristics(e->topoEdge);
        transitionCostsCount.first++;
    }
    e->costsState = LocalEdge::HEURISTIC;
}

void DijkstraTopoPlanner::doBackDijkstraHeuristic() {
    std::set<LocalNode*> q;

    localMap.getEndNode()->hVal = 0;
    q.insert(localMap.getEndNode());
    while (!q.empty()) {
        // remove lowest node
        auto iter = std::min_element(q.begin(), q.end(),
                                     [](LocalNode* a, LocalNode* b) { return a->hVal < b->hVal; });
        LocalNode* n = *iter;
        n->backDijkstraVisited = true;
        q.erase(iter);

        // update succs
        for (LocalEdge* e : n->predEdges) {
            if (e->a->state == LocalNode::DISCOVERED && !e->a->backDijkstraVisited) {
                if (q.find(e->a) == q.end()) {
                    q.insert(e->a);
                }

                if (e->costsState == LocalEdge::UNKNOWN)
                    calcEdgeHeuristics(e);

                double altDist = n->hVal + e->cost;
                if (altDist < e->a->hVal)
                    e->a->hVal = altDist;
            }
        }
    }

    ROS_INFO_STREAM("hStart = " << localMap.getStartNode()->hVal);
}

void DijkstraTopoPlanner::bfsFromGoal() {
    std::list<LocalNode*> queue = {localMap.getEndNode()};
    while (!queue.empty()) {
        LocalNode* n = queue.front();
        queue.pop_front();

        n->state = LocalNode::DISCOVERED; // DISCOVERED corresponds to visited in bfs
        for (LocalEdge* predE : n->predEdges) {
            LocalNode* pred = predE->a;
            if (pred->state == LocalNode::UNKNOWN) {  // UKNOWN means whether visited nor in queue
                pred->state = LocalNode::DISCOVERING; // DISCOVERING means in queue but not visited
                queue.push_back(pred);
            }
        }
    }
}

DijkstraTopoPlanner::LocalNode* DijkstraTopoPlanner::getAndRemoveLowestCostNode() {
    auto iter =
        std::min_element(mainQueue.begin(), mainQueue.end(), [](LocalNode* a, LocalNode* b) {
            return a->getRanking() < b->getRanking();
        });
    LocalNode* n = *iter;
    mainQueue.erase(iter);
    return n;
}
