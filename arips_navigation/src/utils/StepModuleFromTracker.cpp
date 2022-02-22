#include <arips_navigation/utils/StepModuleFromTracker.h>

using namespace toponav_ros;

StepModuleFromTracker::StepModuleFromTracker(PlanningContext& context,
                                             FloorStepTracker& tracker,
                                             StepEdgeModule& stepModule)
    : PlanningContextHolder{context}, mTracker{tracker}, mStepModule{stepModule} {}

void StepModuleFromTracker::visualizeEdge(const toponav_core::TopoMap::Edge* edge) {
    // Nothing to do
}
void StepModuleFromTracker::initializeVisualization(MapEditor* editor) {
    EdgeVisualizationInterface::initializeVisualization(editor);

    mapEditor->getMenuHandler().insert(
        mapEditor->getCreateEdgesMenuHandle(), "Auto-generate step edges from tracked steps",
        [this](const interactive_markers::MenuHandler::FeedbackConstPtr&) { parseStepsFromTracker(); });
}

namespace toponav_ros {
    std::string findFreeStepName(StepEdgeModule::MapStepData const& stepsData);
}

void StepModuleFromTracker::parseStepsFromTracker() {
    const auto steps = mTracker.allSteps();

    for (const auto& s : steps) {
        tf2::Stamped<tf2::Vector3> start(tf2::Vector3(s[0].x(), s[0].y(), 0.0), ros::Time::now(),
                                         _context.globalFrame);
        tf2::Stamped<tf2::Vector3> end(tf2::Vector3(s[1].x(), s[1].y(), 0.0), ros::Time::now(),
                                       _context.globalFrame);

        auto& stepsData = StepEdgeModule::getMapData(mapEditor->getMap());
        StepEdgeModule::createNewMapStep(mapEditor->getMap(),
                                                      findFreeStepName(stepsData), start, end);

        _context.mapChanged();
    }
}
