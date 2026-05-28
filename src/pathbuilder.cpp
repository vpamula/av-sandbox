#include "pathbuilder.h"
#include "trajectorygenerator.h"

Path PathBuilder::connectLanes(Lane& entryLane, Lane& exitLane) {
    Path path;
    Vector2 entryEnd = entryLane.getEndPoint();
    Vector2 exitStart = exitLane.getStartPoint();
    Vector2 entryTrim = {entryEnd.x - entryLane.getDirection().x * 100,
        entryEnd.y - entryLane.getDirection().y * 100};
    Vector2 exitTrim = {exitStart.x + exitLane.getDirection().x * 200,
        exitStart.y + exitLane.getDirection().y * 200};

    std::vector<Vector2> connectorWaypoints =
            TrajectoryGenerator::generateConnector(
                entryTrim, entryLane.getDirection(),
                exitTrim, exitLane.getDirection(),
                20.0f, 5);

    for (Vector2 wp : connectorWaypoints) {
        path.addWaypoint(wp);
    }

    return path;
}

Path PathBuilder::buildRoute(const Route& route) {
    Path fullPath;

    const std::vector<Lane*>& lanes = route.getLanes();
    if (lanes.empty()) {
        return fullPath;
    }

    // Follow first lane fully
    std::vector<Vector2> initialWaypoints = lanes[0]->generateWaypoints();

    for (Vector2 wp : initialWaypoints) {
        fullPath.addWaypoint(wp);
    }

    // Stitch remaining route
    for (int i = 0; i < lanes.size() - 1; i++) {
        Lane* currentLane = lanes[i];
        Lane* nextLane = lanes[i + 1];

        // Generate connector
        Path connector = connectLanes(*currentLane, *nextLane);
        for (Vector2 wp : connector.getWaypoints()) {
            fullPath.addWaypoint(wp);
        }

        // Continue along next lane
        std::vector<Vector2> nextLaneWaypoints = nextLane->generateWaypoints();

        for (Vector2 wp :
             nextLaneWaypoints) {

            fullPath.addWaypoint(wp);
        }
    }

    return fullPath;
}