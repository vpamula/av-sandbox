#include "pathbuilder.h"
#include "trajectorygenerator.h"

ConnectionData PathBuilder::connectLanes(Lane& entryLane, Lane& exitLane) {
    ConnectionData data;
    Vector2 entryStart = entryLane.getStartPoint();
    Vector2 entryEnd = entryLane.getEndPoint();
    Vector2 exitStart = exitLane.getStartPoint();
    Vector2 exitEnd = exitLane.getEndPoint();
    data.entryTrim = {entryEnd.x - entryLane.getDirection().x * 100,
        entryEnd.y - entryLane.getDirection().y * 100};
    data.exitTrim = {exitStart.x + exitLane.getDirection().x * 200,
        exitStart.y + exitLane.getDirection().y * 200};

    std::vector<Vector2> entryWaypoints = entryLane.generateWaypointsBetween(entryStart,
            data.entryTrim);

    std::vector<Vector2> connectorWaypoints = TrajectoryGenerator::generateConnector(
            data.entryTrim, entryLane.getDirection(), data.exitTrim, 
            exitLane.getDirection(), 20.0f, 5);

    std::vector<Vector2> exitWaypoints = exitLane.generateWaypointsBetween(data.exitTrim,
            exitEnd);

    for (Vector2 wp : entryWaypoints) {
        data.path.addWaypoint(wp);
    }

    for (Vector2 wp : connectorWaypoints) {
        data.path.addWaypoint(wp);
    }

    for (Vector2 wp : exitWaypoints) {
        data.path.addWaypoint(wp);
    }

    return data;

}