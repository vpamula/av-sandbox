#pragma once

#include "lane.h"
#include "raylib.h"

#include <unordered_map>
#include <vector>

class Intersection {
public:
    Intersection(Rectangle bounds, std::vector<Vector2> surfacePolygon);

    Rectangle getBounds() const;
    bool containsPoint(Vector2 point) const;
    void Draw() const;
    void addLane(Lane* lane);
    const std::vector<Lane*>& getConnectedLanes() const;
    void computeTransitions();
    const std::vector<Lane*>& getNextLanes(const Lane* incomingLane) const;

private:
    bool isValidTransition(const Lane* incomingLane, const Lane* outgoingLane) const;

    Rectangle bounds;
    std::vector<Vector2> surfacePolygon;
    std::vector<Lane*> connectedLanes;
    std::unordered_map<const Lane*, std::vector<Lane*>> transitions;
};
