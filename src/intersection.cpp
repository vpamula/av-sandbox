#include "intersection.h"

Intersection::Intersection(Rectangle bounds) : bounds(bounds) {}

Rectangle Intersection::getBounds() const { return bounds; }

void Intersection::Draw() const {
    DrawRectangleRec(bounds, GRAY);
}

void Intersection::addLane(Lane* lane) {
    connectedLanes.push_back(lane);
}

const std::vector<Lane*>& Intersection::getConnectedLanes() const {
    return connectedLanes;
}

void Intersection::computeTransitions() {
    transitions.clear();

    for (Lane* incomingLane : connectedLanes) {
        auto& outgoingLanes = transitions[incomingLane];
        for (Lane* outgoingLane : connectedLanes) {
            if (isValidTransition(incomingLane, outgoingLane)) {
                outgoingLanes.push_back(outgoingLane);
            }
        }
    }
}

const std::vector<Lane*>& Intersection::getNextLanes(const Lane* incomingLane) const {
    static const std::vector<Lane*> noTransitions;
    const auto transition = transitions.find(incomingLane);
    return transition == transitions.end() ? noTransitions : transition->second;
}

bool Intersection::isValidTransition(const Lane* incomingLane, const Lane* outgoingLane) const {
    if (!incomingLane || !outgoingLane || incomingLane == outgoingLane) {
        return false;
    }

    const Vector2 incomingDirection = incomingLane->getDirection();
    const Vector2 outgoingDirection = outgoingLane->getDirection();
    const float directionDot = incomingDirection.x * outgoingDirection.x +
        incomingDirection.y * outgoingDirection.y;

    // The current road model has one lane in each direction. Allow straight,
    // left, and right movements; reject only a near-180-degree U-turn.
    return directionDot > -0.8f;
}
