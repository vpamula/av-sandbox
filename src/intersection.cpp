#include "intersection.h"

#include "raymath.h"

#include <limits>
#include <utility>

Intersection::Intersection(Rectangle bounds, std::vector<Vector2> surfacePolygon)
    : bounds(bounds), surfacePolygon(std::move(surfacePolygon)) {}

Rectangle Intersection::getBounds() const { return bounds; }

bool Intersection::containsPoint(Vector2 point) const {
    if (surfacePolygon.size() < 3) {
        return false;
    }

    bool inside = false;
    for (std::size_t current = 0, previous = surfacePolygon.size() - 1;
         current < surfacePolygon.size();
         previous = current++) {
        const Vector2 a = surfacePolygon[current];
        const Vector2 b = surfacePolygon[previous];
        const bool crossesHorizontalRay = (a.y > point.y) != (b.y > point.y);
        if (!crossesHorizontalRay) {
            continue;
        }

        const float crossingX = (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
        if (point.x < crossingX) {
            inside = !inside;
        }
    }

    return inside;
}

void Intersection::Draw() const {
    if (surfacePolygon.size() < 3) {
        return;
    }

    Vector2 center = { 0.0f, 0.0f };
    for (const Vector2 point : surfacePolygon) {
        center = Vector2Add(center, point);
    }
    center = Vector2Scale(center, 1.0f / static_cast<float>(surfacePolygon.size()));

    float signedArea = 0.0f;
    for (std::size_t index = 0; index < surfacePolygon.size(); ++index) {
        const Vector2 next = surfacePolygon[(index + 1) % surfacePolygon.size()];
        signedArea += surfacePolygon[index].x * next.y - next.x * surfacePolygon[index].y;
    }

    std::vector<Vector2> fanPoints;
    fanPoints.reserve(surfacePolygon.size() + 2);
    fanPoints.push_back(center);
    // Raylib's 2D projection flips the y axis, and backface culling is enabled.
    // Keep the world-space perimeter clockwise so it becomes front-facing after projection.
    if (signedArea < 0.0f) {
        fanPoints.insert(fanPoints.end(), surfacePolygon.begin(), surfacePolygon.end());
    }
    else {
        fanPoints.insert(fanPoints.end(), surfacePolygon.rbegin(), surfacePolygon.rend());
    }
    fanPoints.push_back(fanPoints[1]);

    DrawTriangleFan(fanPoints.data(), static_cast<int>(fanPoints.size()), GRAY);
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

    const Vector2 center = {
        bounds.x + bounds.width * 0.5f,
        bounds.y + bounds.height * 0.5f
    };
    auto directionAtIntersection = [&](const Lane* lane) {
        const std::vector<Vector2>& waypoints = lane->getWaypoints();
        std::size_t nearestIndex = 0;
        float nearestDistance = std::numeric_limits<float>::infinity();

        for (std::size_t index = 0; index < waypoints.size(); ++index) {
            const float distance = Vector2Distance(center, waypoints[index]);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestIndex = index;
            }
        }

        return lane->getDirectionAt(nearestIndex);
    };

    const Vector2 incomingDirection = directionAtIntersection(incomingLane);
    const Vector2 outgoingDirection = directionAtIntersection(outgoingLane);
    const float directionDot = Vector2DotProduct(incomingDirection, outgoingDirection);

    // The current road model has one lane in each direction. Allow straight,
    // left, and right movements; reject only a near-180-degree U-turn.
    return directionDot > -0.8f;
}
