#include "intersection.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
constexpr float VEHICLE_STOP_BUFFER = 6.0f;
constexpr float STOP_LINE_HALF_WIDTH = 38.0f;
constexpr float STOP_SIGN_SHOULDER_OFFSET = 58.0f;
constexpr float STOP_SIGN_RADIUS = 20.0f;
constexpr float TRAFFIC_LIGHT_SHOULDER_OFFSET = 82.0f;
constexpr float TRAFFIC_LIGHT_CYCLE_SECONDS = 22.0f;
constexpr int CROSSWALK_STRIPE_COUNT = 4;
constexpr float CROSSWALK_FIRST_STRIPE_OFFSET = 8.0f;
constexpr float CROSSWALK_STRIPE_SPACING = 12.0f;
constexpr float CROSSWALK_STRIPE_WIDTH = 5.0f;

void drawSignalLamp(Vector2 center, bool active, Color lit, Color glow, Color unlit) {
    if (active) {
        DrawCircleV(center, 12.0f, glow);
    }
    DrawCircleV(center, 8.0f, active ? lit : unlit);
}

Vector2 interpolate(Vector2 start, Vector2 end, float amount) {
    return Vector2Add(start, Vector2Scale(Vector2Subtract(end, start), amount));
}
}

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

std::vector<std::vector<Vector2>> Intersection::getPedestrianCrossings(float sidewalkExtension) const {
    std::vector<std::vector<Vector2>> crossings;
    if (surfacePolygon.size() != 4) {
        return crossings;
    }

    Vector2 center = { 0.0f, 0.0f };
    for (const Vector2 point : surfacePolygon) {
        center = Vector2Add(center, point);
    }
    center = Vector2Scale(center, 0.25f);

    auto normalized = [](Vector2 value) {
        const float length = Vector2Length(value);
        return length > 0.0001f ? Vector2Scale(value, 1.0f / length) : Vector2{ 0.0f, 0.0f };
    };

    for (std::size_t edgeIndex = 0; edgeIndex < 2; ++edgeIndex) {
        const Vector2 firstEdgeStart = surfacePolygon[edgeIndex];
        const Vector2 firstEdgeEnd = surfacePolygon[(edgeIndex + 1) % 4];
        const Vector2 oppositeEdgeStart = surfacePolygon[(edgeIndex + 2) % 4];
        const Vector2 oppositeEdgeEnd = surfacePolygon[(edgeIndex + 3) % 4];
        const Vector2 firstMidpoint = Vector2Scale(Vector2Add(firstEdgeStart, firstEdgeEnd), 0.5f);
        const Vector2 oppositeMidpoint = Vector2Scale(
            Vector2Add(oppositeEdgeStart, oppositeEdgeEnd),
            0.5f
        );
        const Vector2 firstOutward = normalized(Vector2Subtract(firstMidpoint, center));
        const Vector2 oppositeOutward = normalized(Vector2Subtract(oppositeMidpoint, center));
        const Vector2 firstInner = interpolate(firstMidpoint, center, 0.5f);
        const Vector2 oppositeInner = interpolate(oppositeMidpoint, center, 0.5f);

        crossings.push_back({
            Vector2Add(firstMidpoint, Vector2Scale(firstOutward, sidewalkExtension)),
            firstMidpoint,
            firstInner,
            center,
            oppositeInner,
            oppositeMidpoint,
            Vector2Add(oppositeMidpoint, Vector2Scale(oppositeOutward, sidewalkExtension))
        });
    }

    return crossings;
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

    for (std::size_t edgeIndex = 0; edgeIndex < surfacePolygon.size(); ++edgeIndex) {
        const Vector2 edgeStart = surfacePolygon[edgeIndex];
        const Vector2 edgeEnd = surfacePolygon[(edgeIndex + 1) % surfacePolygon.size()];
        const Vector2 edge = Vector2Subtract(edgeEnd, edgeStart);
        const float edgeLength = Vector2Length(edge);
        if (edgeLength <= 0.0001f) {
            continue;
        }

        const Vector2 edgeDirection = Vector2Scale(edge, 1.0f / edgeLength);
        const Vector2 inwardDirection = Vector2Scale(
            Vector2Subtract(center, Vector2Scale(Vector2Add(edgeStart, edgeEnd), 0.5f)),
            1.0f / std::max(Vector2Distance(center, Vector2Scale(Vector2Add(edgeStart, edgeEnd), 0.5f)), 0.0001f)
        );
        const Vector2 halfCrossing = Vector2Scale(edgeDirection, std::max(0.0f, edgeLength * 0.5f - 6.0f));
        const Vector2 edgeCenter = Vector2Scale(Vector2Add(edgeStart, edgeEnd), 0.5f);
        for (int stripe = 0; stripe < CROSSWALK_STRIPE_COUNT; ++stripe) {
            const Vector2 stripeCenter = Vector2Add(
                edgeCenter,
                Vector2Scale(
                    inwardDirection,
                    CROSSWALK_FIRST_STRIPE_OFFSET + stripe * CROSSWALK_STRIPE_SPACING
                )
            );
            DrawLineEx(
                Vector2Subtract(stripeCenter, halfCrossing),
                Vector2Add(stripeCenter, halfCrossing),
                CROSSWALK_STRIPE_WIDTH,
                WHITE
            );
        }
    }

    for (const StopApproach& approach : stopApproaches) {
        const Vector2 rightSide = { approach.direction.y, -approach.direction.x };
        DrawLineEx(approach.lineStart, approach.lineEnd, 6.0f, WHITE);

        if (stopControlled) {
            const Vector2 signCenter = Vector2Add(
                approach.lineCenter,
                Vector2Scale(rightSide, STOP_SIGN_SHOULDER_OFFSET)
            );
            Vector2 octagon[10];
            octagon[0] = signCenter;
            for (int index = 0; index <= 8; ++index) {
                const float angle = -static_cast<float>(index) * PI / 4.0f;
                octagon[index + 1] = {
                    signCenter.x + STOP_SIGN_RADIUS * std::cos(angle),
                    signCenter.y + STOP_SIGN_RADIUS * std::sin(angle)
                };
            }

            DrawTriangleFan(octagon, 10, RED);
            for (int index = 1; index <= 8; ++index) {
                DrawLineEx(octagon[index], octagon[index + 1], 2.0f, WHITE);
            }
            DrawText("STOP", static_cast<int>(signCenter.x - 14.0f),
                static_cast<int>(signCenter.y - 4.0f), 8, WHITE);
        }

        if (trafficLightControlled) {
            const Vector2 lightCenter = Vector2Add(
                approach.lineCenter,
                Vector2Scale(rightSide, TRAFFIC_LIGHT_SHOULDER_OFFSET)
            );
            const TrafficLightAspect aspect = getTrafficLightAspect(
                std::abs(approach.direction.x) >= std::abs(approach.direction.y)
            );
            DrawRectangleRec({ lightCenter.x - 15.0f, lightCenter.y - 40.0f, 30.0f, 80.0f }, BLACK);
            drawSignalLamp(
                { lightCenter.x, lightCenter.y - 25.0f },
                aspect == TrafficLightAspect::Red,
                { 255, 28, 40, 255 },
                { 105, 12, 20, 255 },
                { 48, 10, 14, 255 }
            );
            drawSignalLamp(
                lightCenter,
                aspect == TrafficLightAspect::Yellow,
                { 255, 242, 0, 255 },
                { 120, 105, 0, 255 },
                { 44, 38, 8, 255 }
            );
            drawSignalLamp(
                { lightCenter.x, lightCenter.y + 25.0f },
                aspect == TrafficLightAspect::Green,
                { 30, 255, 72, 255 },
                { 8, 112, 30, 255 },
                { 8, 38, 17, 255 }
            );
        }
    }
}

void Intersection::setStopControlled(bool enabled) {
    stopControlled = enabled;
    if (enabled) {
        trafficLightControlled = false;
    }
    rebuildStopApproaches();
}

bool Intersection::isStopControlled() const {
    return stopControlled;
}

void Intersection::setTrafficLightControlled(bool enabled, float phaseOffsetSeconds) {
    trafficLightControlled = enabled;
    if (enabled) {
        stopControlled = false;
        trafficLightElapsed = std::fmod(
            std::max(0.0f, phaseOffsetSeconds),
            TRAFFIC_LIGHT_CYCLE_SECONDS
        );
    }
    rebuildStopApproaches();
}

bool Intersection::isTrafficLightControlled() const {
    return trafficLightControlled;
}

void Intersection::updateTrafficLight(float dt) {
    if (!trafficLightControlled || dt <= 0.0f) {
        return;
    }

    trafficLightElapsed = std::fmod(trafficLightElapsed + dt, TRAFFIC_LIGHT_CYCLE_SECONDS);
}

TrafficLightAspect Intersection::getTrafficLightAspect(bool horizontalAxis) const {
    if (!trafficLightControlled) {
        return TrafficLightAspect::Green;
    }

    if (horizontalAxis) {
        if (trafficLightElapsed < 8.0f) return TrafficLightAspect::Green;
        if (trafficLightElapsed < 10.0f) return TrafficLightAspect::Yellow;
        return TrafficLightAspect::Red;
    }

    if (trafficLightElapsed >= 11.0f && trafficLightElapsed < 19.0f) {
        return TrafficLightAspect::Green;
    }
    if (trafficLightElapsed >= 19.0f && trafficLightElapsed < 21.0f) {
        return TrafficLightAspect::Yellow;
    }
    return TrafficLightAspect::Red;
}

std::optional<RouteControlPoint> Intersection::getRouteControlAlongRoute(
    const std::vector<Vector2>& route,
    float vehicleLength
) const {
    if ((!stopControlled && !trafficLightControlled) || route.size() < 2 || containsPoint(route.front())) {
        return std::nullopt;
    }

    float distanceBeforeSegment = 0.0f;
    for (std::size_t index = 1; index < route.size(); ++index) {
        const Vector2 start = route[index - 1];
        const Vector2 end = route[index];
        const float segmentLength = Vector2Distance(start, end);
        if (segmentLength <= 0.0001f) {
            continue;
        }

        float previousAmount = 0.0f;
        const int samples = std::max(1, static_cast<int>(std::ceil(segmentLength / 10.0f)));
        for (int sample = 1; sample <= samples; ++sample) {
            const float amount = static_cast<float>(sample) / static_cast<float>(samples);
            if (!containsPoint(interpolate(start, end, amount))) {
                previousAmount = amount;
                continue;
            }

            float outsideAmount = previousAmount;
            float insideAmount = amount;
            for (int refinement = 0; refinement < 10; ++refinement) {
                const float midpoint = (outsideAmount + insideAmount) * 0.5f;
                if (containsPoint(interpolate(start, end, midpoint))) {
                    insideAmount = midpoint;
                }
                else {
                    outsideAmount = midpoint;
                }
            }

            const float entryDistance = distanceBeforeSegment + segmentLength * insideAmount;
            const float vehicleCenterOffset = vehicleLength * 0.5f + VEHICLE_STOP_BUFFER;
            const Vector2 entryPoint = interpolate(start, end, insideAmount);
            Vector2 approachDirection = Vector2Subtract(end, start);
            float nearestApproachDistance = std::numeric_limits<float>::infinity();
            for (const StopApproach& approach : stopApproaches) {
                const float distance = Vector2Distance(entryPoint, approach.lineCenter);
                if (distance < nearestApproachDistance) {
                    nearestApproachDistance = distance;
                    approachDirection = approach.direction;
                }
            }
            const bool horizontalAxis = std::abs(approachDirection.x) >= std::abs(approachDirection.y);
            return RouteControlPoint{
                std::max(0.0f, entryDistance - vehicleCenterOffset),
                stopControlled ? RouteControlType::StopSign : RouteControlType::TrafficLight,
                horizontalAxis
            };
        }

        distanceBeforeSegment += segmentLength;
    }

    return std::nullopt;
}

void Intersection::rebuildStopApproaches() {
    stopApproaches.clear();
    if (!stopControlled && !trafficLightControlled) {
        return;
    }

    for (const Lane* lane : connectedLanes) {
        if (!lane) {
            continue;
        }

        const std::vector<Vector2>& waypoints = lane->getWaypoints();
        if (waypoints.size() < 2) {
            continue;
        }

        bool previousInside = containsPoint(waypoints.front());
        for (std::size_t index = 1; index < waypoints.size(); ++index) {
            const bool currentInside = containsPoint(waypoints[index]);
            if (currentInside && !previousInside) {
                float outsideAmount = 0.0f;
                float insideAmount = 1.0f;
                for (int refinement = 0; refinement < 10; ++refinement) {
                    const float midpoint = (outsideAmount + insideAmount) * 0.5f;
                    if (containsPoint(interpolate(waypoints[index - 1], waypoints[index], midpoint))) {
                        insideAmount = midpoint;
                    }
                    else {
                        outsideAmount = midpoint;
                    }
                }

                const Vector2 entryPoint = interpolate(
                    waypoints[index - 1],
                    waypoints[index],
                    insideAmount
                );
                const Vector2 direction = lane->getDirectionAt(index);

                Vector2 lineCenter = entryPoint;
                Vector2 lineDirection = { direction.y, -direction.x };
                float closestBoundaryDistanceSquared = std::numeric_limits<float>::infinity();
                for (std::size_t edgeIndex = 0; edgeIndex < surfacePolygon.size(); ++edgeIndex) {
                    const Vector2 edgeStart = surfacePolygon[edgeIndex];
                    const Vector2 edge = Vector2Subtract(
                        surfacePolygon[(edgeIndex + 1) % surfacePolygon.size()],
                        edgeStart
                    );
                    const float edgeLengthSquared = Vector2LengthSqr(edge);
                    if (edgeLengthSquared <= 0.0001f) {
                        continue;
                    }

                    const float edgeAmount = std::clamp(
                        Vector2DotProduct(Vector2Subtract(entryPoint, edgeStart), edge) / edgeLengthSquared,
                        0.0f,
                        1.0f
                    );
                    const Vector2 boundaryPoint = Vector2Add(edgeStart, Vector2Scale(edge, edgeAmount));
                    const float distanceSquared = Vector2LengthSqr(Vector2Subtract(entryPoint, boundaryPoint));
                    if (distanceSquared < closestBoundaryDistanceSquared) {
                        closestBoundaryDistanceSquared = distanceSquared;
                        lineCenter = boundaryPoint;
                        lineDirection = Vector2Scale(edge, 1.0f / std::sqrt(edgeLengthSquared));
                    }
                }

                const Vector2 lineHalfWidth = Vector2Scale(lineDirection, STOP_LINE_HALF_WIDTH);
                stopApproaches.push_back({
                    lineCenter,
                    Vector2Subtract(lineCenter, lineHalfWidth),
                    Vector2Add(lineCenter, lineHalfWidth),
                    direction
                });
                break;
            }

            previousInside = currentInside;
        }
    }
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
