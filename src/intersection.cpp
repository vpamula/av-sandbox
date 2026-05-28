#include "intersection.h"
#include <iostream>

Intersection::Intersection(Rectangle bounds) {
    this->bounds = bounds;
}

Rectangle Intersection::getBounds() const {
    return bounds;
}

void Intersection::Draw() {
    DrawRectangleRec(bounds,GRAY);
}

void Intersection::addLane(Lane* lane) {
    connectedLanes.push_back(lane);
}

std::vector<Lane*>& Intersection::getConnectedLanes() {
    return connectedLanes;
}

void Intersection::computeTransitions() {
    transitions.clear();
    for (Lane* incoming : connectedLanes) {
        std::vector<std::pair<Lane*, TurnType>>
            nextLanes;
        std::cout << "Incoming lane: "
                  << incoming->getId()
                  << "\n";
        for (Lane* outgoing : connectedLanes) {
            if (incoming == outgoing) {
                continue;
            }
            TurnType turnType =
                classifyTurn(
                    incoming,
                    outgoing
                );
            if (turnType ==
                TurnType::INVALID) {
                continue;
            }
            nextLanes.push_back({
                outgoing,
                turnType
            });
            std::string turnString;
            switch (turnType) {
                case TurnType::LEFT:
                    turnString = "LEFT";
                    break;
                case TurnType::RIGHT:
                    turnString = "RIGHT";
                    break;
                case TurnType::STRAIGHT:
                    turnString = "STRAIGHT";
                    break;
                default:
                    turnString = "INVALID";
            }
            std::cout
                << "    -> Lane "
                << outgoing->getId()
                << " ("
                << turnString
                << ")\n";
        }
        transitions[incoming] = nextLanes;
    }
}

const std::vector<std::pair<Lane*, TurnType>>& Intersection::getNextLanes(Lane* incomingLane) {
    static const std::vector<std::pair<Lane*, TurnType>> empty;    
    auto it = transitions.find(incomingLane);
    if (it == transitions.end()) {
        return empty;
    }
    return it->second;
}

TurnType Intersection::classifyTurn(Lane* incoming, Lane* outgoing) {
    Vector2 inDir = incoming->getDirection();
    Vector2 outDir = outgoing->getDirection();

    float dot = inDir.x * outDir.x + inDir.y * outDir.y;

    float cross = inDir.x * outDir.y - inDir.y * outDir.x;

    if (dot < -0.8f) {
        return TurnType::INVALID;
    }

    if (dot > 0.8f) {
        return TurnType::STRAIGHT;
    }

    if (cross > 0.0f) {
        return TurnType::LEFT;
    }

    return TurnType::RIGHT;
}

bool Intersection::isValidTransition(Lane* incoming, Lane* outgoing) {
    Vector2 inDir = incoming->getDirection();
    Vector2 outDir = outgoing->getDirection();
    float dot = inDir.x * outDir.x + inDir.y * outDir.y;
    if (dot < -0.8f) {
        return false;
    }
    return true;
}