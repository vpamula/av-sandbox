#include "stopsigncoordinator.h"

#include <algorithm>

namespace {
constexpr float CLEARANCE_MARGIN = 12.0f;

bool sameVehicle(const StopSignParticipant& first, const StopSignParticipant& second) {
    return first.vehicle == second.vehicle;
}

bool hasClearedIntersection(
    const World& world,
    std::size_t intersectionIndex,
    const StopSignParticipant& participant
) {
    const std::optional<Rectangle> bounds = world.getIntersectionBounds(intersectionIndex);
    if (!bounds || !participant.vehicle) {
        return true;
    }

    const float clearance = participant.vehicle->getLength() * 0.5f + CLEARANCE_MARGIN;
    const float x = participant.vehicle->getX();
    const float y = participant.vehicle->getY();
    return x < bounds->x - clearance ||
        x > bounds->x + bounds->width + clearance ||
        y < bounds->y - clearance ||
        y > bounds->y + bounds->height + clearance;
}
}

void StopSignCoordinator::update(
    const World& world,
    const std::vector<StopSignParticipant>& participants
) {
    for (const StopSignParticipant& participant : participants) {
        if (participant.controller) {
            participant.controller->setStopSignPermission(false);
        }
    }

    for (auto junction = junctions.begin(); junction != junctions.end();) {
        JunctionQueue& queue = junction->second;
        if (queue.active && hasClearedIntersection(world, junction->first, *queue.active)) {
            queue.active.reset();
        }

        queue.waiting.erase(
            std::remove_if(
                queue.waiting.begin(),
                queue.waiting.end(),
                [&](const StopSignParticipant& participant) {
                    if (!participant.vehicle || !participant.controller) {
                        return true;
                    }
                    const auto waitingAt = participant.controller->getWaitingStopSignIntersectionIndex();
                    return !waitingAt || *waitingAt != junction->first;
                }
            ),
            queue.waiting.end()
        );

        if (!queue.active && queue.waiting.empty()) {
            junction = junctions.erase(junction);
        }
        else {
            ++junction;
        }
    }

    for (const StopSignParticipant& participant : participants) {
        if (!participant.vehicle || !participant.controller) {
            continue;
        }

        const auto intersectionIndex = participant.controller->getWaitingStopSignIntersectionIndex();
        if (!intersectionIndex) {
            continue;
        }

        JunctionQueue& queue = junctions[*intersectionIndex];
        if ((queue.active && sameVehicle(*queue.active, participant)) ||
            std::any_of(queue.waiting.begin(), queue.waiting.end(), [&](const StopSignParticipant& queued) {
                return sameVehicle(queued, participant);
            })) {
            continue;
        }
        queue.waiting.push_back(participant);
    }

    for (auto& [intersectionIndex, queue] : junctions) {
        if (!queue.active && !queue.waiting.empty()) {
            queue.active = queue.waiting.front();
            queue.waiting.pop_front();
        }

        if (queue.active && queue.active->controller) {
            const auto waitingAt = queue.active->controller->getWaitingStopSignIntersectionIndex();
            if (waitingAt && *waitingAt == intersectionIndex) {
                queue.active->controller->setStopSignPermission(true);
            }
        }
    }
}
