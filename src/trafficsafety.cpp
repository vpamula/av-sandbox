#include "trafficsafety.h"

#include "raymath.h"
#include "world.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace {
constexpr float PREDICTION_HORIZON_SECONDS = 3.5f;
constexpr float PREDICTION_STEP_SECONDS = 0.02f;
constexpr float MAX_SAFETY_DECELERATION = 700.0f;
constexpr float COLLISION_CLEARANCE = 3.0f;
constexpr float JUNCTION_APPROACH_MARGIN = 40.0f;
constexpr std::size_t RESERVATION_REORDER_MARGIN_STEPS = 2;
constexpr float ANGLE_PI = 3.14159265358979323846f;

enum class MotionMode {
    Normal,
    Braking,
    Stopped
};

struct Pose {
    Vector2 position;
    float heading;
};

using Trajectory = std::vector<Pose>;

Pose poseOf(const Vehicle& vehicle) {
    return { { vehicle.getX(), vehicle.getY() }, vehicle.getHeading() };
}

float wrapAngle(float angle) {
    while (angle > ANGLE_PI) angle -= 2.0f * ANGLE_PI;
    while (angle < -ANGLE_PI) angle += 2.0f * ANGLE_PI;
    return angle;
}

Pose interpolatePose(Pose start, Pose end, float amount) {
    return {
        Vector2Add(start.position, Vector2Scale(Vector2Subtract(end.position, start.position), amount)),
        start.heading + wrapAngle(end.heading - start.heading) * amount
    };
}

float speedLimitForMode(const Vehicle& vehicle, MotionMode mode, float dt) {
    if (mode == MotionMode::Stopped) {
        return 0.0f;
    }
    if (mode == MotionMode::Braking) {
        return std::max(0.0f, vehicle.getSpeed() - MAX_SAFETY_DECELERATION * dt);
    }
    return std::numeric_limits<float>::infinity();
}

Trajectory predictTrajectory(
    const Vehicle& vehicle,
    bool manualControl,
    MotionMode mode,
    float dt
) {
    Vehicle prediction = vehicle;
    Trajectory trajectory;
    const std::size_t futureSteps = static_cast<std::size_t>(
        std::ceil(PREDICTION_HORIZON_SECONDS / PREDICTION_STEP_SECONDS)
    );
    const std::size_t currentSteps = std::max<std::size_t>(
        1,
        static_cast<std::size_t>(std::ceil(dt / PREDICTION_STEP_SECONDS))
    );
    trajectory.reserve(futureSteps + currentSteps + 1);
    const Pose startPose = poseOf(vehicle);
    trajectory.push_back(startPose);

    prediction.Update(dt, manualControl, speedLimitForMode(prediction, mode, dt));
    const Pose endPose = poseOf(prediction);
    for (std::size_t step = 1; step <= currentSteps; ++step) {
        trajectory.push_back(interpolatePose(
            startPose,
            endPose,
            static_cast<float>(step) / static_cast<float>(currentSteps)
        ));
    }

    const float futureStep = PREDICTION_HORIZON_SECONDS / static_cast<float>(futureSteps);
    for (std::size_t step = 0; step < futureSteps; ++step) {
        prediction.Update(
            futureStep,
            manualControl,
            speedLimitForMode(prediction, mode, futureStep)
        );
        trajectory.push_back(poseOf(prediction));
    }

    return trajectory;
}

bool rectanglesOverlap(const Vehicle& first, Pose firstPose, const Vehicle& second, Pose secondPose) {
    const Vector2 firstForward = { std::cos(firstPose.heading), std::sin(firstPose.heading) };
    const Vector2 firstSide = { -firstForward.y, firstForward.x };
    const Vector2 secondForward = { std::cos(secondPose.heading), std::sin(secondPose.heading) };
    const Vector2 secondSide = { -secondForward.y, secondForward.x };
    const Vector2 centerOffset = Vector2Subtract(secondPose.position, firstPose.position);
    const Vector2 axes[] = { firstForward, firstSide, secondForward, secondSide };
    const bool firstIsPedestrian = first.getLength() <= 20.0f && first.getWidth() <= 20.0f;
    const bool secondIsPedestrian = second.getLength() <= 20.0f && second.getWidth() <= 20.0f;
    const float collisionClearance = (firstIsPedestrian && !secondIsPedestrian) ||
        (secondIsPedestrian && !firstIsPedestrian)
        ? 5.0f
        : COLLISION_CLEARANCE;

    for (const Vector2 axis : axes) {
        const float centerDistance = std::abs(Vector2DotProduct(centerOffset, axis));
        const float firstRadius =
            first.getLength() * 0.5f * std::abs(Vector2DotProduct(firstForward, axis)) +
            first.getWidth() * 0.5f * std::abs(Vector2DotProduct(firstSide, axis)) +
            collisionClearance;
        const float secondRadius =
            second.getLength() * 0.5f * std::abs(Vector2DotProduct(secondForward, axis)) +
            second.getWidth() * 0.5f * std::abs(Vector2DotProduct(secondSide, axis)) +
            collisionClearance;
        if (centerDistance > firstRadius + secondRadius) {
            return false;
        }
    }

    return true;
}

bool trajectoriesCollide(
    const Vehicle& first,
    const Trajectory& firstPath,
    const Vehicle& second,
    const Trajectory& secondPath
) {
    const std::size_t sampleCount = std::min(firstPath.size(), secondPath.size());
    for (std::size_t index = 1; index < sampleCount; ++index) {
        if (rectanglesOverlap(first, firstPath[index], second, secondPath[index])) {
            return true;
        }

        const Pose firstMidpoint = interpolatePose(firstPath[index - 1], firstPath[index], 0.5f);
        const Pose secondMidpoint = interpolatePose(secondPath[index - 1], secondPath[index], 0.5f);
        if (rectanglesOverlap(first, firstMidpoint, second, secondMidpoint)) {
            return true;
        }
    }

    return false;
}

std::vector<std::size_t> firstJunctionEntrySteps(
    const World& world,
    const Vehicle& vehicle,
    const Trajectory& trajectory,
    float approachMargin
) {
    const std::size_t noEntry = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> entrySteps(world.getIntersectionCount(), noEntry);

    for (std::size_t step = 0; step < trajectory.size(); ++step) {
        const Vector2 position = trajectory[step].position;
        for (std::size_t intersection = 0; intersection < entrySteps.size(); ++intersection) {
            if (entrySteps[intersection] != noEntry) {
                continue;
            }

            const std::optional<Rectangle> bounds = world.getIntersectionBounds(intersection);
            if (!bounds) {
                continue;
            }

            const float margin = approachMargin + vehicle.getLength() * 0.5f;
            if (position.x >= bounds->x - margin &&
                position.x <= bounds->x + bounds->width + margin &&
                position.y >= bounds->y - margin &&
                position.y <= bounds->y + bounds->height + margin) {
                entrySteps[intersection] = step;
            }
        }
    }

    return entrySteps;
}

std::optional<std::size_t> preferredJunctionYielder(
    const World& world,
    const Vehicle& first,
    const Trajectory& firstPath,
    const Vehicle& second,
    const Trajectory& secondPath,
    std::size_t firstIndex,
    std::size_t secondIndex
) {
    const std::size_t noEntry = std::numeric_limits<std::size_t>::max();
    const std::vector<std::size_t> firstEntries = firstJunctionEntrySteps(world, first, firstPath, 0.0f);
    const std::vector<std::size_t> secondEntries = firstJunctionEntrySteps(world, second, secondPath, 0.0f);
    const std::vector<std::size_t> firstApproaches = firstJunctionEntrySteps(
        world, first, firstPath, JUNCTION_APPROACH_MARGIN
    );
    const std::vector<std::size_t> secondApproaches = firstJunctionEntrySteps(
        world, second, secondPath, JUNCTION_APPROACH_MARGIN
    );

    const auto firstInside = world.findIntersectionContaining(firstPath.front().position);
    const auto secondInside = world.findIntersectionContaining(secondPath.front().position);
    if (firstInside && !secondInside && secondApproaches[*firstInside] != noEntry) {
        return secondIndex;
    }
    if (secondInside && !firstInside && firstApproaches[*secondInside] != noEntry) {
        return firstIndex;
    }

    auto compareArrivalTimes = [&](
        const std::vector<std::size_t>& firstTimes,
        const std::vector<std::size_t>& secondTimes
    ) -> std::optional<std::size_t> {
        std::size_t earliestSharedJunctionArrival = noEntry;
        std::optional<std::size_t> yielder;
        for (std::size_t intersection = 0; intersection < firstTimes.size(); ++intersection) {
            const std::size_t firstEntry = firstTimes[intersection];
            const std::size_t secondEntry = secondTimes[intersection];
            if (firstEntry == noEntry || secondEntry == noEntry || firstEntry == secondEntry) {
                continue;
            }

            const std::size_t laterArrival = std::max(firstEntry, secondEntry);
            if (laterArrival < earliestSharedJunctionArrival) {
                earliestSharedJunctionArrival = laterArrival;
                yielder = firstEntry < secondEntry ? secondIndex : firstIndex;
            }
        }
        return yielder;
    };

    const std::optional<std::size_t> actualArrivalYielder =
        compareArrivalTimes(firstEntries, secondEntries);
    if (actualArrivalYielder) {
        return actualArrivalYielder;
    }
    return compareArrivalTimes(firstApproaches, secondApproaches);
}

MotionMode moreRestrictiveMode(MotionMode mode) {
    if (mode == MotionMode::Normal) {
        return MotionMode::Braking;
    }
    if (mode == MotionMode::Braking) {
        return MotionMode::Stopped;
    }
    return MotionMode::Stopped;
}
}

void TrafficSafety::step(
    const World& world,
    const std::vector<Vehicle*>& vehicles,
    bool manualEgoControl,
    float dt,
    std::size_t priorityParticipantStart,
    std::size_t pedestrianPriorityStart
) {
    if (vehicles.empty() || dt <= 0.0f) {
        return;
    }

    std::vector<MotionMode> modes(vehicles.size(), MotionMode::Normal);
    const auto manualControlFor = [&](std::size_t index) {
        return index == 0 && manualEgoControl;
    };
    std::vector<Trajectory> trajectories;
    trajectories.reserve(vehicles.size());
    for (std::size_t index = 0; index < vehicles.size(); ++index) {
        trajectories.push_back(predictTrajectory(
            *vehicles[index], manualControlFor(index), modes[index], dt
        ));
    }

    std::vector<bool> priorityAgentInsideJunction(vehicles.size(), false);
    for (std::size_t index = priorityParticipantStart; index < vehicles.size(); ++index) {
        const Vector2 position = { vehicles[index]->getX(), vehicles[index]->getY() };
        priorityAgentInsideJunction[index] = world.findIntersectionContaining(position).has_value();
    }

    // Uncontrolled intersections have no signal or stop-sign coordinator.
    // Give approaching cars a persistent, first-in reservation so pairwise
    // collision avoidance cannot repeatedly stop both cars at the junction.
    const std::size_t noEntry = std::numeric_limits<std::size_t>::max();
    std::vector<std::optional<std::size_t>> requestedJunctions(vehicles.size());
    std::vector<std::size_t> requestedArrivalSteps(vehicles.size(), noEntry);
    for (std::size_t index = 0; index < std::min(priorityParticipantStart, vehicles.size()); ++index) {
        const std::vector<std::size_t> approachSteps = firstJunctionEntrySteps(
            world, *vehicles[index], trajectories[index], JUNCTION_APPROACH_MARGIN
        );
        const std::vector<std::size_t> entrySteps = firstJunctionEntrySteps(
            world, *vehicles[index], trajectories[index], 0.0f
        );
        for (std::size_t intersection = 0; intersection < world.getIntersectionCount(); ++intersection) {
            if (world.isIntersectionControlled(intersection) ||
                approachSteps[intersection] != 0 ||
                entrySteps[intersection] == noEntry) {
                continue;
            }
            if (entrySteps[intersection] < requestedArrivalSteps[index]) {
                requestedJunctions[index] = intersection;
                requestedArrivalSteps[index] = entrySteps[intersection];
            }
        }
    }

    auto isInsideReservationClearance = [&](std::size_t intersection, const Vehicle& vehicle) {
        const std::optional<Rectangle> bounds = world.getIntersectionBounds(intersection);
        if (!bounds) {
            return false;
        }
        const float clearance = vehicle.getLength() * 0.5f + 12.0f;
        const float x = vehicle.getX();
        const float y = vehicle.getY();
        return x >= bounds->x - clearance && x <= bounds->x + bounds->width + clearance &&
            y >= bounds->y - clearance && y <= bounds->y + bounds->height + clearance;
    };

    for (auto iterator = uncontrolledJunctions.begin(); iterator != uncontrolledJunctions.end();) {
        const std::size_t intersection = iterator->first;
        JunctionReservation& reservation = iterator->second;
        if (reservation.owner) {
            const auto ownerIndex = std::find(vehicles.begin(), vehicles.end(), reservation.owner);
            if (ownerIndex == vehicles.end()) {
                reservation.owner = nullptr;
                reservation.ownerEntered = false;
            }
            else {
                const Vector2 ownerPosition = {
                    reservation.owner->getX(), reservation.owner->getY()
                };
                reservation.ownerEntered = reservation.ownerEntered ||
                    world.findIntersectionContaining(ownerPosition) == intersection;
                const std::size_t index = static_cast<std::size_t>(ownerIndex - vehicles.begin());
                const bool stillApproaching = index < requestedJunctions.size() &&
                    requestedJunctions[index] == intersection;
                if ((reservation.ownerEntered && !isInsideReservationClearance(intersection, *reservation.owner)) ||
                    (!reservation.ownerEntered && !stillApproaching)) {
                    reservation.owner = nullptr;
                    reservation.ownerEntered = false;
                }
                else if (!reservation.ownerEntered) {
                    // Before a vehicle enters, refresh its priority from the
                    // current predicted arrival times. A reservation claimed
                    // earlier must not make an earlier-arriving car stop.
                    std::size_t earliestIndex = index;
                    std::size_t earliestArrival = requestedArrivalSteps[index];
                    const std::size_t carCount = std::min(priorityParticipantStart, vehicles.size());
                    for (std::size_t candidate = 0; candidate < carCount; ++candidate) {
                        if (requestedJunctions[candidate] != intersection ||
                            requestedArrivalSteps[candidate] >= earliestArrival) {
                            continue;
                        }
                        earliestArrival = requestedArrivalSteps[candidate];
                        earliestIndex = candidate;
                    }

                    if (earliestIndex != index &&
                        earliestArrival + RESERVATION_REORDER_MARGIN_STEPS < requestedArrivalSteps[index]) {
                        reservation.owner = vehicles[earliestIndex];
                        reservation.ownerEntered = false;
                    }
                }
            }
        }

        reservation.waiting.erase(std::remove_if(
            reservation.waiting.begin(), reservation.waiting.end(), [&](const JunctionRequest& request) {
            const auto vehicle = std::find(vehicles.begin(), vehicles.end(), request.vehicle);
            if (vehicle == vehicles.end()) {
                return true;
            }
            const std::size_t index = static_cast<std::size_t>(vehicle - vehicles.begin());
            return index >= requestedJunctions.size() ||
                requestedJunctions[index] != intersection ||
                request.vehicle == reservation.owner;
        }), reservation.waiting.end());

        for (JunctionRequest& request : reservation.waiting) {
            const auto vehicle = std::find(vehicles.begin(), vehicles.end(), request.vehicle);
            if (vehicle != vehicles.end()) {
                const std::size_t index = static_cast<std::size_t>(vehicle - vehicles.begin());
                request.arrivalStep = requestedArrivalSteps[index];
            }
        }
        std::stable_sort(
            reservation.waiting.begin(),
            reservation.waiting.end(),
            [](const JunctionRequest& first, const JunctionRequest& second) {
                return first.arrivalStep < second.arrivalStep;
            }
        );

        if (!reservation.owner && reservation.waiting.empty()) {
            iterator = uncontrolledJunctions.erase(iterator);
        }
        else {
            ++iterator;
        }
    }

    std::vector<std::pair<std::size_t, JunctionRequest>> newRequests;
    for (std::size_t index = 0; index < requestedJunctions.size(); ++index) {
        if (!requestedJunctions[index]) {
            continue;
        }
        JunctionReservation& reservation = uncontrolledJunctions[*requestedJunctions[index]];
        Vehicle* vehicle = vehicles[index];
        if (reservation.owner == vehicle || std::any_of(
            reservation.waiting.begin(), reservation.waiting.end(),
            [&](const JunctionRequest& request) { return request.vehicle == vehicle; }
        )) {
            continue;
        }
        newRequests.push_back({
            *requestedJunctions[index],
            { vehicle, requestedArrivalSteps[index] }
        });
    }
    std::stable_sort(newRequests.begin(), newRequests.end(), [](const auto& first, const auto& second) {
        if (first.first != second.first) {
            return first.first < second.first;
        }
        return first.second.arrivalStep < second.second.arrivalStep;
    });
    for (const auto& [intersection, request] : newRequests) {
        uncontrolledJunctions[intersection].waiting.push_back(request);
    }

    for (auto& entry : uncontrolledJunctions) {
        const std::size_t intersection = entry.first;
        JunctionReservation& reservation = entry.second;
        if (!reservation.owner && !reservation.waiting.empty()) {
            // If the simulation starts with a car already in the junction,
            // let it finish before promoting the queued approach.
            const auto alreadyInside = std::find_if(
                reservation.waiting.begin(), reservation.waiting.end(),
                [&](const JunctionRequest& request) {
                    return world.findIntersectionContaining({ request.vehicle->getX(), request.vehicle->getY() }) == intersection;
                }
            );
            const auto selected = alreadyInside == reservation.waiting.end()
                ? reservation.waiting.begin()
                : alreadyInside;
            reservation.owner = selected->vehicle;
            reservation.ownerEntered = alreadyInside != reservation.waiting.end();
            reservation.waiting.erase(selected);
        }

        for (const JunctionRequest& request : reservation.waiting) {
            const auto vehicle = std::find(vehicles.begin(), vehicles.end(), request.vehicle);
            if (vehicle == vehicles.end()) {
                continue;
            }
            const std::size_t index = static_cast<std::size_t>(vehicle - vehicles.begin());
            if (modes[index] == MotionMode::Normal) {
                modes[index] = MotionMode::Braking;
                trajectories[index] = predictTrajectory(
                    *vehicles[index], manualControlFor(index), modes[index], dt
                );
            }
        }
    }

    const auto reservedJunctionYielder = [&] (
        std::size_t firstIndex,
        std::size_t secondIndex
    ) -> std::optional<std::size_t> {
        for (const auto& [intersection, reservation] : uncontrolledJunctions) {
            if (!reservation.owner) {
                continue;
            }

            const bool firstOwns = reservation.owner == vehicles[firstIndex];
            const bool secondOwns = reservation.owner == vehicles[secondIndex];
            const bool firstRequests = requestedJunctions[firstIndex] == intersection;
            const bool secondRequests = requestedJunctions[secondIndex] == intersection;
            if (firstOwns && secondRequests && !secondOwns) {
                return secondIndex;
            }
            if (secondOwns && firstRequests && !firstOwns) {
                return firstIndex;
            }
        }
        return std::nullopt;
    };

    // Resolve one predicted conflict at a time. Apply braking first; if that
    // cannot clear the conflict, escalate to a full stop.
    const std::size_t maxAdjustments = vehicles.size() * 2 + 1;
    for (std::size_t adjustment = 0; adjustment < maxAdjustments; ++adjustment) {
        bool changed = false;
        for (std::size_t first = 0; first < vehicles.size() && !changed; ++first) {
            for (std::size_t second = first + 1; second < vehicles.size(); ++second) {
                if (!trajectoriesCollide(
                    *vehicles[first], trajectories[first],
                    *vehicles[second], trajectories[second]
                )) {
                    continue;
                }

                // Vulnerable road users retain right of way. For car-to-car
                // junction conflicts, yield the car that will arrive later.
                const bool firstHasPriority = first >= priorityParticipantStart;
                const bool secondHasPriority = second >= priorityParticipantStart;
                const bool firstIsBicycle = first >= priorityParticipantStart &&
                    first < pedestrianPriorityStart;
                const bool secondIsBicycle = second >= priorityParticipantStart &&
                    second < pedestrianPriorityStart;
                const bool firstIsPedestrian = first >= pedestrianPriorityStart;
                const bool secondIsPedestrian = second >= pedestrianPriorityStart;
                const bool protectFirstInJunction = firstHasPriority && !secondHasPriority &&
                    priorityAgentInsideJunction[first];
                const bool protectSecondInJunction = secondHasPriority && !firstHasPriority &&
                    priorityAgentInsideJunction[second];
                const bool priorityAgentClearsJunction =
                    protectFirstInJunction || protectSecondInJunction;
                const std::optional<std::size_t> reservationYielder =
                    !firstHasPriority && !secondHasPriority
                    ? reservedJunctionYielder(
                        first, second
                    )
                    : std::nullopt;
                const std::optional<std::size_t> junctionYielder =
                    !firstHasPriority && !secondHasPriority
                    ? preferredJunctionYielder(
                        world,
                        *vehicles[first], trajectories[first],
                        *vehicles[second], trajectories[second],
                        first, second
                    )
                    : std::nullopt;
                std::size_t preferredYielder;
                if (firstIsBicycle && secondIsPedestrian) {
                    preferredYielder = second;
                }
                else if (secondIsBicycle && firstIsPedestrian) {
                    preferredYielder = first;
                }
                else if (firstHasPriority != secondHasPriority) {
                    preferredYielder = firstHasPriority ? second : first;
                }
                else {
                    preferredYielder = reservationYielder.value_or(junctionYielder.value_or(second));
                }
                const std::size_t alternateYielder = preferredYielder == first ? second : first;
                for (const std::size_t yielder : { preferredYielder, alternateYielder }) {
                    if (priorityAgentClearsJunction &&
                        yielder == (protectFirstInJunction ? first : second)) {
                        continue;
                    }

                    const MotionMode nextMode = moreRestrictiveMode(modes[yielder]);
                    if (nextMode == modes[yielder]) {
                        continue;
                    }

                    const Trajectory nextPath = predictTrajectory(
                        *vehicles[yielder], manualControlFor(yielder), nextMode, dt
                    );
                    const std::size_t other = yielder == first ? second : first;
                    if (!trajectoriesCollide(
                        *vehicles[yielder], nextPath,
                        *vehicles[other], trajectories[other]
                    )) {
                        modes[yielder] = nextMode;
                        trajectories[yielder] = nextPath;
                        changed = true;
                        break;
                    }
                }

                // Once a road user is in the junction, don't freeze it to
                // satisfy a car's prediction. Keep yielding pressure on the
                // car and let the road user clear the conflict area.
                if (!changed && !priorityAgentClearsJunction) {
                    const Trajectory stoppedFirst = predictTrajectory(
                        *vehicles[first], manualControlFor(first), MotionMode::Stopped, dt
                    );
                    const Trajectory stoppedSecond = predictTrajectory(
                        *vehicles[second], manualControlFor(second), MotionMode::Stopped, dt
                    );
                    if (!trajectoriesCollide(
                        *vehicles[first], stoppedFirst,
                        *vehicles[second], stoppedSecond
                    )) {
                        modes[first] = MotionMode::Stopped;
                        modes[second] = MotionMode::Stopped;
                        trajectories[first] = stoppedFirst;
                        trajectories[second] = stoppedSecond;
                        changed = true;
                    }
                }

                if (changed) {
                    break;
                }
            }
        }

        if (!changed) {
            break;
        }
    }

    for (std::size_t index = 0; index < vehicles.size(); ++index) {
        vehicles[index]->Update(
            dt,
            manualControlFor(index),
            speedLimitForMode(*vehicles[index], modes[index], dt)
        );
    }
}
