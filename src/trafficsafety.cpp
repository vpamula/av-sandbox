#include "trafficsafety.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {
constexpr float PREDICTION_HORIZON_SECONDS = 2.0f;
constexpr float PREDICTION_STEP_SECONDS = 0.02f;
constexpr float MAX_SAFETY_DECELERATION = 700.0f;
constexpr float COLLISION_CLEARANCE = 3.0f;
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

    for (const Vector2 axis : axes) {
        const float centerDistance = std::abs(Vector2DotProduct(centerOffset, axis));
        const float firstRadius =
            first.getLength() * 0.5f * std::abs(Vector2DotProduct(firstForward, axis)) +
            first.getWidth() * 0.5f * std::abs(Vector2DotProduct(firstSide, axis)) +
            COLLISION_CLEARANCE;
        const float secondRadius =
            second.getLength() * 0.5f * std::abs(Vector2DotProduct(secondForward, axis)) +
            second.getWidth() * 0.5f * std::abs(Vector2DotProduct(secondSide, axis)) +
            COLLISION_CLEARANCE;
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
    const std::vector<Vehicle*>& vehicles,
    bool manualEgoControl,
    float dt,
    std::size_t priorityParticipantStart
) const {
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

    // Resolve one predicted conflict at a time. NPCs yield first; if braking
    // alone cannot clear the conflict, the safety cap escalates to a full stop.
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

                // Vulnerable road users receive right of way over cars. Car-to-
                // car conflicts retain the existing rule: the later participant yields first.
                const bool firstHasPriority = first >= priorityParticipantStart;
                const bool secondHasPriority = second >= priorityParticipantStart;
                const std::size_t preferredYielder = firstHasPriority != secondHasPriority
                    ? (firstHasPriority ? second : first)
                    : second;
                const std::size_t alternateYielder = preferredYielder == first ? second : first;
                for (const std::size_t yielder : { preferredYielder, alternateYielder }) {
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

                if (!changed) {
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
