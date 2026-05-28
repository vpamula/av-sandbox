#include "route.h"

void Route::addLane(Lane* lane) {
    lanes.push_back(lane);
}

const std::vector<Lane*>& Route::getLanes() const {
    return lanes;
}