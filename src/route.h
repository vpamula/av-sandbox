#pragma once
#include "lane.h"
#include <vector>

class Route {
public:
    void addLane(Lane* lane);
    const std::vector<Lane*>& getLanes() const;

private:
    std::vector<Lane*> lanes;
};