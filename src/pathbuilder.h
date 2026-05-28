#pragma once
#include "path.h"
#include "lane.h"
#include "route.h"

class PathBuilder {
    public:
        static Path connectLanes(Lane& entryLane, Lane& exitLane);
        static Path buildRoute(const Route& route);
};

