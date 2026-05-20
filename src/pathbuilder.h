#pragma once
#include "path.h"
#include "lane.h"

struct ConnectionData {
    Path path;
    Vector2 entryTrim;
    Vector2 exitTrim;

};

class PathBuilder {
    public:
        static ConnectionData connectLanes(Lane& entryLane, Lane& exitLane);

};

