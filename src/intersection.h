#pragma once
#include "raylib.h"
#include "lane.h"
#include <vector>
#include <unordered_map>

enum class TurnType {
    STRAIGHT,
    LEFT,
    RIGHT,
    INVALID
};

class Intersection {
    public:
        Intersection(Rectangle bounds);
        Rectangle getBounds() const;
        void Draw();
        void addLane(Lane* lane);
        std::vector<Lane*>& getConnectedLanes();
        void computeTransitions();
        const std::vector<std::pair<Lane*, TurnType>>& getNextLanes(Lane* incomingLane);        TurnType classifyTurn(Lane* incoming, Lane* outgoing);

    
    private:
        bool isValidTransition(Lane* incoming, Lane* outgoing);

    private:
        Rectangle bounds;
        std::vector<Lane*> connectedLanes;
        std::unordered_map<Lane*, std::vector<std::pair<Lane*, TurnType>>> transitions;        
};