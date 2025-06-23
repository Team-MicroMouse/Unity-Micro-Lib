//
// Created by Jeroen on 20-6-2025.
//

#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "../../algorithms/algorithms.h"


class Dijkstra : IPathfinder {
public:
    static constexpr int BASE_COST = 5;
    static constexpr int CELL_COST = 1;

    void Setup(Microsim::Robot robot, void* data) override;
    int Pathfind(Map map, RobotPosition position, v2i target, v2i* path) override;
};

struct GraphConnection {
    int cost_g;
    v2i start;
    v2i end;
};




#endif //DIJKSTRA_H
