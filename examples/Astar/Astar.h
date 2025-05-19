#ifndef ASTAR_H
#define ASTAR_H
#include "../../algorithms/algorithms.h"


class Astar : IPathfinder {
public:
    void Setup(Microsim::Robot robot, void* data) override;
    int Pathfind(Map map, RobotPosition position, v2f target, v2i* path) override;
};



#endif //ASTAR_H
