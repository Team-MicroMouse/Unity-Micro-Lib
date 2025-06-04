#ifndef FLOODFILL_H
#define FLOODFILL_H
#include "../../algorithms/algorithms.h"


class Floodfill : IPathfinder {
public:
    void Setup(Microsim::Robot robot, void* data) override;
    int Pathfind(Map map, RobotPosition position, v2i target, v2i* path) override;
};



#endif //FLOODFILL_H
