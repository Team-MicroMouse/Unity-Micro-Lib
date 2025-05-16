#ifndef FLOODFILLSTACK_H
#define FLOODFILLSTACK_H
#include "../../algorithms/algorithms.h"


class FloodfillStack : IPathfinder {
public:
    void Setup(Microsim::Robot robot, void* data) override;
    int Pathfind(Map map, RobotPosition position, v2f target, v2i* path) override;
};



#endif //FLOODFILLSTACK_H
