#ifndef ASTAR_H
#define ASTAR_H
#include "../../algorithms/algorithms.h"

class Astar : IPathfinder {
    int (*h)(v2i a, v2i b);

public:
    Astar();

    void Setup(Microsim::Robot robot, void* data) override;
    int Pathfind(Map map, RobotPosition robot_position, v2i target, v2i* path) override;

    struct NodeData {
        v2i position;
        v2i origin;
        int gCost;
        int fCost;

        int operator<(const NodeData& b) const { return fCost < b.fCost; }
        int operator>(const NodeData& b) const { return fCost > b.fCost; }
    };
};

int ManhattenDistance(v2i node, v2i target);



#endif //ASTAR_H
