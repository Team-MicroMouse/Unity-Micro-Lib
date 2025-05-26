#ifndef ASTAR_H
#define ASTAR_H
#include "../../algorithms/algorithms.h"

class Astar : IPathfinder {
    int (*h)(v2i a, v2i b);

public:
    Astar();

    void Setup(Microsim::Robot robot, void* data) override;
    int Pathfind(Map map, RobotPosition robot_position, v2i target, v2i* path) override;

private:
    struct NodeData;
    static void insert_ordered(std::list<NodeData*>& list, NodeData* node_data);
    void check_position(
        std::list<NodeData*>& nodes_to_process,
        std::unordered_map<v2i, NodeData, v2iHasher>& node_datas,
        NodeData* current_node,
        Map map, v2i prev_pos, v2i dir, v2i target_pos);

public:
    struct NodeData {
        v2i position;
        v2i origin;
        int depth; // Should be replaced with f-cost, but this is just more convenient to use
        int cost_g;
        int cost_f;
    };
};

int ManhattenDistance(v2i node, v2i target);



#endif //ASTAR_H
