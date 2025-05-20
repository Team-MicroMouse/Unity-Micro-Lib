#include "Astar.h"

#include <set>


Astar::Astar() {
    h = ManhattenDistance;
}

void Astar::Setup(Microsim::Robot robot, void* data) {
}

int Astar::Pathfind(Map map, RobotPosition robot_position, v2i target, v2i* path) {

    std::list<NodeData*> nodes_to_process;
    std::unordered_map<v2i, NodeData> node_datas;

    node_datas.insert({ robot_position.position, NodeData(robot_position.position, robot_position.position, h(robot_position.position, target), h(robot_position.position, target)) });
    nodes_to_process.push_back(&node_datas[robot_position.position]);

    while (!nodes_to_process.empty()) {
        NodeData* node_data = nodes_to_process.front();
        nodes_to_process.remove(node_data);

        v2i pos = node_data->position;
        astar_check_position(nodes_to_process, node_datas, node_data, map, pos, v2i::up(), target);
    }

    return -1;
}

bool astar_check_position(
    std::list<Astar::NodeData>& nodes_to_process,
    std::unordered_map<v2i, Astar::NodeData>& node_datas,
    Astar::NodeData* node_data,
    Map map, v2i pos, v2i dir, v2i target) {

    if (map.get_cell(pos)->is_wall_in_dir(dir)) {
        return false;
    }

    v2i next_pos = pos + dir;
    MapCell* map_cell = map.get_cell(next_pos);
    if (!map_cell->is_discovered()) {
        return false;
    }

    if (node_datas.contains(next_pos)) {
        Astar::NodeData* node_data = node_datas.at(next_pos);
        int fCost =
    }
    map_cell->set_wall_highlight(true);
}

void astar_insert_ordered(std::list<Astar::NodeData*>& list, Astar::NodeData* node_data) {
    auto lBegin = list.begin();
    for (int i = 0; i < list.size(); i++) {
        if ((*lBegin)->fCost < node_data->fCost) {
            ++lBegin;
            continue;
        }

        list.insert(lBegin, node_data);
        break;
    }

    list.push_back(node_data);
}

int ManhattenDistance(v2i node, v2i target) {
    return std::abs(node.x - target.x) + std::abs(node.y - target.y);
}
