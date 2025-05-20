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

    int cost_g = h(robot_position.position, target);
    node_datas.insert({ robot_position.position, NodeData(robot_position.position, robot_position.position, 0, cost_g, cost_g) });
    nodes_to_process.push_back(&node_datas[robot_position.position]);

    while (!nodes_to_process.empty()) {
        NodeData* node_data = nodes_to_process.front();

        if (node_data->position == target) {
            break;
        }

        nodes_to_process.remove(node_data);

        v2i pos = node_data->position;
        check_position(nodes_to_process, node_datas, node_data, map, pos, v2i::up(), target);
        check_position(nodes_to_process, node_datas, node_data, map, pos, v2i::right(), target);
        check_position(nodes_to_process, node_datas, node_data, map, pos, v2i::down(), target);
        check_position(nodes_to_process, node_datas, node_data, map, pos, v2i::left(), target);
    }

    if (nodes_to_process.empty()) {
        return -1;
    }

    NodeData* final_node = nodes_to_process.front();
    v2i curr_pos = final_node->position;
    int depth = final_node->depth+1;
    for (int i = depth; i --> 0;) {
        path[i] = curr_pos;
        curr_pos = node_datas[curr_pos].origin;
    }
    return depth;
}

void Astar::check_position(
    std::list<NodeData*>& nodes_to_process,
    std::unordered_map<v2i, NodeData>& node_datas,
    NodeData* current_node,
    Map map, v2i prev_pos, v2i dir, v2i target_pos) {

    if (map.get_cell(prev_pos)->is_wall_in_dir(dir)) {
        return;
    }

    v2i curr_pos = prev_pos + dir;
    MapCell* map_cell = map.get_cell(curr_pos);
    if (!map_cell->is_discovered()) {
        return;
    }

    if (node_datas.contains(curr_pos)) {
        NodeData& node_data = node_datas.at(curr_pos);
        const int cost_f = node_data.cost_h;
        const int new_cost_f = current_node->cost_h + node_data.cost_g;
        if (cost_f > new_cost_f) {
            node_data.cost_h = new_cost_f;
            node_data.position = current_node->position;
            node_data.depth = current_node->depth+1;
        }

        return;
    }

    map_cell->set_wall_highlight(true);
    const int cost_g = h(curr_pos, target_pos);
    auto node_data = NodeData(curr_pos, prev_pos, current_node->depth+1, cost_g, current_node->cost_h + cost_g);
    node_datas.insert({ curr_pos, node_data });
    insert_ordered(nodes_to_process, &node_datas[curr_pos]);
}

void Astar::insert_ordered(std::list<NodeData*>& list, NodeData* node_data) {
    auto lBegin = list.begin();
    for (int i = 0; i < list.size(); i++) {
        if ((*lBegin)->cost_g < node_data->cost_g) {
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
