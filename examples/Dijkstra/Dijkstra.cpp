//
// Created by Jeroen on 20-6-2025.
//

#include "Dijkstra.h"

#include <execution>
#include <queue>

#include "../../com/com.h"

void add_optimized_nodes(std::unordered_map<v2i, std::list<GraphConnection>, v2iHasher>* map, std::queue<v2i>* nodes_to_visit, v2i from, v2i to) {
    if (!map->contains(to)) {
        nodes_to_visit->push(to);
    }

    int cost_g = static_cast<int>((from - to).length()) * Dijkstra::CELL_COST + Dijkstra::BASE_COST;
    map->at(from).push_back(GraphConnection(cost_g, from, to));
}

void Dijkstra::Setup(Microsim::Robot robot, void* data) {
}

int Dijkstra::Pathfind(Map map, RobotPosition position, v2i target, v2i* path) {
    v2i grid_pos = (position.position / CELL_SIZE_F).roundToV2i();

    std::unordered_map<v2i, std::list<GraphConnection>, v2iHasher> graph;
    std::queue<v2i> nodes_to_visit;

    nodes_to_visit.push(grid_pos);
    graph.insert({ grid_pos, std::list<GraphConnection>() });

    while (!nodes_to_visit.empty()) {
        v2i start_node = nodes_to_visit.front();
        nodes_to_visit.pop();

        map.get_cell(start_node)->set_wall_highlight(true);

        for (int i = 0; i < 4; i++) {
            v2i dir = v2i(i % 2 * i - 2 * (i % 2), (i+1) % 2 * (i+1) - 2 * ((i+1) % 2));
            v2i current_node = start_node;
            MapCell* current_cell = map.get_cell(current_node);

            // Debug::LogV2i("start node", current_node);
            // Debug::LogV2i("dir", dir);

            if (current_cell->is_wall_in_dir(dir)) {
                continue;
            }

            while (current_cell->is_discovered()) {
                v2i next_node = current_node + dir;
                MapCell* next_cell = map.get_cell(next_node);
                int wall_count = next_cell->wall_count();

                // Debug::LogV2i("next node", next_node);
                // Debug::Logi("wall count", wall_count);

                if (!graph.contains(next_node)) {
                    nodes_to_visit.push(next_node);
                    graph.insert({ next_node, std::list<GraphConnection>() });
                }

                int cost_g = static_cast<int>((start_node - next_node).length()) * CELL_COST + BASE_COST;
                graph.at(start_node).push_back(GraphConnection(cost_g, start_node, next_node));
                // Debug::LogV2i("Created connection to", next_node);

                if (next_cell->is_wall_in_dir(dir)) {
                    break;
                }

                current_node = next_node;
                current_cell = next_cell;
            }
        }
    }

    //  Prefer going straight from the first node
    const v2i dir = v2f::fromAngle(static_cast<int>(round(position.angle / 90.0)) * 90).roundToV2i();
    for (GraphConnection& connection : graph.at(grid_pos)) {
        connection.cost_g -= ((connection.end - connection.start).toV2f().normalize().roundToV2i() == dir) * BASE_COST;
    }

    struct NodeCostData {
        int cost_f;
        int depth;
        v2i parent;
    };

    std::unordered_map<v2i, NodeCostData, v2iHasher> node_costs;
    nodes_to_visit.push(grid_pos);
    node_costs.insert({ grid_pos, { 0, 0, grid_pos } });

    while (!nodes_to_visit.empty()) {
        v2i current_node = nodes_to_visit.front();
        NodeCostData current_node_cost = node_costs.at(current_node);
        nodes_to_visit.pop();

        if (!graph.contains(current_node)) {
            continue;
        }
        for (GraphConnection connection : graph.at(current_node)) {
            int new_cost_f = current_node_cost.cost_f + connection.cost_g;
            if (node_costs.contains(connection.end)) {
                if (new_cost_f < node_costs.at(connection.end).cost_f) {
                    node_costs.at(connection.end) = { new_cost_f, current_node_cost.depth + 1, current_node };
                }
            } else {
                node_costs.insert({ connection.end, { new_cost_f, current_node_cost.depth + 1, current_node } });
                nodes_to_visit.push(connection.end);
            }

        }
    }

    if (!node_costs.contains(target)) {
        return -1;
    }

    auto& [cost_f, depth, parent] = node_costs.at(target);
    v2i curr_node = target;
    for (int i = depth+1; i --> 0;) {
        path[i] = curr_node;
        curr_node = node_costs[curr_node].parent;
    }

    return depth+1;
}
