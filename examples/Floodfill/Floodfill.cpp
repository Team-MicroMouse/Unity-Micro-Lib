#include "Floodfill.h"
#include <queue>

void Floodfill::Setup(Microsim::Robot robot, void* data) { }

int Floodfill::Pathfind(Map map, RobotPosition position, v2f target, v2i* path) {
    struct CellData {
        v2i parent;
        int depth;
    };


    v2i grid_pos = (position.position / CELL_SIZE_F).roundToV2i();

    std::unordered_map<v2i, CellData, v2iHasher> cell_datas;
    std::queue<v2i> cells_to_process;

    cells_to_process.push(grid_pos);
    cell_datas.insert({ grid_pos, CellData(grid_pos, 0) });
    map.get_cell(grid_pos)->set_wall_highlight(true);

    while (!cells_to_process.empty()) {
        v2i current_cell = cells_to_process.front();
        MapCell* cell = map.get_cell(current_cell);
        CellData cell_data = cell_datas[current_cell];

        if (!cell->is_discovered()) {
            break;
        }

        cells_to_process.pop();

        v2i pos = current_cell + v2i::up();
        if (!cell->is_wall_north() && !cell_datas.contains(pos)) {
            cells_to_process.push(pos);
            cell_datas.insert({ pos, CellData(current_cell, cell_data.depth+1) });
            map.get_cell(pos)->set_wall_highlight(true);
        }

        pos = current_cell + v2i::right();
        if (!cell->is_wall_east() && !cell_datas.contains(pos)) {
            cells_to_process.push(pos);
            cell_datas.insert({ pos, CellData(current_cell, cell_data.depth+1) });
            map.get_cell(pos)->set_wall_highlight(true);
        }

        pos = current_cell + v2i::down();
        if (!cell->is_wall_south() && !cell_datas.contains(pos)) {
            cells_to_process.push(pos);
            cell_datas.insert({ pos, CellData(current_cell, cell_data.depth+1) });
            map.get_cell(pos)->set_wall_highlight(true);
        }

        pos = current_cell + v2i::left();
        if (!cell->is_wall_west() && !cell_datas.contains(pos)) {
            cells_to_process.push(pos);
            cell_datas.insert({ pos, CellData(current_cell, cell_data.depth+1) });
            map.get_cell(pos)->set_wall_highlight(true);
        }
    }

    if (cells_to_process.empty()) {
        return -1;
    }

    v2i current_pos = cells_to_process.front();
    int depth = cell_datas[current_pos].depth+1;
    for (int i = depth; i --> 0;) {
        path[i] = current_pos;
        current_pos = cell_datas[current_pos].parent;
    }

    return depth;
}
