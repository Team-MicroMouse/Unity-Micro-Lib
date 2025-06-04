#include "Objectdetection.h"

#include "Direction.h"
#include "../../types/types.h"
#include "../../algorithms/algorithms.h"
#include "../../com/com.h"

using namespace Microsim;

#ifdef DEBUG_MODE
#include <iostream>
#endif

#define WALL_THRESHOLD 100 // Afstand (in mm) die telt als de muur

bool is_wall_of_cell(v2i grid_pos, v2f wall_hit_pos) {
    const v2i wall_grid_pos = (wall_hit_pos / CELL_SIZE_F).roundToV2i();
    return grid_pos == wall_grid_pos;
}

void Objectdetection::Setup(Microsim::Robot robot, void *data) {
    fwdSensor = robot.FindComponent(Guid(4764948050219179759u, 13563840741769542562u)).ToSensor_i32();
    leftSensor = robot.FindComponent(Guid(5421639628147193954u, 6046799433377730472u)).ToSensor_i32();
    rightSensor = robot.FindComponent(Guid(5124586844430470358u, 6050595459198776716u)).ToSensor_i32();
}

#ifdef DEBUG_MODE
void Objectdetection::LogSensors(int front, int left, int right) {
    std::cout << "Sensor readings - Front: " << front << ", Left: " << left << ", Right: " << right << std::endl;
}
#endif

void Objectdetection::Process(Map map, RobotPosition robot_position) {
    const v2i grid_pos = (robot_position.position / CELL_SIZE_F).roundToV2i();

    const v2f fwd_offset = v2f(0, 20);
    const v2f lhs_offset = v2f(-20, 20);
    const v2f rhs_offset = v2f(20, 20);

    const v2f fwd_dir = v2f(0,1);
    const v2f lhs_dir = v2f(-1,0);
    const v2f rhs_dir = v2f(1,0);

    const float fwd_value = static_cast<float>(fwdSensor.ReadValue());
    const float lhs_value = static_cast<float>(leftSensor.ReadValue());
    const float rhs_value = static_cast<float>(rightSensor.ReadValue());

    const float angle_rad = static_cast<float>(-robot_position.angle) * DEG2RAD;

    const v2f fwd_hit_pos = robot_position.position.toV2f() + (fwd_offset + fwd_dir * fwd_value).rotated(angle_rad);
    const v2f lhs_hit_pos = robot_position.position.toV2f() + (lhs_offset + lhs_dir * lhs_value).rotated(angle_rad);
    const v2f rhs_hit_pos = robot_position.position.toV2f() + (rhs_offset + rhs_dir * rhs_value).rotated(angle_rad);

    const v2i fwd_dir_abs = fwd_dir.rotated(angle_rad).explode().normalize().roundToV2i();
    const v2i lhs_dir_abs = lhs_dir.rotated(angle_rad).explode().normalize().roundToV2i();
    const v2i rhs_dir_abs = rhs_dir.rotated(angle_rad).explode().normalize().roundToV2i();

    MapCell* cell = map.get_cell(grid_pos);

    if (fwd_value != -1) {
        UnityEngine::Debug::DrawLine2D((robot_position.position.toV2f() + fwd_offset.rotated(angle_rad)) / 1000, fwd_hit_pos / 1000, 1, Color::red());
        cell->set_wall_in_dir(fwd_dir_abs, is_wall_of_cell(grid_pos, fwd_hit_pos));
    } else {
        cell->set_wall_in_dir(fwd_dir_abs, false);
    }

    if (lhs_value != -1) {
        UnityEngine::Debug::DrawLine2D((robot_position.position.toV2f() + lhs_offset.rotated(angle_rad)) / 1000, lhs_hit_pos / 1000, 1, Color::green());
        cell->set_wall_in_dir(lhs_dir_abs, is_wall_of_cell(grid_pos, lhs_hit_pos));
    } else {
        cell->set_wall_in_dir(lhs_dir_abs, false);
    }

    if (rhs_value != -1) {
        UnityEngine::Debug::DrawLine2D((robot_position.position.toV2f() + rhs_offset.rotated(angle_rad)) / 1000, rhs_hit_pos / 1000, 1, Color::blue());
        cell->set_wall_in_dir(rhs_dir_abs, is_wall_of_cell(grid_pos, rhs_hit_pos));
    } else {
        cell->set_wall_in_dir(rhs_dir_abs, false);
    }
}