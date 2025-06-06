#include "Tawd.h"

#include "../../types/types.h"
#include "../../algorithms/algorithms.h"
#include "../../com/com.h"

using namespace Microsim;

 int is_wall_of_cell(v2i robot_position, v2f wall_hit_pos, v2f ray_offset) {
     const v2f robot_position_f = robot_position.toV2f();
     const v2i grid_pos = (robot_position_f / CELL_SIZE_F).roundToV2i();
     const v2f robot_dir = wall_hit_pos - (robot_position_f + ray_offset);

     const v2f dir_to_wall = (wall_hit_pos - grid_pos.toV2f() * 180).explode().normalize();
     const v2f dir_along_wall = dir_to_wall.rotated(90 * DEG2RAD);
     const float pos_along_wall = dir_along_wall.dot(robot_dir);
     const float max_hit_length = CELL_SIZE_F / 2 + dir_to_wall.dot((robot_position_f + ray_offset) - grid_pos.toV2f() * 180);
     const float hit_length = dir_to_wall.dot(robot_dir) - dir_to_wall.dot(ray_offset);

     UnityEngine::Logf("max_len", max_hit_length);
     UnityEngine::Logf("len", hit_length);
     UnityEngine::Logf("dpr", pos_along_wall);

     if (abs(pos_along_wall > CELL_SIZE_F * .3f)) {
         return  2;
     }

     return hit_length < max_hit_length;
}

void Tawd::Setup(Robot robot, void *data) {
    fwdSensor = robot.FindComponent(Guid(4764948050219179759u, 13563840741769542562u)).ToSensor_i32();
    leftSensor = robot.FindComponent(Guid(5421639628147193954u, 6046799433377730472u)).ToSensor_i32();
    rightSensor = robot.FindComponent(Guid(5124586844430470358u, 6050595459198776716u)).ToSensor_i32();
}

void Tawd::Process(Map map, RobotPosition robot_position) {
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
        int is_wall = is_wall_of_cell(robot_position.position, fwd_hit_pos, fwd_offset.rotated(static_cast<float>(robot_position.angle)));
        UnityEngine::Logi("fwd", is_wall);
        if (is_wall < 2) {
            cell->set_wall_in_dir(fwd_dir_abs, is_wall);
        }
    } else {
        cell->set_wall_in_dir(fwd_dir_abs, false);
    }

    if (lhs_value != -1) {
        UnityEngine::Debug::DrawLine2D((robot_position.position.toV2f() + lhs_offset.rotated(angle_rad)) / 1000, lhs_hit_pos / 1000, 1, Color::green());
        int is_wall = is_wall_of_cell(robot_position.position, lhs_hit_pos, lhs_offset.rotated(static_cast<float>(robot_position.angle)));
        UnityEngine::Logi("lhs", is_wall);
        if (is_wall < 2) {
            cell->set_wall_in_dir(lhs_dir_abs, is_wall);
        }
    } else {
        cell->set_wall_in_dir(lhs_dir_abs, false);
    }

    if (rhs_value != -1) {
        UnityEngine::Debug::DrawLine2D((robot_position.position.toV2f() + rhs_offset.rotated(angle_rad)) / 1000, rhs_hit_pos / 1000, 1, Color::blue());
        int is_wall = is_wall_of_cell(robot_position.position, rhs_hit_pos, rhs_offset.rotated(static_cast<float>(robot_position.angle)));
        UnityEngine::Logi("rhs", is_wall);
        if (is_wall < 2) {
            cell->set_wall_in_dir(rhs_dir_abs, is_wall);
        }
    } else {
        cell->set_wall_in_dir(rhs_dir_abs, false);
    }
}