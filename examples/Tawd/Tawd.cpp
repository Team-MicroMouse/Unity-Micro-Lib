#include "Tawd.h"

#include "../../types/types.h"
#include "../../algorithms/algorithms.h"
#include "../../com/com.h"

using namespace Microsim;

void process_wall_hit(float sensor_value, v2f sensor_offset, v2f sensor_dir, RobotPosition robot_position, Map map) {
    const float angle_rad = static_cast<float>(-robot_position.angle) * DEG2RAD;

    const v2i grid_pos = (robot_position.position.toV2f() / CELL_SIZE_F).roundToV2i();
    const v2f hit_pos = robot_position.position.toV2f() + (sensor_offset + sensor_dir * sensor_value).rotated(angle_rad);

    Debug::DrawRay3D(
        (robot_position.position.toV2f() + sensor_offset.rotated(angle_rad)).toFlatV3f() / 1000 + v3f(0,0.1,0),
        (sensor_dir.rotated(angle_rad) * sensor_value).toFlatV3f() / 1000 + v3f(0,0.1,0),
        0, Color::green());


    if (sensor_value == -1) {
        v2f sensor_pos = robot_position.position.toV2f() + sensor_offset.rotated(angle_rad);
        v2f sensor_dir_rotated = sensor_dir.rotated(angle_rad);
        v2f sensor_local_pos = sensor_pos - grid_pos.toV2f() * CELL_SIZE_F;
        v2f dir_to_wall = sensor_dir_rotated.explode().normalize();
        v2f dir_along_wall = v2f(dir_to_wall.y, -dir_to_wall.x);

        float len_to_wall = CELL_SIZE_F / 2 - dir_to_wall.dot(sensor_local_pos);
        float len_across_wall = dir_to_wall.signedAngle(sensor_dir_rotated);
        v2f hit_pos = sensor_pos + dir_to_wall * len_to_wall + dir_along_wall * len_across_wall;

        Debug::DrawLine2D(sensor_pos / 1000, hit_pos / 1000, 0, Color::green());
        Debug::DrawRay3D(hit_pos.toFlatV3f() / 1000 + v3f(0,0.2f,0), v3f(0,1,0), 0, Color::red());
        const v2i halfs_grid_pos = (hit_pos / (CELL_SIZE_F / 2)).roundToV2i() + v2i::one();

        const v2i idx = halfs_grid_pos % 2;
        const v2i dir = v2i(idx.y, idx.x);
        const v2i cell_1 = halfs_grid_pos / 2;
        const v2i cell_2 = halfs_grid_pos / 2 - dir;

        if (idx.x == idx.y) {
            return;
        }

        if (map.is_in_bounds(cell_1)) {
            map.get_cell(cell_1)->set_wall_in_dir(-dir_to_wall.roundToV2i().abs(), false);
        }
        if (map.is_in_bounds(cell_2)) {
            map.get_cell(cell_2)->set_wall_in_dir(dir_to_wall.roundToV2i().abs(), false);
        }
        return;
    }


    const v2i halfs_grid_pos = (hit_pos / (CELL_SIZE_F / 2)).roundToV2i() + v2i::one();
    Debug::DrawLine2D((robot_position.position.toV2f() + sensor_offset.rotated(angle_rad)) / 1000, hit_pos / 1000, 0, Color::red());

    const v2i idx = halfs_grid_pos % 2;
    if (idx.x == idx.y) {
        return;
    }

    const v2i dir = v2i(idx.y, idx.x);
    const v2i cell_1 = halfs_grid_pos / 2;
    const v2i cell_2 = halfs_grid_pos / 2 - dir;

    if (map.is_in_bounds(cell_1)) {
        map.get_cell(cell_1)->set_wall_in_dir(-dir.abs(), true);
    }
    if (map.is_in_bounds(cell_2)) {
        map.get_cell(cell_2)->set_wall_in_dir(dir.abs(), true);
    }
}

void Tawd::Setup(Robot robot, void *data) {
    fwdSensor = robot.FindComponent(Guid(4764948050219179759u, 13563840741769542562u)).ToSensor_i32();
    leftSensor = robot.FindComponent(Guid(5421639628147193954u, 6046799433377730472u)).ToSensor_i32();
    rightSensor = robot.FindComponent(Guid(5124586844430470358u, 6050595459198776716u)).ToSensor_i32();
}

void Tawd::Process(Map map, RobotPosition robot_position) {
    const v2f fwd_offset = v2f(0, 67.5);
    const v2f lhs_offset = v2f(-25.8, 58.3);
    const v2f rhs_offset = v2f(25.8, 58.3);

    const v2f fwd_dir = v2f(0,1);
    const v2f lhs_dir = v2f(-1,0);
    const v2f rhs_dir = v2f(1,0);

    process_wall_hit(fwdSensor.ReadValue(), fwd_offset, fwd_dir, robot_position, map);
    process_wall_hit(leftSensor.ReadValue(), lhs_offset, lhs_dir, robot_position, map);
    process_wall_hit(rightSensor.ReadValue(), rhs_offset, rhs_dir, robot_position, map);
}