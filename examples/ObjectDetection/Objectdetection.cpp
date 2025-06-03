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

    const v2f fwd_offset = v2f(0, 10);
    const v2f lhs_offset = v2f(-10, 10);
    const v2f rhs_offset = v2f(10, 10);

    const v2f fwd_dir = v2f(0,1);
    const v2f lhs_dir = v2f(-1,1);
    const v2f rhs_dir = v2f(1,1);

    const float fwd_angle = robot_position.angle;
    const float lhs_angle = robot_position.angle - 90;
    const float rhs_angle = robot_position.angle + 90;

    const int fwd_value = fwdSensor.ReadValue();
    const int lhs_value = leftSensor.ReadValue();
    const int rhs_value = rightSensor.ReadValue();

    const v2f fwd_hit_pos = robot_position.position.toV2f() + (fwd_offset + fwd_dir * fwd_value).rotated(fwd_angle * DEG2RAD);
    const v2f lhs_hit_pos = robot_position.position.toV2f() + (lhs_offset + lhs_dir * lhs_value).rotated(lhs_angle * DEG2RAD);
    const v2f rhs_hit_pos = robot_position.position.toV2f() + (rhs_offset + rhs_dir * rhs_value).rotated(rhs_angle * DEG2RAD);

    UnityEngine::Debug::DrawLine2D(robot_position.position.toV2f() + fwd_offset.rotated(fwd_angle), fwd_hit_pos, 1, Color::red());
    UnityEngine::Debug::DrawLine2D(robot_position.position.toV2f() + lhs_offset.rotated(lhs_angle), lhs_hit_pos, 1, Color::green());
    UnityEngine::Debug::DrawLine2D(robot_position.position.toV2f() + rhs_offset.rotated(rhs_angle), rhs_hit_pos, 1, Color::blue());
}