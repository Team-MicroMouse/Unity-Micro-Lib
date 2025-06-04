#include "Objectdetection.h"

#include "Direction.h"
#include "../../types/types.h"
#include "../../algorithms/algorithms.h"

using namespace Microsim;

#ifdef DEBUG_MODE
#include <iostream>
#endif

#define  WALL_THRESHOLD 300 // Afstand (in mm) die telt als de muur
#define CELL_WALL 1

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

void Objectdetection::Process(Map map, RobotPosition position) {
    // huidige positie op grid
    v2i grid_pos = (position.position / CELL_SIZE_F).roundToV2i();

    //sensorwaarden
    int front = fwdSensor.ReadValue();
    int left = leftSensor.ReadValue();
    int right = rightSensor.ReadValue();

    //richtingsvectoren bepalen afhankelijk van waar de robot naar kijkt
    v2i forwardDir, leftDir, rightDir;

    int normalized_angle = ((position.angle % 360) + 360) % 360;
    int direction = static_cast<int>(round(normalized_angle / 90.0));

    switch (direction) {
        case DIR_NORTH:
            forwardDir = v2i(0, 1);
            leftDir = v2i(-1, 0);
            rightDir = v2i(1, 0);
            break;
        case DIR_EAST:
            forwardDir = v2i(1, 0);
            leftDir = v2i(0, 1);
            rightDir = v2i(0, -1);
            break;
        case DIR_SOUTH:
            forwardDir = v2i(0, -1);
            leftDir = v2i(1, 0);
            rightDir = v2i(-1, 0);
            break;
        case DIR_WEST:
            forwardDir = v2i(-1, 0);
            leftDir = v2i(0, -1);
            rightDir = v2i(0, 1);
            break;
        default:
            forwardDir = v2i(0,0);
            leftDir = v2i(0, 0);
            rightDir = v2i(0, 0);
            break;


    }
    // muren tekenen in de map op basis van sensorinput
    MapCell* cell = map.get_cell(grid_pos);
    cell->set_wall_in_dir(forwardDir, front < WALL_THRESHOLD);
    cell->set_wall_in_dir(leftDir, left < WALL_THRESHOLD);
    cell->set_wall_in_dir(rightDir, right < WALL_THRESHOLD);
}

