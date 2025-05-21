#include "Objectdetection.h"
#include "types.h"
#include "Direction.h"
#include "WallFollowerRobotController.h
"
using namespace Microsim;

#ifdef DEBUG_MODE
#include <iostream>
#endif

#define  WALL_THRESHOLD 300 // Afstand (in mm) die telt als de muur

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
    int front = fwdSensor.GetValue();
    int left = leftSensor.GetValue();
    int right = rightSensor.GetValue();

    //richtingsvectoren bepalen afhankelijk van waar de robot naar kijkt
    v2i forwardDir, leftDir, rightDir;

    switch (position.direction) {
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

    }
    // muren tekenen in de map op basis van sensorinput
    if (front <WALL_THRESHOLD) {
        map.SetCell(grid_pos + forwardDir, CELL_WALL);
    }
    if (left <WALL_THRESHOLD) {
        map.SetCell(grid_pos + leftDir, CELL_WALL);
    }
    if (right <WALL_THRESHOLD) {
        map.SetCell(grid_pos + rightDir, CELL_WALL);
    }
}

