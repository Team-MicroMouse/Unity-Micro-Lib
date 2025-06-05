#include "MMarc.h"

#include "../Astar/Astar.h"
#include "../Floodfill/Floodfill.h"
#include "../ObjectDetection/Objectdetection.h"

void MMarc::Setup(Microsim::Robot robot, void* data) {
    object_detector = new Objectdetection();
    motor_controller = reinterpret_cast<IMotorController*>(new SimulatorMotorController("DefaultMotorController"));
    position_tracker = reinterpret_cast<IPositionTracker*>(new SimulatorPositionTracker("DefaultPositionTracker"));
    astar = reinterpret_cast<IPathfinder*>(new Astar());
    floodfill = reinterpret_cast<IPathfinder*>(new Floodfill());

    map = Map {
        static_cast<MapCell*>(malloc(sizeof(MapCell) * MAZE_SIZE.x * MAZE_SIZE.y)),
        MAZE_SIZE
    };

    object_detector->Setup(robot, nullptr);
    position_tracker->Setup(robot, nullptr);
    position_tracker->Setup(robot, nullptr);
    astar->Setup(robot, nullptr);
    floodfill->Setup(robot, nullptr);

    path = static_cast<v2i*>(malloc(sizeof(v2i) * MAZE_SIZE.x * MAZE_SIZE.y));
    path_index = 0;
    path_size = 0;

    state = ResetMemory;
}

void MMarc::Loop(float dtf) {
    position_tracker->Process(&robot_position);
    object_detector->Process(map, robot_position);

    switch (state) {
    case ResetMemory:
        reset_memory();
        state = Explore;
        break;
    case Explore:
        if (explore_loop()) {
            break;
        }
        state = Return;
        break;
    case Return:
        if (move_to_point_loop(v2i::zero())) {
            break;
        }
        state = ResetPosition;
        break;
    case ResetPosition:
        if (reset_angle_loop()) {
            break;
        }
        state = WaitSpeedrun;
        break;
    case WaitSpeedrun:
        if (timer == -1) {
            timer = 3;
        }
        if (wait_loop(dtf)) {
            break;
        }
        state = Speedrun;
        break;
    case Speedrun:
        if (move_to_point_loop(TARGET_POS)) {
            break;
        }
        state = WaitEndSpeedrun;
        break;
    case WaitEndSpeedrun:
        if (timer == -1) {
            timer = 3;
        }
        if (wait_loop(dtf)) {
            break;
        }
        state = ReturnAfterSpeedrun;
        break;
    case ReturnAfterSpeedrun:
        if (move_to_point_loop(v2i::zero())) {
            break;
        }
        state = ResetAfterSpeedrun;
        break;
    case ResetAfterSpeedrun:
        if (reset_angle_loop()) {
            break;
        }
        state = WaitForReset;
        break;
    case WaitForReset:
        if (timer == -1) {
            timer = 3;
        }
        if (wait_loop(dtf)) {
            break;
        }
        state = ResetMemory;
        break;
    }

    motor_controller->UpdateMovement(dtf, robot_position);
}

MMarc::~MMarc() {
    delete object_detector;
    delete motor_controller;
    delete astar;
    delete floodfill;
    free(map.cells);
    free(path);
}

bool MMarc::explore_loop() {
    if (motor_controller->GetMoveState() != IMotorController::Idle) {
        return true;
    }

    if (path_index >= path_size) {
        path_size = floodfill->Pathfind(map, robot_position, v2i::zero(), path);
        path_index = 1;
        if (path_size == -1) {
            return false;
        }
    }

    v2i target = path[path_index];
    // TODO: Send move command
    path_index++;
    return true;
}

bool MMarc::reset_angle_loop() {
    if (motor_controller->GetMoveState() != IMotorController::Idle) {
        return true;
    }

    if (abs((0 - robot_position.angle + 180) % 360 - 180) < 3) {
        return false;
    }

    motor_controller->RotateToAngle(0);
    return true;
}

bool MMarc::wait_loop(float dtf) {
    if (timer > 0) {
        timer -= dtf;
        return true;
    }
    timer = -1;
    return false;
}

bool MMarc::move_to_point_loop(v2i point) {
    if (motor_controller->GetMoveState() != IMotorController::Idle) {
        return true;
    }

    if (path_index >= path_size) {
        path_size = astar->Pathfind(map, robot_position, point, path);
        path_index = 1;
        if (path_size <= 0) {
            return false;
        }
    }

    v2i target = path[path_index];
    // TODO: Send move command
    path_index++;
    return true;
}

void MMarc::reset_memory() {
    robot_position = RobotPosition();
    for (int i = 0; i < map.size.x * map.size.y; i++) {
        path[i] = v2i::zero();
        map.cells[i] = MapCell();
    }
}
