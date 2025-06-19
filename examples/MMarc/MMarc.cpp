#include "MMarc.h"

#include "../../com/com.h"
#include "../Astar/Astar.h"
#include "../Floodfill/Floodfill.h"
#include "../ObjectDetection/Objectdetection.h"
#include "../Tawd/Tawd.h"

void MMarc::Setup(Microsim::Robot robot, void* data) {
    object_detector = reinterpret_cast<IObjectDetectorAlgorithm*>(new Tawd());
    motor_controller = reinterpret_cast<IMotorController*>(new SimulatorMotorController("DefaultMotorController"));
    position_tracker = reinterpret_cast<IPositionTracker*>(new SimulatorPositionTracker("DefaultPositionTracker"));
    astar = reinterpret_cast<IPathfinder*>(new Astar());
    floodfill = reinterpret_cast<IPathfinder*>(new Floodfill());

    map = Map {
        static_cast<MapCell*>(malloc(sizeof(MapCell) * MAZE_SIZE.x * MAZE_SIZE.y)),
        MAZE_SIZE
    };

    object_detector->Setup(robot, nullptr);
    motor_controller->Setup(robot, nullptr);
    position_tracker->Setup(robot, nullptr);
    astar->Setup(robot, nullptr);
    floodfill->Setup(robot, nullptr);

    path = static_cast<v2i*>(malloc(sizeof(v2i) * MAZE_SIZE.x * MAZE_SIZE.y));
    path_index = 0;
    path_size = 0;

    state = ResetMemory;

    motor_controller->SetGyroNull();
    motor_controller->SetRpm(40);
}

void MMarc::Loop(float dtf) {
    Debug::Log("-- begin --");

    position_tracker->Process(&robot_position);
    object_detector->Process(map, robot_position);
    Debug::LogV2i("Grid Pos", (robot_position.position / CELL_SIZE_F).roundToV2i());
    Debug::DrawRay3D(
        ((robot_position.position / CELL_SIZE_F).roundToV2i().toV2f() * CELL_SIZE_F / 1000).toFlatV3f(),
        v3f(0, 1, 0), 0, Color::blue());

    Debug::Logi("State:", state);
    Debug::Logi("Motor:", motor_controller->GetMoveState());

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
    Debug::Log("-- end --");
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

    v2i grid_pos = (robot_position.position / CELL_SIZE_F).roundToV2i();
    map.get_cell(grid_pos)->set_discovered(true);

    if (path_index >= path_size) {
        Debug::Log("Making new path");
        map.reset_highlights();
        path_size = floodfill->Pathfind(map, robot_position, v2i::zero(), path);
        Debug::DisplayMap(map);
        path_index = 1;
        if (path_size == -1) {
            return false;
        }
    }

    Debug::DisplayMap(map);

    if (path[path_index-1] != grid_pos) {
        Debug::Log("Positions did not match!");
        path_size = 0;
        path_index = 0;
        return true;
    }

    v2i target = path[path_index];
    Debug::LogV2i("Moving to new position ", target);
    Debug::Logi("Path Size", path_size);
    Debug::Logi("Path Index", path_index);
    motor_controller->MoveToGridPos(target, CELL_SIZE_F);
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
        map.reset_highlights();
        path_size = astar->Pathfind(map, robot_position, point, path);
        path_index = 1;
        if (path_size <= 0) {
            return false;
        }
    }

    v2i grid_pos = (robot_position.position / CELL_SIZE_F).roundToV2i();
    if (path[path_index-1] != grid_pos) {
        Debug::Log("Positions did not match!");
        path_size = 0;
        path_index = 0;
        return true;
    }

    v2i target = path[path_index];
    motor_controller->MoveToGridPos(target, CELL_SIZE_F);
    path_index++;
    return true;
}

void MMarc::reset_memory() {
    robot_position = RobotPosition();
    for (int i = 0; i < map.size.x * map.size.y; i++) {
        path[i] = v2i::zero();
        map.cells[i] = MapCell();
    }

    map.get_cell(v2i::zero())->set_discovered(true);
    map.get_cell(v2i::zero())->set_wall_north(false);
    map.get_cell(v2i::zero())->set_wall_east(false);
    map.get_cell(v2i::zero())->set_wall_south(true);
    map.get_cell(v2i::zero())->set_wall_west(true);
}
