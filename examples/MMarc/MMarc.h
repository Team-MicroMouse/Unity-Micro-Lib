#ifndef CARL_H
#define CARL_H
#include "../../algorithms/algorithms.h"


class MMarc : IRobotController {
    enum RobotState { ResetMemory, Explore, Return, ResetPosition, WaitSpeedrun, Speedrun, WaitEndSpeedrun, ReturnAfterSpeedrun, ResetAfterSpeedrun, WaitForReset };

    static constexpr v2i TARGET_POS = v2i(5, 5);
    static constexpr v2i MAZE_SIZE = v2i(6, 6);

    IObjectDetectorAlgorithm* object_detector = nullptr;
    IMotorController* motor_controller = nullptr;
    IPositionTracker* position_tracker = nullptr;
    IPathfinder* astar = nullptr;
    IPathfinder* floodfill = nullptr;
    Map map = {};
    RobotPosition robot_position;
    RobotState state;

    v2i* path = nullptr;
    int path_size = 0;
    int path_index = 0;
    float timer = -1;

public:
    void Setup(Microsim::Robot robot, void* data) override;
    void Loop(float dtf) override;
    ~MMarc() override;

private:
    bool explore_loop();
    bool reset_angle_loop();
    bool wait_loop(float dtf);
    bool move_to_point_loop(v2i point);

    void reset_memory();
};



#endif //CARL_H
