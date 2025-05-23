
#ifndef DIRECTION_H
#define DIRECTION_H

enum Direction {
    DIR_NORTH,
    DIR_SOUTH,
    DIR_EAST,
    DIR_WEST
};

struct RobotPosition {
    v2f position;
    Direction direction;
};

#endif //DIRECTION_H
