//
// Created by Jeroen on 15-4-2025.
//

#ifndef MATH_H
#define MATH_H
#include <cstdint>
#include <math.h>

const float DEG2RAD = M_PI / 180;
const float RAD2DEG = 180 / M_PI;

struct v2f;
struct v2i {
    int32_t x,y;

    int lengthSq() const;
    float length() const;

    v2i operator-(const v2i & b) const;
    v2i operator*(int b) const;
    v2i operator/(int b) const;
    v2f operator/(float b) const;
};

struct v2f {
    static v2f fromAngle(int angle);

    float x,y;

    v2f round() const;
    float lengthSq() const;
    float length() const;
    v2i roundToV2i() const;
    v2f explode() const;
    v2f normalize() const;

    bool operator==(const v2f & b) const = default;
    v2f operator+(const v2i & b) const;
    v2f operator+(const v2f & b) const;
    v2f operator-(const v2f & b) const;
    v2f operator*(float b) const;
    v2f operator/(float b) const;
};

struct v3i { int32_t x,y,z; };
struct Guid { uint64_t a, b; };

struct RobotPosition {
    v2i position;
    int angle;
};

#endif //MATH_H
