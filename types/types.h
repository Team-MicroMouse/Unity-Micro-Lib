//
// Created by Jeroen on 15-4-2025.
//

#ifndef MATH_H
#define MATH_H
#include <cstdint>
#include <math.h>

const float PI = 3.14159265358979323846f;
const float DEG2RAD = PI / 180;
const float RAD2DEG = 180 / PI;

struct v2f;
struct v3f;

struct v2i {
    int32_t x,y;

    v2f toV2f() const;
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

    v3f toFlatV3f() const;
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
struct v3f
{
    float x,y,z;
    v3f operator/(double b) const;
};
struct Guid { uint64_t a, b; };

struct Color
{
    float r,g,b,a;
    static Color black();
    static Color white();
    static Color red();
    static Color green();
    static Color blue();
};

struct RobotPosition {
    v2i position;
    int angle;
};

#endif //MATH_H
