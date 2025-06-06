//
// Created by Jeroen on 15-4-2025.
//

#ifndef MATH_H
#define MATH_H
#define CELL_WALL 1
#include <cstdint>
#include <functional>
#include <math.h>

const float PI = 3.14159265358979323846f;
const float DEG2RAD = PI / 180;
const float RAD2DEG = 180 / PI;
constexpr uint32_t CELL_SIZE = 180;
constexpr float CELL_SIZE_F = static_cast<float>(CELL_SIZE);

struct v2f;
struct v3f;

struct v2i {
    int32_t x,y;

    static v2i up();
    static v2i right();
    static v2i down();
    static v2i left();
    static v2i zero();
    static v2i one();

    v2f toV2f() const;

    int lengthSq() const;
    float length() const;
    v2i explode() const;

    v2i operator+(v2i b) const;
    v2i operator-(v2i b) const;
    v2i operator*(int b) const;
    v2i operator/(int b) const;
    v2f operator/(float b) const;
    bool operator==(v2i b) const;
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
    float dot(v2f rhs) const;

    bool operator==(const v2f & b) const = default;
    v2f operator+(const v2i & b) const;
    v2f operator+(const v2f & b) const;
    v2f operator-(const v2f & b) const;
    v2f operator*(float b) const;
    v2f operator/(float b) const;
    v2f rotated(float angle_rad) const;
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

struct MapCell {
    uint8_t value;
    bool is_discovered() const;
    bool is_wall_north() const;
    bool is_wall_east() const;
    bool is_wall_south() const;
    bool is_wall_west() const;
    bool is_wall_in_dir(v2i dir) const;
    bool is_wall_highlight() const;

    void set_discovered(bool value);
    void set_wall_north(bool value);
    void set_wall_east(bool value);
    void set_wall_south(bool value);
    void set_wall_west(bool value);
    void set_wall_in_dir(v2i dir, bool value);
    void set_wall_highlight(bool value);
};

struct Map {
    MapCell* cells;
    v2i size;

    bool is_in_bounds(v2i position) const;
    MapCell* get_cell(v2i position) const;

    void SetCell(v2i position, int value);
};

struct v2iHasher {
    std::size_t operator()(const v2i& v2i) const;
};

#endif //MATH_H