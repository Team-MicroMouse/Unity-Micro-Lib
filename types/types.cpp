#include "types.h"

#include <cmath>

#include "../com/com.h"
v2i v2i::up() { return v2i(0, 1); }
v2i v2i::right() { return v2i(1, 0); }
v2i v2i::down() { return v2i(0, -1); }
v2i v2i::left() { return v2i(-1, 0); }

v2f v2i::toV2f() const
{
    return v2f(x,y);
}

int v2i::lengthSq() const {
    return x*x + y*y;
}

float v2i::length() const {
    return std::sqrt(lengthSq());
}

v2i v2i::explode() const {
    int xAbs = abs(x), yAbs = abs(y);
    return v2i(x * (xAbs > yAbs), y * (yAbs > xAbs));
}

v2i v2i::operator+(v2i b) const {
    return v2i(x + b.x, y + b.y);
}

v2i v2i::operator-(const v2i b) const {
    return v2i(x - b.x, y - b.y);
}

v2i v2i::operator*(int b) const {
    return v2i(x * b, y * b);
}

v2i v2i::operator/(const int b) const {
    return v2i(x / b, y / b);
}

v2f v2i::operator/(const float b) const {
    return v2f {
        x / b,
        y / b,
    };
}

bool v2i::operator==(v2i b) const {
    return x == b.x && y == b.y;
}

v2f v2f::fromAngle(const int angle) {
    const float rads = static_cast<float>(angle) * DEG2RAD;
    return v2f(sin(rads), -cos(rads));
}

v2f v2f::round() const {
    return v2f(std::round(x), std::round(y));
}

float v2f::lengthSq() const {
    return x*x + y*y;
}

float v2f::length() const {
    return sqrt(lengthSq());
}


v2i v2f::roundToV2i() const {
    return v2i(static_cast<int32_t>(std::round(x)), static_cast<int32_t>(std::round(y)));
}

v2f v2f::explode() const {
    float xAbs = abs(x), yAbs = abs(y);
    return v2f(x * (xAbs > yAbs), y * (yAbs > xAbs));
}

v2f v2f::normalize() const {
    const float l = length();
    if (l == 0) { return v2f(0, 0); }
    return v2f(x / l, y / l);
}

v2f v2f::operator+(const v2i & b) const {
    return v2f(x + b.x, y + b.y);
}

v2f v2f::operator+(const v2f & b) const {
    return v2f(x + b.x, y + b.y);
}

v2f v2f::operator-(const v2f & b) const {
    return v2f(x - b.x, y - b.y);
}

v2f v2f::operator*(float b) const {
    return v2f(x * b, y * b);
}

v2f v2f::operator/(float b) const {
    return v2f(x / b, y / b);
}

v2f v2f::rotated(float angleRad) const {
    return v2f(
        x * cos(angleRad) - y * sin(angleRad),
        x * sin(angleRad) + y * cos(angleRad)
    );
}

v3f v2f::toFlatV3f() const
{
    return v3f(x,0, y);
}

v3f v3f::operator/(double b) const
{
    return v3f(x / b, y / b, z / b);
}

Color Color::black()
{
    return Color(0,0,0,1);
}

Color Color::white()
{
    return Color(1,1,1,1);
}

Color Color::red()
{
    return Color(1,0,0,1);
}

Color Color::green()
{
    return Color(0,1,0,1);
}

Color Color::blue()
{
    return Color(0,0,1,1);
}

bool MapCell::is_discovered() const { return (value & 1 << 0) == 1 << 0; }
bool MapCell::is_wall_north() const { return (value & 1 << 1) == 1 << 1; }
bool MapCell::is_wall_east() const { return (value & 1 << 2) == 1 << 2;  }
bool MapCell::is_wall_south() const { return (value & 1 << 3) == 1 << 3; }
bool MapCell::is_wall_west() const { return (value & 1 << 4) == 1 << 4; }

bool MapCell::is_wall_in_dir(v2i dir) const {
    int xByte = (static_cast<int>(std::pow(((1 - (dir.x + 1) / 2) + 1) & 3, 2)) << 2) * (dir.x != 0);
    int yByte = (static_cast<int>(std::pow(((1 - (dir.y + 1) / 2) + 1) & 3, 2)) << 1) * (dir.y != 0);
    int byte = xByte + yByte;
    return (value & byte) == byte;
}

bool MapCell::is_wall_highlight() const { return (value & 1 << 5) == 1 << 5; }


void MapCell::set_discovered(bool value) {
    this->value = (this->value & ~(1 << 0)) | (value << 0);
}

void MapCell::set_wall_north(bool value) {
    this->value = (this->value & ~(1 << 1)) | (value << 1);
}

void MapCell::set_wall_east(bool value) {
    this->value = (this->value & ~(1 << 2)) | (value << 2);
}

void MapCell::set_wall_south(bool value) {
    this->value = (this->value & ~(1 << 3)) | (value << 3);
}

void MapCell::set_wall_west(bool value) {
    this->value = (this->value & ~(1 << 4)) | (value << 4);
}

void MapCell::set_wall_in_dir(v2i dir, bool value) {
    int xByte = (static_cast<int>(std::pow(((1 - (dir.x + 1) / 2) + 1) & 3, 2)) << 2) * (dir.x != 0);
    int yByte = (static_cast<int>(std::pow(((1 - (dir.y + 1) / 2) + 1) & 3, 2)) << 1) * (dir.y != 0);
    int byte = xByte + yByte;
    this->value = (this->value & ~byte) | (value * byte);
}

void MapCell::set_wall_highlight(bool value) {
    this->value = (this->value & ~(1 << 5)) | (value << 5);
}

void Map::SetCell(v2i position, int value) {
    if (is_in_bounds(position)) {
        get_cell(position)->value = value;
    }
}

bool Map::is_in_bounds(v2i position) const {
    return position.x >= 0 && position.y >= 0 && position.x < size.x && position.y < size.y;
}

MapCell* Map::get_cell(v2i position) const {
    return &cells[position.y * size.x + position.x];
}

std::size_t v2iHasher::operator()(const v2i& v2i) const {
    return static_cast<long>(v2i.x) << 32 + static_cast<long>(v2i.y);
}
