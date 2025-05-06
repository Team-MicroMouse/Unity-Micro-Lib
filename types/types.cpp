#include "types.h"

#include <cmath>

#include "../com/com.h"

int v2i::lengthSq() const {
    return x*x + y*y;
}

float v2i::length() const {
    return std::sqrt(lengthSq());
}

v2i v2i::operator-(const v2i &b) const {
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
