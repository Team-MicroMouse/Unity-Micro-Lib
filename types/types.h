//
// Created by Jeroen on 15-4-2025.
//

#ifndef MATH_H
#define MATH_H
#include <cstdint>

struct v2i { int32_t x,y; };
struct v3i { int32_t x,y,z; };
struct Guid {
    uint64_t a, b;
    static Guid FromUint128_t(__uint128_t);
};

#endif //MATH_H
