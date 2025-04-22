
#include "algorithms.h"
#include "../com/com.h"
#include "../types/types.h"

constexpr __uint128_t operator""_uint128_t(const char* x)
{
    __uint128_t y = 0;
    for (int i = 2; x[i] != '\0'; ++i)
    {
        y *= 16ull;
        if ('0' <= x[i] && x[i] <= '9')
            y += x[i] - '0';
        else if ('A' <= x[i] && x[i] <= 'F')
            y += x[i] - 'A' + 10;
        else if ('a' <= x[i] && x[i] <= 'f')
            y += x[i] - 'a' + 10;
    }
    return y;
}

void TestObjectDetector::Setup(Microsim::Robot robot, void* data) {
    UnityEngine::Log("Setup TestObjectDetector");
    fwdSensor = robot.FindComponent(Guid { 4764948050219179759u, 13563840741769542562u }).ToSensor_i32();
}

void TestObjectDetector::Process(int* map, v2i mapSize) {
    UnityEngine::Logi("FWD", fwdSensor.ReadValue());
}