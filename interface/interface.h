//
// Created by Jeroen on 3-4-2025.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <cstdint>

struct Object {
	int32_t handle;
};

struct IExtractableData {
};

struct IMotorController {
};

struct Sensor : Object {
	int32_t ReadValue();
};

struct Motor : Object {
	int8_t CurrentThrottle();
	void SetThrottle(int8_t throttle);
};

#endif //INTERFACE_H
