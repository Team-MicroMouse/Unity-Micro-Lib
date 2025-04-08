//
// Created by Jeroen on 3-4-2025.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <cstdint>
#include "../com/com.h"

struct IExtractableData {
};

struct IMotorController {
};

struct Sensor : Microsim::Object {
	int32_t ReadValue();
};

struct Motor : Microsim::Object {
	int8_t CurrentThrottle();
	void SetThrottle(int8_t throttle);
};

#endif //INTERFACE_H
