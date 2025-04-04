//
// Created by Jeroen on 3-4-2025.
//

#include "interface.h"
#include "../com/com.h"

int32_t Sensor::ReadValue() {
	return Microsim::Sensor_ReadValue(handle);
}

int8_t Motor::CurrentThrottle() {
	return Microsim::Motor_CurrentThrottle(handle);
}

void Motor::SetThrottle(int8_t throttle) {
	Microsim::Motor_SetThrottle(handle, throttle);
}
