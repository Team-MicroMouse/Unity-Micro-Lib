//
// Created by Jeroen on 3-4-2025.
//

#include "microsim.h"
#include "../com/com.h"

int32_t Microsim::Sensor_i32::ReadValue() const {
	return Sensor_i32_ReadValue(handle);
}

float Microsim::Sensor_f32::ReadValue() const {
	return Sensor_f32_ReadValue(handle);
}

v3i Microsim::Sensor_v3i::ReadValue() const {
	return Sensor_v3i_ReadValue(handle);
}

int8_t Microsim::Motor::CurrentThrottle() const {
	return Motor_CurrentThrottle(handle);
}

void Microsim::Motor::SetThrottle(int8_t throttle) const {
	Motor_SetThrottle(handle, throttle);
}

Microsim::IFindableComponent Microsim::Robot::FindComponent(Guid guid) const {
	return { Robot_FindComponent(handle, guid) };
}

Microsim::Sensor_i32 Microsim::IFindableComponent::ToSensor_i32() const {
	return Sensor_i32 { handle };
}

Microsim::Sensor_f32 Microsim::IFindableComponent::ToSensor_f32() const {
	return Sensor_f32 { handle };
}

Microsim::Sensor_v3i Microsim::IFindableComponent::ToSensor_v3i() const {
	return Sensor_v3i { handle };
}

Microsim::Motor Microsim::IFindableComponent::ToMotor() const {
	return Motor { handle };
}
