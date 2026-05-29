#include "sg90.h"

void SG90_Init(SG90_t *dev, sg90_pin_write_cb_t set_pin) {
	dev->set_pin = set_pin;
	if (dev->set_pin) dev->set_pin(0);
	SG90_SetAngle(dev, 90); // Posición neutra inicial de seguridad
}

void SG90_SetAngle(SG90_t *dev, uint8_t angle_deg) {
	if (angle_deg > 180U) angle_deg = 180U;
	
	// Mapeo lineal entre 1000us (0°) y 2000us (180°)
	uint32_t pulse_us = SG90_PULSE_MIN_US + (((uint32_t)angle_deg * (SG90_PULSE_MAX_US - SG90_PULSE_MIN_US)) / 180U);
	
	// Conversión a ticks de hardware (Timer 1 con prescaler 8 -> 1 us = 2 ticks)
	dev->pulse_ticks = (uint16_t)(pulse_us * 2U);
}

uint16_t SG90_GetTicks(const SG90_t *dev) {
	return dev->pulse_ticks;
}