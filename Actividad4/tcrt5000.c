#include "tcrt5000.h"

void IR_Init(IR_Sensor_t *dev, ir_read_cb_t read, ir_event_cb_t on_detected, ir_event_cb_t on_released) {
	dev->read_pin    = read;
	dev->on_detected = on_detected;
	dev->on_released = on_released;
	
	uint8_t init_read = (read != (void*)0) ? read() : 0U;
	dev->stable_state = init_read;
	dev->debounce_cnt = 0;
	
	/* Carga inicial con el valor por defecto seguro */
	dev->debounce_threshold = IR_DEFAULT_DEBOUNCE_THRESHOLD;
}

/* Nueva función para configurar el comportamiento desde el main */
void IR_TuneParams(IR_Sensor_t *dev, uint8_t debounce_threshold) {
	if (debounce_threshold > 0) {
		dev->debounce_threshold = debounce_threshold;
	}
}

void IR_Tick(IR_Sensor_t *dev) {
	if (dev->read_pin == (void*)0) return;

	uint8_t raw_sample = dev->read_pin();

	// Comparamos contra el estado estable usando el umbral dinámico de la estructura
	if (raw_sample != dev->stable_state) {
		dev->debounce_cnt++;
		
		if (dev->debounce_cnt >= dev->debounce_threshold) {
			dev->stable_state = raw_sample;
			dev->debounce_cnt = 0;
			
			if (dev->stable_state) {
				if (dev->on_detected) dev->on_detected();
				} else {
				if (dev->on_released) dev->on_released();
			}
		}
		} else {
		dev->debounce_cnt = 0;
	}
}

uint8_t IR_IsActive(const IR_Sensor_t *dev) {
	return dev->stable_state;
}