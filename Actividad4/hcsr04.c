#include "hcsr04.h"

	void HCSR04_Init(HCSR04_t *dev, hcsr04_pin_write_cb_t set_trig, hcsr04_result_cb_t on_result) {
		dev->set_trigger   = set_trig;
		dev->on_result     = on_result;
		dev->captura_lista = 0;
		dev->esperando_eco = 0;
	
		dev->trig_pulse_us  = HCSR04_DEFAULT_TRIG_PULSE_US;
		dev->us_to_mm        = HCSR04_DEFAULT_US_TO_MM;

		if (dev->set_trigger) dev->set_trigger(0);
	}

	void HCSR04_TuneParams(HCSR04_t *dev, uint16_t trig_pulse_us, float us_to_mm) {
		if (trig_pulse_us > 0) dev->trig_pulse_us = trig_pulse_us;
		if (us_to_mm > 0.0f)   dev->us_to_mm = us_to_mm;
	}

	void HCSR04_Trigger(HCSR04_t *dev) {
		// Si ya está midiendo o está en medio del pulso, ignoramos
		if (dev->esperando_eco || dev->captura_lista || dev->trig_generando) return;

		// Le cargamos 2 ticks. Como tu ISR corre a 200 µs,
		// esto garantiza un pulso de entre 200 y 400 µs, sobradísimo para el sensor.
		dev->trig_generando = 2;

		if (dev->set_trigger) {
			dev->set_trigger(1); // Levantamos el pin y nos vamos inmediatamente
		}
	}

	// NUEVA FUNCIÓN: Se ejecuta asíncronamente en el Timer
	void HCSR04_TickISR(HCSR04_t *dev) {
		if (dev->trig_generando > 0) {
			dev->trig_generando--;
		
			// Cuando el conteo llega a 0, bajamos el pin y habilitamos la espera del eco
			if (dev->trig_generando == 0) {
				if (dev->set_trigger) {
					dev->set_trigger(0);
				}
				dev->captura_lista = 0;
				dev->esperando_eco = 1;
			}
		}
	}

	// El main.c invoca esto desde su interrupción de hardware
	void HCSR04_UpdateFromISR(HCSR04_t *dev, uint16_t current_ticks, uint8_t pin_state) {
		if (!dev->esperando_eco) return;

		if (pin_state) {
			// Pin en ALTO -> Flanco de subida
			dev->t_subida = current_ticks;
			} else {
			// Pin en BAJO -> Flanco de bajada
			dev->t_bajada = current_ticks;
			dev->captura_lista = 1;
		}
	}

	void HCSR04_EventHandler(HCSR04_t *dev) {
		if (dev->captura_lista) {
			uint16_t ticks = dev->t_bajada - dev->t_subida;
		
			// Conversión de ticks a tiempo real.
			// Asumiendo que el hardware que llama corre con ticks de 0.5us (Prescaler 8 a 16MHz):
			float echo_us = (float)ticks * 0.5f;
			float dist_mm = echo_us * dev->us_to_mm;

			dev->captura_lista = 0;
			dev->esperando_eco = 0;

			if (dev->on_result) dev->on_result(dist_mm);
		}
	}