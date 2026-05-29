#ifndef HCSR04_H_
#define HCSR04_H_

#include <stdint.h>

typedef void (*hcsr04_pin_write_cb_t)(uint8_t state);
typedef void (*hcsr04_result_cb_t)(float distance_mm);

typedef struct {
	hcsr04_pin_write_cb_t  set_trigger;    /* Callback para escribir TRIG */
	hcsr04_result_cb_t     on_result;      /* Callback resultado en mm */
	
	volatile uint16_t      t_subida;       /* Marca de tiempo flanco subida */
	volatile uint16_t      t_bajada;       /* Marca de tiempo flanco bajada */
	volatile uint8_t       captura_lista;  /* Flag de medición completada */
	volatile uint8_t       esperando_eco;  /* Flag para control de timeout */

	// NUEVO: Flag/Contador para generar el pulso no bloqueante
	volatile uint8_t       trig_generando;

	/* Parámetros dinámicos configurables */
	uint16_t               trig_pulse_us;
	float                  us_to_mm;
} HCSR04_t;

#define HCSR04_DEFAULT_TRIG_PULSE_US    12U
#define HCSR04_DEFAULT_US_TO_MM         0.1717f

/* API de la Librería */
void HCSR04_Init(HCSR04_t *dev, hcsr04_pin_write_cb_t set_trig, hcsr04_result_cb_t on_result);
void HCSR04_TuneParams(HCSR04_t *dev, uint16_t trig_pulse_us, float us_to_mm);
void HCSR04_Trigger(HCSR04_t *dev);
void HCSR04_EventHandler(HCSR04_t *dev);
/* Agregá este prototipo nuevo en la API de la librería */
void HCSR04_TickISR(HCSR04_t *dev);
/* Esta función es el puente: el main la llama dentro de la ISR pasándole el tiempo actual */
void HCSR04_UpdateFromISR(HCSR04_t *dev, uint16_t current_ticks, uint8_t pin_state);

#endif /* HCSR04_H_ */