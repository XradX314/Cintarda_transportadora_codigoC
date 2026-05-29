#ifndef TCRT5000_H_
#define TCRT5000_H_

#include <stdint.h>

typedef uint8_t (*ir_read_cb_t)(void);
typedef void (*ir_event_cb_t)(void);

typedef struct {
	ir_read_cb_t   read_pin;       /* lectura del pin digital */
	ir_event_cb_t  on_detected;    /* callback flanco positivo */
	ir_event_cb_t  on_released;    /* callback flanco negativo */
	uint8_t        stable_state;   /* último estado filtrado definitivo */
	uint8_t        debounce_cnt;   /* contador interno del filtro */
	
	/* Parámetro configurable dinámicamente */
	uint8_t        debounce_threshold; /* Muestras consecutivas requeridas */
} IR_Sensor_t;

/* Valor por defecto (como referencia o fallback seguro de 5ms @ 200µs) */
#define IR_DEFAULT_DEBOUNCE_THRESHOLD   25U

/* API */
void IR_Init(IR_Sensor_t *dev, ir_read_cb_t read, ir_event_cb_t on_detected, ir_event_cb_t on_released);
void IR_TuneParams(IR_Sensor_t *dev, uint8_t debounce_threshold);
void IR_Tick(IR_Sensor_t *dev);
uint8_t IR_IsActive(const IR_Sensor_t *dev);

#endif /* TCRT5000_H_ */