#ifndef SG90_H
#define SG90_H

#include <stdint.h>

/* Constantes calibradas para el SG90 real (Evita sobre-recorrido y jitter) */
#define SG90_PERIOD_US        20000U   /* Período estándar de 20 ms (50 Hz) [cite: 27, 28] */
#define SG90_PULSE_MIN_US      500U   /* 0° (Límite seguro de la hoja de datos)  */
#define SG90_PULSE_MAX_US      2400U   /* 180° (Límite seguro de la hoja de datos)  */
#define SG90_PULSE_NEUTRAL_US  1450U   /* 90° - Centro perfecto  */

typedef void (*sg90_pin_write_cb_t)(uint8_t state);

typedef struct {
	sg90_pin_write_cb_t  set_pin;      /* Callback HAL para escribir pin */
	uint16_t             pulse_ticks;  /* Almacena el pulso directamente en ticks de hardware */
} SG90_t;

/* API */
void SG90_Init(SG90_t *dev, sg90_pin_write_cb_t set_pin);
void SG90_SetAngle(SG90_t *dev, uint8_t angle_deg);
uint16_t SG90_GetTicks(const SG90_t *dev);

#endif /* SG90_H */