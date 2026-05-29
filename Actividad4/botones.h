#ifndef BOTONES_H
#define BOTONES_H

#include <avr/io.h>

#define MAX_BOTONES 8

// Tipo de dato para la función callback que se ejecuta al detectar que se apreto un pulsador
typedef void (*BtnCallback)(void);

// Estructura interna que almacena la configuración de cada botón registrado
typedef struct {
	volatile uint8_t *pin;   
	volatile uint8_t *port; 
	volatile uint8_t *ddr;   
	uint8_t           mask; 
	BtnCallback       cb;   
} _sBoton;

void botonesInit(void);
void botonesRegister(volatile uint8_t *pin, volatile uint8_t *port, volatile uint8_t *ddr, uint8_t mask, BtnCallback cb);
void botonesUpdate(void);

#endif /* BOTONES_H_ */