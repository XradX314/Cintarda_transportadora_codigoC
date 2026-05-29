#include "botones.h"

static _sBoton botones[MAX_BOTONES];
static uint8_t cantBotones = 0;
static uint8_t estadoAnterior[MAX_BOTONES];

void botonesInit(void) {
	cantBotones = 0;
}

// Registra un botón en un pin específico y configura automáticamente el pin como entrada con resistencia pull-up
// cb es una función que se llamará al detectar flanco ascendente.
void botonesRegister(volatile uint8_t *pin, volatile uint8_t *port, volatile uint8_t *ddr, uint8_t mask, BtnCallback cb) {
	if (cantBotones >= MAX_BOTONES) return;

	*ddr  &= ~mask; 
	*port |=  mask;  

	botones[cantBotones].pin  = pin;
	botones[cantBotones].port = port;
	botones[cantBotones].ddr  = ddr;
	botones[cantBotones].mask = mask;
	botones[cantBotones].cb   = cb;

	estadoAnterior[cantBotones] = *pin & mask;
	cantBotones++;
}

// Actualiza el estado de todos los botones registrados.
void botonesUpdate(void) {
	for (uint8_t i = 0; i < cantBotones; i++) {
		uint8_t actual  = *botones[i].pin & botones[i].mask;
		uint8_t cambio  = actual ^ estadoAnterior[i];
		uint8_t flanco  = cambio & actual;  
		
		if (flanco && botones[i].cb)
		botones[i].cb();
		
		estadoAnterior[i] = actual;
	}
}