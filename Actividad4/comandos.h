#ifndef COMANDOS_H_
#define COMANDOS_H_

#include <stdint.h>
#include "clasificador.h"

/* Definición de Códigos de Comando (API de la HMI) */
#define CMD_START_SISTEMA   0x01
#define CMD_STOP_SISTEMA    0x02
#define CMD_SET_DELAYS      0x10
#define CMD_SET_UMBRALES    0x11
#define CMD_SET_PISO        0x12
#define CMD_ACK             0x06
#define CMD_MEDIR_CALIBRACION 0x13
#define CMD_START_CIEGO         0x03
#define CMD_SET_GEOMETRIA_CIEGA 0x14

/**
 * @brief Inicializa el módulo de comandos y lo vincula al clasificador y al protocolo.
 * @param clasificador_ctx Puntero a la estructura de control principal.
 */
void Comandos_Init(Clasificador_Ctrl_t *clasificador_ctx);

#endif /* COMANDOS_H_ */