#ifndef MODO_CIEGO_H_
#define MODO_CIEGO_H_

#include <stdint.h>
#include "clasificador.h"

#define CIEGO_MAX_CAJAS 8U

/* Estados seguros del Servomotor en Modo Ciego */
typedef enum {
	CIEGO_ESTADO_VIAJANDO = 0,
	CIEGO_ESTADO_PATEANDO,
	CIEGO_ESTADO_RETRAYENDO
} Ciego_EstadoBrazo_t;

/* Estructura de una Caja Virtual (Simulación Cinemática) */
typedef struct {
	uint8_t  activa;
	Clasificador_TipoCaja_t tipo;
	uint8_t  brazo_asignado; // 0, 1, 2, o 255 si es CAJA_DESCONOCIDA
	
	// Temporizadores decrecientes en milisegundos para simular topología
	int32_t t_s1_ms;
	int32_t t_s2_ms;
	int32_t t_s3_ms;
	int32_t t_impacto_ms;
	int32_t t_fin_ms;

	// Banderas para emitir el evento del Tracker una sola vez
	uint8_t paso_s1;
	uint8_t paso_s2;
	uint8_t paso_s3;

	// Control seguro del impacto físico
	Ciego_EstadoBrazo_t estado_servo;
	uint32_t cronometro_servo_ms;
} Ciego_CajaVirtual_t;

/* Estructura de Configuración Geométrica Dinámica */
typedef struct {
	float longitud_caja_default_mm;
	float distancia_servo_mm[3];
	float offset_golpe_mm;
} Ciego_Config_t;

/* Controlador Principal del Modo Ciego */
typedef struct {
	uint8_t             activo;
	Ciego_Config_t      config;
	Ciego_CajaVirtual_t cajas_virtuales[CIEGO_MAX_CAJAS]; // Reemplaza las FIFOs
	uint32_t            t_subida_ir0_us;
	Clasificador_TipoCaja_t caja_en_medicion;
} ModoCiego_Ctrl_t;

/* API Pública de la Biblioteca */
void Ciego_Init(ModoCiego_Ctrl_t *ctx);
void Ciego_ConfigurarGeometria(ModoCiego_Ctrl_t *ctx, float long_caja, float d0, float d1, float d2, float offset_golpe);
void Ciego_ComandoActivar(ModoCiego_Ctrl_t *ctx, uint8_t activar);

/* Eventos de Entrada */
void Ciego_InfrarrojoEntrada_Callback(ModoCiego_Ctrl_t *ctx, uint8_t objeto_detectado, uint32_t ahora_us);
void Ciego_AsignarTipoCajaActual(ModoCiego_Ctrl_t *ctx, Clasificador_TipoCaja_t tipo);

/* Tarea de tiempo real para los brazos (Llamar en On2Ms) */
void Ciego_Task_Brazos_ms(ModoCiego_Ctrl_t *ctx, uint8_t periodo_ms);

#endif /* MODO_CIEGO_H_ */