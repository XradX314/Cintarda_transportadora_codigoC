#ifndef CLASIFICADOR_H_
#define CLASIFICADOR_H_

#include <stdint.h>

#define CLASIFICADOR_FIFO_CAPACITY   16U

/* Valores geométricos por defecto (en milímetros) */
#define CLASIFICADOR_DEFAULT_PISO_MM        180.0f
#define CLASIFICADOR_DEFAULT_CHICA_MM       120.0f  /* 180mm - 60mm de caja */
#define CLASIFICADOR_DEFAULT_MEDIANA_MM     100.0f  /* 180mm - 80mm de caja */
#define CLASIFICADOR_DEFAULT_GRANDE_MM      80.0f   /* 180mm - 100mm de caja */
#define CLASIFICADOR_DEFAULT_TOLERANCIA_MM  10.0f

/* Tiempos por defecto para la mecánica (en milisegundos) */
#define CLASIFICADOR_DEFAULT_DELAY_MS       1500U   /* Retardo post-detección pedido */
#define CLASIFICADOR_BRAZO_ACTUACION_MS     400U    /* Tiempo que el servo se queda afuera */

/* Tipos de cajas posibles */
typedef enum {
	CAJA_NINGUNA     = 0x00,
	CAJA_CHICA       = 0x01,
	CAJA_MEDIANA     = 0x02,
	CAJA_GRANDE      = 0x03,
	CAJA_DESCONOCIDA = 0x04  // <--- NUEVO: Caja física detectada, pero fuera de rango
} Clasificador_TipoCaja_t;

/* Máquina de estados principal del coordinador */
typedef enum {
	ESTADO_APAGADO = 0,
	ESTADO_ESPERA_CAJA,
	ESTADO_MIDIENDO_RAFAGA,
	ESTADO_CALIBRACION_MANUAL
} Clasificador_EstadoMaster_t;

/* Estado independiente de cada brazo eyector */
typedef enum {
	EYECTOR_IDLE = 0,
	EYECTOR_ESPERANDO_DELAY,
	EYECTOR_PATEANDO,
	EYECTOR_RETRAYENDO
} Clasificador_EstadoEyector_t;

/* Configuración de distancias para clasificación (Medidas desde el techo) */
typedef struct {
	float distancia_piso_mm;      /* Distancia de referencia sin caja */
	float umbral_chica_mm;         /* Distancia leída para caja chica */
	float umbral_mediana_mm;       /* Distancia leída para caja mediana */
	float umbral_grande_mm;        /* Distancia leída para caja grande */
	float tolerancia_mm;           /* Tolerancia (Default 10mm) */
} Clasificador_ConfigCajas_t;

/* Módulo Eyector (Mánager de cada par IR-Servo +su FIFO) */
typedef struct {
	Clasificador_TipoCaja_t caja_asignada;            /* Qué tipo de caja debe patear este brazo */
	uint32_t                delay_post_deteccion_ms;  /* Tiempo de retraso configurable por el usuario */
	
	/* Cola FIFO Estática */
	Clasificador_TipoCaja_t fifo_cola[CLASIFICADOR_FIFO_CAPACITY];
	uint8_t                 fifo_head;
	uint8_t                 fifo_tail;
	uint8_t                 fifo_count;

	/* Control de tiempo no bloqueante para el actuador físico */
	Clasificador_EstadoEyector_t estado;
	uint32_t                     cronometro_ms;
} Clasificador_Eyector_t;

/* Estructura Principal del Clasificador Cooperativo */
typedef struct {
	Clasificador_EstadoMaster_t estado_master;
	Clasificador_ConfigCajas_t  config_cajas;
	Clasificador_Eyector_t      eyectores[3]; /* Los 3 pares IR-Servo independientes */

	/* Acumulador para el promedio del ultrasonido en ráfaga */
	uint32_t                    suma_distancias_mm;
	uint32_t                    cantidad_muestras;
	uint8_t  calib_muestras_restantes;  // Contador de muestras (5 a 0)
	uint32_t calib_suma_mm;             // Acumulador numérico
	uint8_t  calib_comando_origen;      // Guarda el ID para saber a quién responder
	uint8_t  calib_timer_retrigger_ms; // <--- NUEVA: Temporizador de respiro acústico
} Clasificador_Ctrl_t;

/* API de Inicialización y Configuración */
void Clasificador_Init(Clasificador_Ctrl_t *ctx, float distancia_piso_mm);
void Clasificador_ConfigurarUmbrales(Clasificador_Ctrl_t *ctx, float chica, float mediana, float grande, float tolerancia);
void Clasificador_ConfigurarEyector(Clasificador_Ctrl_t *ctx, uint8_t indice_eyector, Clasificador_TipoCaja_t tipo_caja, uint32_t delay_ms);

/* Comandos de Control Externo */
void Clasificador_ComandoInicio(Clasificador_Ctrl_t *ctx);
void Clasificador_ComandoParada(Clasificador_Ctrl_t *ctx);

/* Puentes de Eventos (Se llaman desde los callbacks de los sensores del main) */
void Clasificador_InfrarrojoEntrada_Callback(Clasificador_Ctrl_t *ctx, uint8_t objeto_detectado);
void Clasificador_Ultrasonico_Callback(Clasificador_Ctrl_t *ctx, float distancia_mm);
void Clasificador_InfrarrojoEyector_Callback(Clasificador_Ctrl_t *ctx, uint8_t indice_eyector, uint8_t objeto_detectado);

/* Motores de procesamiento cooperativo (Llamados periódicos no bloqueantes) */
void Clasificador_Task_Brazos_ms(Clasificador_Ctrl_t *ctx, uint8_t periodo_ms);

/*Define la altura del sensor de manera manual*/
void Clasificador_ConfigurarPiso(Clasificador_Ctrl_t *ctx, float distancia_piso_mm);

void Clasificador_IniciarCalibracionManual(Clasificador_Ctrl_t *ctx, uint8_t cmd_origen);

#endif /* CLASIFICADOR_H_ */