#include "clasificador.h"
#include "sg90.h"
#include "hcsr04.h"    // <--- NUEVO: Resuelve 'HCSR04_t' y 'HCSR04_Trigger'
#include "protocolo.h" // <--- NUEVO: Resuelve 'Encode'
#include "modo_ciego.h"
#include "tracker.h"


extern ModoCiego_Ctrl_t mi_modo_ciego;

// ========================================================================
// PROTOTIPOS DE FUNCIONES PRIVADAS (Agregá estas líneas acá arriba)
// ========================================================================
static Clasificador_TipoCaja_t Clasificador_IdentificarCaja(Clasificador_Ctrl_t *ctx, float promedio_mm);
static void Clasificador_PushFIFO(Clasificador_Ctrl_t *ctx, Clasificador_TipoCaja_t caja_detectada);
static void Clasificador_ProcesarResultadoMedicion(Clasificador_Ctrl_t *ctx, float promedio_mm);
static Clasificador_TipoCaja_t Clasificador_PeekFIFO(Clasificador_Eyector_t *b);
static void Clasificador_PopFIFO(Clasificador_Eyector_t *b);

// Declaramos los servos del main como externos para que la librería pueda interactuar con ellos
extern SG90_t servo1;
extern SG90_t servo2;
extern SG90_t servo3;

void Clasificador_Init(Clasificador_Ctrl_t *ctx, float distancia_piso_mm) {
	if (ctx == (void*)0) return;

	// 1. Inicializar la Máquina de Estados General en APAGADO (Espera comando USART)
	ctx->estado_master = ESTADO_APAGADO;
	ctx->suma_distancias_mm = 0UL;
	ctx->cantidad_muestras = 0UL;

	// 2. Cargar la configuración geométrica
	// Si el usuario pasa un valor válido (>0) para el piso lo usamos, sino el default
	ctx->config_cajas.distancia_piso_mm = (distancia_piso_mm > 0.0f) ? distancia_piso_mm : CLASIFICADOR_DEFAULT_PISO_MM;
	
	// Seteamos las distancias de las cajas usando los defaults calibrados desde el techo
	ctx->config_cajas.umbral_chica_mm   = CLASIFICADOR_DEFAULT_CHICA_MM;
	ctx->config_cajas.umbral_mediana_mm = CLASIFICADOR_DEFAULT_MEDIANA_MM;
	ctx->config_cajas.umbral_grande_mm  = CLASIFICADOR_DEFAULT_GRANDE_MM;
	ctx->config_cajas.tolerancia_mm     = CLASIFICADOR_DEFAULT_TOLERANCIA_MM;

	// 3. Inicializar y mapear los 3 módulos Eyectores independientes
	for (uint8_t i = 0; i < 3; i++) {
		ctx->eyectores[i].delay_post_deteccion_ms = CLASIFICADOR_DEFAULT_DELAY_MS;
		ctx->eyectores[i].estado = EYECTOR_IDLE;
		ctx->eyectores[i].cronometro_ms = 0UL;
		
		// Inicializar la cola FIFO de cada brazo (vacía)
		ctx->eyectores[i].fifo_head = 0U;
		ctx->eyectores[i].fifo_tail = 0U;
		ctx->eyectores[i].fifo_count = 0U;
		
		for (uint8_t j = 0; j < CLASIFICADOR_FIFO_CAPACITY; j++) {
			ctx->eyectores[i].fifo_cola[j] = CAJA_NINGUNA;
		}
	}

	// Mapeo por defecto de la aplicación: Brazo 0 -> Chica, Brazo 1 -> Mediana, Brazo 2 -> Grande
	ctx->eyectores[0].caja_asignada = CAJA_CHICA;
	ctx->eyectores[1].caja_asignada = CAJA_MEDIANA;
	ctx->eyectores[2].caja_asignada = CAJA_GRANDE;
}

/* ========================================================================
   COMANDOS DE CONTROL EXTERNO (USART)
   ======================================================================== */

void Clasificador_ComandoInicio(Clasificador_Ctrl_t *ctx) {
	if (ctx == (void*)0) return;
	
	// Si el sistema estaba apagado, se pone en modo de escucha activo
	if (ctx->estado_master == ESTADO_APAGADO) {
		ctx->estado_master = ESTADO_ESPERA_CAJA;
		ctx->suma_distancias_mm = 0UL;
		ctx->cantidad_muestras = 0UL;
	}
}

void Clasificador_ComandoParada(Clasificador_Ctrl_t *ctx) {
	if (ctx == (void*)0) return;
	
	// Fuerza el apagado completo del coordinador
	ctx->estado_master = ESTADO_APAGADO;
	// 2. Limpiamos las colas FIFO y reseteamos el estado de los 3 brazos
	for (uint8_t i = 0; i < 3; i++) {
		ctx->eyectores[i].fifo_head = 0U;
		ctx->eyectores[i].fifo_tail = 0U;
		ctx->eyectores[i].fifo_count = 0U;
		ctx->eyectores[i].estado = EYECTOR_IDLE; // Cancelamos cualquier golpe en curso
	}

	// 3. Limpiamos la memoria geográfica (Topología)
	Tracker_Init();
	// 4. Limpiamos las simulaciones en curso del Modo Ciego
	for (uint8_t i = 0; i < CIEGO_MAX_CAJAS; i++) {
		mi_modo_ciego.cajas_virtuales[i].activa = 0;
	}
}

/* ========================================================================
   PUENTES DE EVENTOS: INFRARROJO DE ENTRADA (ir_s0)
   ======================================================================== */

void Clasificador_InfrarrojoEntrada_Callback(Clasificador_Ctrl_t *ctx, uint8_t objeto_detectado) {
	if (ctx == (void*)0) return;

	// 1. CANDADO GENERAL: Si ambos modos están apagados, no hacemos nada.
	if (ctx->estado_master == ESTADO_APAGADO && !mi_modo_ciego.activo) return;

	if (objeto_detectado) {
		// Flanco de subida: Entramos a medir la ráfaga (sin importar el modo)
		if (ctx->estado_master == ESTADO_ESPERA_CAJA || ctx->estado_master == ESTADO_APAGADO) {
			ctx->estado_master = ESTADO_MIDIENDO_RAFAGA;
			ctx->suma_distancias_mm = 0UL;
			ctx->cantidad_muestras = 0UL;
		}
		} else {
		// Flanco de bajada: Terminó de pasar la caja, calculamos.
		if (ctx->estado_master == ESTADO_MIDIENDO_RAFAGA) {
			
			if (ctx->cantidad_muestras > 0UL) {
				float promedio_mm = (float)ctx->suma_distancias_mm / (float)ctx->cantidad_muestras;
				
				uint8_t payload_prom[1] = { (uint8_t)promedio_mm };
				Encode(0x5F, payload_prom, 1U);
				
				Clasificador_ProcesarResultadoMedicion(ctx, promedio_mm);
			}
			
			// 2. RESTAURACIÓN: Si estamos en modo ciego, el clasificador normal
			// vuelve a dormirse. Si no, vuelve a esperar caja.
			if (mi_modo_ciego.activo) {
				ctx->estado_master = ESTADO_APAGADO;
				} else {
				ctx->estado_master = ESTADO_ESPERA_CAJA;
			}
		}
	}
}

/* ========================================================================
   PUENTES DE EVENTOS: CAPTURA DE DISTANCIA DEL ULTRASONICO
   ======================================================================== */

void Clasificador_Ultrasonico_Callback(Clasificador_Ctrl_t *ctx, float distancia_mm) {
	// 1. Filtrado universal: Si es un error (-1.0f), salimos.
	// La lógica de abortar la calibración por error ya la maneja on_resultado en main.c
	if (ctx == (void*)0 || distancia_mm < 0.0f) return;

	// =========================================================
	// BLOQUE 1: MODO PRODUCCIÓN NORMAL (Cajas en la cinta)
	// =========================================================
	if (ctx->estado_master == ESTADO_MIDIENDO_RAFAGA) {
		ctx->suma_distancias_mm += (uint32_t)distancia_mm;
		ctx->cantidad_muestras++;
		// NUEVO: Agregamos el respiro acústico para la próxima medición de la caja
		ctx->calib_timer_retrigger_ms = 60U;
	}

	// =========================================================
	// BLOQUE 2: MODO CALIBRACIÓN MANUAL (Totalmente independiente)
	// =========================================================
	else if (ctx->estado_master == ESTADO_CALIBRACION_MANUAL) {
		
		if (ctx->calib_muestras_restantes > 0U) {
			// Sumamos a la variable EXCLUSIVA de calibración
			ctx->calib_suma_mm += (uint32_t)distancia_mm;
			ctx->calib_muestras_restantes--;

			if (ctx->calib_muestras_restantes > 0U) {
				ctx->calib_timer_retrigger_ms = 60U; // Respiro acústico
			} else {
				// ¡Llegamos a las 5 muestras!
				uint8_t promedio_final_mm = (uint8_t)(ctx->calib_suma_mm / 5UL);
				
				uint8_t payload_res[1];
				payload_res[0] = promedio_final_mm;
				Encode(ctx->calib_comando_origen, payload_res, 1U);

				ctx->estado_master = ESTADO_APAGADO;
			}
		}
	}
}

/* ========================================================================
   FUNCIONES INTERNAS PRIVADAS (STATIC)
   ======================================================================== */

/**
 * @brief Evalúa geométricamente el promedio y decide el tamaño de la caja.
 * Recuerda: Menor distancia al techo -> Caja más alta (Grande).
 */
static Clasificador_TipoCaja_t Clasificador_IdentificarCaja(Clasificador_Ctrl_t *ctx, float promedio_mm) {
	Clasificador_ConfigCajas_t *cfg = &ctx->config_cajas;

	// Validación preliminar: Si el promedio está muy cerca del piso (con su tolerancia),
	// significa que fue un falso disparo o el haz se cortó sin una caja real en la base.
	if (promedio_mm >= (cfg->distancia_piso_mm - cfg->tolerancia_mm)) {
		return CAJA_NINGUNA;
	}

	// Evaluamos la Caja Grande (10cm de altura -> aprox 80mm al sensor)
	if (promedio_mm >= (cfg->umbral_grande_mm - cfg->tolerancia_mm) &&
		promedio_mm <= (cfg->umbral_grande_mm + cfg->tolerancia_mm)) {
		return CAJA_GRANDE;
	}

	// Evaluamos la Caja Mediana (8cm de altura -> aprox 100mm al sensor)
	if (promedio_mm >= (cfg->umbral_mediana_mm - cfg->tolerancia_mm) &&
		promedio_mm <= (cfg->umbral_mediana_mm + cfg->tolerancia_mm)) {
		return CAJA_MEDIANA;
	}

	// Evaluamos la Caja Chica (6cm de altura -> aprox 120mm al sensor)
	if (promedio_mm >= (cfg->umbral_chica_mm - cfg->tolerancia_mm) &&
		promedio_mm <= (cfg->umbral_chica_mm + cfg->tolerancia_mm)) {
		return CAJA_CHICA;
	}

	// Si cayó fuera de los tres rangos configurados con su tolerancia
	return CAJA_DESCONOCIDA; // <--- ANTES DECÍA CAJA_NINGUNA
}

/**
 * @brief Inserta el tipo de caja detectado en la cola del eyector correspondiente.
 */
static void Clasificador_PushFIFO(Clasificador_Ctrl_t *ctx, Clasificador_TipoCaja_t caja_detectada) {
	if (caja_detectada == CAJA_NINGUNA) return;

	// FIX: Metemos la caja en la cola de LOS 3 BRAZOS
	// para que la memoria de todos mantenga el mismo orden físico que la cinta.
	for (uint8_t i = 0; i < 3; i++) {
		Clasificador_Eyector_t *b = &ctx->eyectores[i];

		if (b->fifo_count < CLASIFICADOR_FIFO_CAPACITY) {
			b->fifo_cola[b->fifo_head] = caja_detectada;
			b->fifo_head = (b->fifo_head + 1) & (CLASIFICADOR_FIFO_CAPACITY - 1);
			b->fifo_count++;
		}
	}
}

/**
 * @brief Función coordinadora intermedia que conecta la ráfaga con la decisión
 */
static void Clasificador_ProcesarResultadoMedicion(Clasificador_Ctrl_t *ctx, float promedio_mm) {
	Clasificador_TipoCaja_t tipo = Clasificador_IdentificarCaja(ctx, promedio_mm);
	
	if (tipo != CAJA_NINGUNA) {
		
		if (mi_modo_ciego.activo) {
			Ciego_AsignarTipoCajaActual(&mi_modo_ciego, tipo);
			} else {
			Tracker_NuevaCaja(tipo);
			
			// FIX: Ahora SI metemos TODAS las cajas físicas en la FIFO,
			// incluso las CAJA_DESCONOCIDA, para que hagan bulto y sincronicen a los sensores
			Clasificador_PushFIFO(ctx, tipo);
		}
	}
}

/**
 * @brief Mira el elemento en el frente de la cola de un eyector sin removerlo.
 */
static Clasificador_TipoCaja_t Clasificador_PeekFIFO(Clasificador_Eyector_t *b) {
	if (b->fifo_count == 0U) {
		return CAJA_NINGUNA;
	}
	return b->fifo_cola[b->fifo_tail];
}

/**
 * @brief Remueve el elemento del frente de la cola (Pop).
 */
static void Clasificador_PopFIFO(Clasificador_Eyector_t *b) {
	if (b->fifo_count == 0U) return;
	
	b->fifo_cola[b->fifo_tail] = CAJA_NINGUNA;
	b->fifo_tail = (b->fifo_tail + 1) & (CLASIFICADOR_FIFO_CAPACITY - 1);
	b->fifo_count--;
}

/**
 * @brief Callback que se ejecuta cuando cambian los sensores IR de los brazos.
 */
void Clasificador_InfrarrojoEyector_Callback(Clasificador_Ctrl_t *ctx, uint8_t indice_eyector, uint8_t objeto_detectado) {
	if (ctx == (void*)0 || ctx->estado_master == ESTADO_APAGADO) return;
	if (indice_eyector >= 3) return;

	Clasificador_Eyector_t *b = &ctx->eyectores[indice_eyector];

	if (objeto_detectado) {
		// Flanco de subida: Una caja física acaba de cortar el haz
		if (b->estado == EYECTOR_IDLE) {
			
			Clasificador_TipoCaja_t proxima_caja_en_fifo = Clasificador_PeekFIFO(b);
			
			if (proxima_caja_en_fifo == b->caja_asignada) {
				// Coincide: Arrancamos el retraso programable y pateamos
				b->estado = EYECTOR_ESPERANDO_DELAY;
				b->cronometro_ms = 0UL;
			}
			else if (proxima_caja_en_fifo != CAJA_NINGUNA) {
				// FIX: ¡La caja física llegó, pero es de otro tamaño o es la mala!
				// La sacamos INMEDIATAMENTE de la memoria de este brazo (Pop)
				// para que la deje pasar y podamos esperar tranquilos a la siguiente.
				Clasificador_PopFIFO(b);
			}
		}
		} else {
		// Flanco de bajada: La caja terminó de pasar
		Tracker_TransicionSensor(indice_eyector + 1);
	}
}

void Clasificador_Task_Brazos_ms(Clasificador_Ctrl_t *ctx, uint8_t periodo_ms) {
	
	if (ctx == (void*)0) return;

	// --- Motor Unificado de Re-disparo (Watchdog + Respiro Acústico) ---
	if (ctx->estado_master == ESTADO_CALIBRACION_MANUAL || ctx->estado_master == ESTADO_MIDIENDO_RAFAGA) {
		if (ctx->calib_timer_retrigger_ms > periodo_ms) {
			ctx->calib_timer_retrigger_ms -= periodo_ms;
			} else {
			extern HCSR04_t g_hcsr04;
		
			if (!g_hcsr04.esperando_eco) {
				HCSR04_Trigger(&g_hcsr04);
				ctx->calib_timer_retrigger_ms = 100U; // Watchdog 100ms
				} else {
				ctx->calib_timer_retrigger_ms = 10U;  // Gracia 10ms
			}
		}
	
		// Si es calibración, no procesamos los servos, nos vamos.
		if (ctx->estado_master == ESTADO_CALIBRACION_MANUAL) return;
	}

	// Si el sistema está apagado, nos vamos sin mover servos
	if (ctx->estado_master == ESTADO_APAGADO) return;

	// Vector auxiliar para poder indexar y apuntar a tus 3 estructuras de servos globales
	SG90_t *servos[3] = {&servo1, &servo2, &servo3};

	// Recorremos los 3 eyectores independientes
	for (uint8_t i = 0; i < 3; i++) {
		Clasificador_Eyector_t *b = &ctx->eyectores[i];

		switch (b->estado) {
			case EYECTOR_IDLE:
			// No hay nada que hacer, el brazo está en reposo (0 grados)
			break;

			case EYECTOR_ESPERANDO_DELAY:
			// Incrementamos el tiempo transcurrido no bloqueante
			b->cronometro_ms += periodo_ms;
			
			// Si se cumplió el retraso programado por el usuario (ej: 1500 ms)
			if (b->cronometro_ms >= b->delay_post_deteccion_ms) {
				// ACCIÓN: Extendemos el servo a 90° para impactar la caja
				SG90_SetAngle(servos[i], 0U);
				

				
				// Transición al estado de impacto
				b->cronometro_ms = 0UL;
				b->estado = EYECTOR_PATEANDO;
			}
			break;

			case EYECTOR_PATEANDO:
			b->cronometro_ms += periodo_ms;
			
			// Dejamos el brazo afuera el tiempo configurado para el arrastre (400 ms)
			if (b->cronometro_ms >= CLASIFICADOR_BRAZO_ACTUACION_MS) {
				// ACCIÓN: Retraemos el servo a su posición inicial segura (0°)
				SG90_SetAngle(servos[i], 90U);

				// Transición al estado de retorno
				b->cronometro_ms = 0UL;
				b->estado = EYECTOR_RETRAYENDO;
			}
			break;

			case EYECTOR_RETRAYENDO:
			b->cronometro_ms += periodo_ms;
			
			// Esperamos un tiempo prudencial (ej: 200 ms) para asegurar que el brazo físico volvió a 0°
			// antes de liberar la cola y permitir que el sensor acepte una nueva caja
			if (b->cronometro_ms >= 200U) {
				// CRÍTICO: La caja ya fue eyectada con éxito, la removemos del frente de la FIFO (Pop)
				Clasificador_PopFIFO(b);
				
				// ¡NUEVO!: Le avisamos a la HMI que el brazo sacó esta caja de la cinta
				Tracker_CajaEyectada(b->caja_asignada);
				
				// El eyector vuelve a quedar libre para la próxima caja asignada
				b->estado = EYECTOR_IDLE;
			}
			break;

			default:
			b->estado = EYECTOR_IDLE;
			break;
		}
	}
}
/**
 * @brief Permite reconfigurar dinámicamente los umbrales de las cajas (en mm) desde el HMI.
 */
void Clasificador_ConfigurarUmbrales(Clasificador_Ctrl_t *ctx, float chica, float mediana, float grande, float tolerancia) {
	if (ctx == (void*)0) return;
	
	if (chica > 0.0f)   ctx->config_cajas.umbral_chica_mm   = chica;
	if (mediana > 0.0f) ctx->config_cajas.umbral_mediana_mm = mediana;
	if (grande > 0.0f)  ctx->config_cajas.umbral_grande_mm  = grande;
	if (tolerancia > 0.0f) ctx->config_cajas.tolerancia_mm  = tolerancia;
}

/**
 * @brief Permite modificar dinámicamente el retraso o el tipo de caja asignado a un brazo eyector.
 */
void Clasificador_ConfigurarEyector(Clasificador_Ctrl_t *ctx, uint8_t indice_eyector, Clasificador_TipoCaja_t tipo_caja, uint32_t delay_ms) {
	if (ctx == (void*)0 || indice_eyector >= 3) return;
	
	ctx->eyectores[indice_eyector].caja_asignada = tipo_caja;
	ctx->eyectores[indice_eyector].delay_post_deteccion_ms = delay_ms;
}
/**
 * @brief Permite reconfigurar o calibrar únicamente la distancia al piso en mm.
 */
void Clasificador_ConfigurarPiso(Clasificador_Ctrl_t *ctx, float distancia_piso_mm) {
	if (ctx == (void*)0) return;
	
	if (distancia_piso_mm > 0.0f) {
		ctx->config_cajas.distancia_piso_mm = distancia_piso_mm;
	}
}
/**
 * @brief Inicia el proceso de calibración manual a pedido de la HMI.
 */
void Clasificador_IniciarCalibracionManual(Clasificador_Ctrl_t *ctx, uint8_t cmd_origen) {
	if (ctx == (void*)0) return;
	
	// Configura la mini máquina de estados interna
	ctx->estado_master = ESTADO_CALIBRACION_MANUAL;
	ctx->calib_muestras_restantes = 5U; // Queremos 5 mediciones exactas
	ctx->calib_suma_mm = 0UL;
	ctx->calib_comando_origen = cmd_origen;
	ctx->calib_timer_retrigger_ms = 0U;
	
	// Forzamos el primer disparo del ultrasonido para arrancar la cadena
	// Nota: Como no tenemos el handle de g_hcsr04 acá, dejamos que el evento se dispare.
	// Para cumplir con la arquitectura, declaramos un extern o usamos tu instancia global:
	ctx->calib_timer_retrigger_ms = 60U;
}