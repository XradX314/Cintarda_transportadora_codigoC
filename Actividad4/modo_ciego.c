#include "modo_ciego.h"
#include "sg90.h"
#include "tracker.h"

// Vinculación con los servos globales del main
extern SG90_t servo1, servo2, servo3;

void Ciego_Init(ModoCiego_Ctrl_t *ctx) {
	if (ctx == (void*)0) return;
	
	ctx->activo = 0U;
	ctx->caja_en_medicion = CAJA_NINGUNA;
	ctx->t_subida_ir0_us = 0UL;
	
	ctx->config.longitud_caja_default_mm = 80.0f;
	ctx->config.distancia_servo_mm[0]    = 100.0f;
	ctx->config.distancia_servo_mm[1]    = 200.0f;
	ctx->config.distancia_servo_mm[2]    = 300.0f;
	ctx->config.offset_golpe_mm          = 20.0f;
	
	for (uint8_t i = 0; i < CIEGO_MAX_CAJAS; i++) {
		ctx->cajas_virtuales[i].activa = 0;
	}
}

void Ciego_ConfigurarGeometria(ModoCiego_Ctrl_t *ctx, float long_caja, float d0, float d1, float d2, float offset_golpe) {
	if (ctx == (void*)0) return;
	if (long_caja > 0.0f) ctx->config.longitud_caja_default_mm = long_caja;
	if (d0 > 0.0f) ctx->config.distancia_servo_mm[0] = d0;
	if (d1 > 0.0f) ctx->config.distancia_servo_mm[1] = d1;
	if (d2 > 0.0f) ctx->config.distancia_servo_mm[2] = d2;
	if (offset_golpe > 0.0f) ctx->config.offset_golpe_mm = offset_golpe;
}

void Ciego_ComandoActivar(ModoCiego_Ctrl_t *ctx, uint8_t activar) {
	if (ctx == (void*)0) return;
	ctx->activo = activar;
	
	// Limpieza de seguridad
	if (!activar) {
		for (uint8_t i = 0; i < CIEGO_MAX_CAJAS; i++) {
			ctx->cajas_virtuales[i].activa = 0;
		}
	}
}

void Ciego_AsignarTipoCajaActual(ModoCiego_Ctrl_t *ctx, Clasificador_TipoCaja_t tipo) {
	if (ctx == (void*)0) return;
	ctx->caja_en_medicion = tipo;
}

void Ciego_InfrarrojoEntrada_Callback(ModoCiego_Ctrl_t *ctx, uint8_t objeto_detectado, uint32_t ahora_us) {
	if (ctx == (void*)0 || !ctx->activo) return;

	if (objeto_detectado) {
		ctx->t_subida_ir0_us = ahora_us;
	}
	else {
		if (ctx->t_subida_ir0_us == 0UL || ctx->caja_en_medicion == CAJA_NINGUNA) return;

		uint32_t delta_t_us = ahora_us - ctx->t_subida_ir0_us;
		if (delta_t_us == 0) return;
		float tiempo_transito_seg = (float)delta_t_us / 1000000.0f;
		float vel_mms = ctx->config.longitud_caja_default_mm / tiempo_transito_seg;
		float t_transcurrido_ms = tiempo_transito_seg * 1000.0f;

		// Buscar asignación
		uint8_t idx_brazo = 255U;
		extern Clasificador_Ctrl_t mi_clasificador;
		if (ctx->caja_en_medicion != CAJA_DESCONOCIDA) {
			for (uint8_t i = 0; i < 3; i++) {
				if (mi_clasificador.eyectores[i].caja_asignada == ctx->caja_en_medicion) {
					idx_brazo = i;
					break;
				}
			}
		}

		// Crear la Caja Virtual en el primer slot libre
		for (uint8_t i = 0; i < CIEGO_MAX_CAJAS; i++) {
			Ciego_CajaVirtual_t *c = &ctx->cajas_virtuales[i];
			if (!c->activa) {
				c->activa = 1;
				c->tipo = ctx->caja_en_medicion;
				c->brazo_asignado = idx_brazo;
				
				// Cálculos cinemáticos de transición de zonas
				c->t_s1_ms = (int32_t)(((ctx->config.distancia_servo_mm[0] / vel_mms) * 1000.0f) - t_transcurrido_ms);
				c->t_s2_ms = (int32_t)(((ctx->config.distancia_servo_mm[1] / vel_mms) * 1000.0f) - t_transcurrido_ms);
				c->t_s3_ms = (int32_t)(((ctx->config.distancia_servo_mm[2] / vel_mms) * 1000.0f) - t_transcurrido_ms);
				c->t_fin_ms = c->t_s3_ms + 1000; // Cae al descarte 1 segundo después de S3

				// Cálculo del golpe si tiene brazo asignado
				if (idx_brazo < 3U) {
					float t_offset_ms = (ctx->config.offset_golpe_mm / vel_mms) * 1000.0f;
					c->t_impacto_ms = (int32_t)(((ctx->config.distancia_servo_mm[idx_brazo] / vel_mms) * 1000.0f) - t_transcurrido_ms + t_offset_ms);
					} else {
					c->t_impacto_ms = 9999999; // Nunca impacta
				}

				c->paso_s1 = 0; c->paso_s2 = 0; c->paso_s3 = 0;
				c->estado_servo = CIEGO_ESTADO_VIAJANDO;
				c->cronometro_servo_ms = 0;

				// ¡Avisamos a la HMI que la caja entró al sistema físico!
				Tracker_NuevaCaja(c->tipo);
				break;
			}
		}
		ctx->caja_en_medicion = CAJA_NINGUNA;
	}
}

void Ciego_Task_Brazos_ms(ModoCiego_Ctrl_t *ctx, uint8_t periodo_ms) {
	if (ctx == (void*)0 || !ctx->activo) return;

	SG90_t *servos[3] = {&servo1, &servo2, &servo3};

	// Procesar de manera independiente la física de cada caja virtual
	for (uint8_t i = 0; i < CIEGO_MAX_CAJAS; i++) {
		Ciego_CajaVirtual_t *c = &ctx->cajas_virtuales[i];

		if (c->activa) {
			// Descontar base de tiempos (incluso si está golpeando, el tiempo físico avanza)
			c->t_s1_ms -= periodo_ms;
			c->t_s2_ms -= periodo_ms;
			c->t_s3_ms -= periodo_ms;
			c->t_impacto_ms -= periodo_ms;
			c->t_fin_ms -= periodo_ms;

			// Disparar las transiciones topológicas falsas para engañar a la HMI
			if (c->t_s1_ms <= 0 && !c->paso_s1) { Tracker_TransicionSensor(1); c->paso_s1 = 1; }
			if (c->t_s2_ms <= 0 && !c->paso_s2) { Tracker_TransicionSensor(2); c->paso_s2 = 1; }
			if (c->t_s3_ms <= 0 && !c->paso_s3) { Tracker_TransicionSensor(3); c->paso_s3 = 1; }

			// --- MÁQUINA DE ESTADOS DEL SERVO ---
			switch (c->estado_servo) {
				case CIEGO_ESTADO_VIAJANDO:
				if (c->brazo_asignado < 3U && c->t_impacto_ms <= 0) {
					// Momento del impacto físico
					SG90_SetAngle(servos[c->brazo_asignado], 0U);
					c->cronometro_servo_ms = 0;
					c->estado_servo = CIEGO_ESTADO_PATEANDO;
				}
				else if (c->t_fin_ms <= 0) {
					// Si era una CAJA_DESCONOCIDA o falló el golpe, se cae al tacho
					c->activa = 0;
				}
				break;

				case CIEGO_ESTADO_PATEANDO:
				c->cronometro_servo_ms += periodo_ms;
				if (c->cronometro_servo_ms >= 400U) { // Tiempo de arrastre
					SG90_SetAngle(servos[c->brazo_asignado], 90U);
					c->cronometro_servo_ms = 0;
					c->estado_servo = CIEGO_ESTADO_RETRAYENDO;
				}
				break;

				case CIEGO_ESTADO_RETRAYENDO:
				c->cronometro_servo_ms += periodo_ms;
				if (c->cronometro_servo_ms >= 200U) { // Tiempo seguro de retracción
					// El golpe terminó exitosamente. Avisamos a la HMI y la matamos
					Tracker_CajaEyectada(c->tipo);
					c->activa = 0;
				}
				break;
			}
		}
	}
}