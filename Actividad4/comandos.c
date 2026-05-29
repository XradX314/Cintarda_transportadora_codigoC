#include "comandos.h"
#include "protocolo.h"
#include "modo_ciego.h"

extern ModoCiego_Ctrl_t mi_modo_ciego; // Instancia global

static Clasificador_Ctrl_t *_clasificador = (void*)0;
static void Comandos_ParserCallback(uint8_t cmd, uint8_t* params, uint8_t len);

void Comandos_Init(Clasificador_Ctrl_t *clasificador_ctx) {
	if (clasificador_ctx == (void*)0) return;
	_clasificador = clasificador_ctx;
	Protocolo_SetCmdParser(Comandos_ParserCallback);
}

static void Comandos_ParserCallback(uint8_t cmd, uint8_t* params, uint8_t len) {
	if (_clasificador == (void*)0) return;

	uint8_t comando_valido = 1U;

	// 1. EVALUAR COMANDOS OPERATIVOS (Se permiten en cualquier momento)
	switch (cmd) {
		case CMD_START_SISTEMA:
		// Si ya estaba en modo normal, ignoramos para no reiniciar variables
		if (_clasificador->estado_master == ESTADO_APAGADO) {
			Ciego_ComandoActivar(&mi_modo_ciego, 0U);
			Clasificador_ComandoInicio(_clasificador);
			} else {
			comando_valido = 0U; // No corresponde ACK si ya estaba corriendo
		}
		break;
		
		case CMD_START_CIEGO:
		// Solo permitimos arrancar el Modo Ciego si la máquina venía de estar apagada
		if (_clasificador->estado_master == ESTADO_APAGADO && !mi_modo_ciego.activo) {
			Clasificador_ComandoParada(_clasificador);
			Ciego_ComandoActivar(&mi_modo_ciego, 1U);
			} else {
			comando_valido = 0U;
		}
		break;

		case CMD_STOP_SISTEMA:
		Clasificador_ComandoParada(_clasificador);
		Ciego_ComandoActivar(&mi_modo_ciego, 0U);
		break;
		
		default:
		// 2. FILTRO DE SEGURIDAD GENERAL PARA CONFIGURACIONES
		// Si no es un botón de arranque/parada, obligatoriamente debe estar APAGADO
		if (_clasificador->estado_master != ESTADO_APAGADO || mi_modo_ciego.activo) {
			comando_valido = 0U;
		}
		break;
	}

	// 3. PROCESAMIENTO DE CONFIGURACIONES (Solo si comando_valido sigue en 1)
	if (comando_valido) {
		switch (cmd) {
			// Casos operativos ya resueltos arriba
			case CMD_START_SISTEMA:
			case CMD_START_CIEGO:
			case CMD_STOP_SISTEMA:
			break;

			case CMD_SET_GEOMETRIA_CIEGA: // <--- CORREGIDO: Ahora está protegido bajo el ala del ESTADO_APAGADO
			if (len >= 5) {
				float largo_caja   = (float)params[0] * 10.0f;
				float d0           = (float)params[1] * 10.0f;
				float d1           = (float)params[2] * 10.0f;
				float d2           = (float)params[3] * 10.0f;
				float offset_golpe = (float)params[4] * 10.0f;
				
				Ciego_ConfigurarGeometria(&mi_modo_ciego, largo_caja, d0, d1, d2, offset_golpe);
				} else {
				comando_valido = 0U;
			}
			break;

			case CMD_SET_DELAYS:
			if (len >= 6) {
				uint8_t idx       = params[0];
				uint8_t tipo_caja = params[1];
				
				uint32_t delay_ms = ((uint32_t)params[2])        |
				((uint32_t)params[3] << 8)  |
				((uint32_t)params[4] << 16) |
				((uint32_t)params[5] << 24);
				
				Clasificador_ConfigurarEyector(_clasificador, idx, (Clasificador_TipoCaja_t)tipo_caja, delay_ms);
				} else {
				comando_valido = 0U;
			}
			break;

			case CMD_SET_UMBRALES:
			if (len >= 4) {
				float chica   = (float)params[0];
				float mediana = (float)params[1];
				float grande  = (float)params[2];
				float tol     = (float)params[3];
				
				Clasificador_ConfigurarUmbrales(_clasificador, chica, mediana, grande, tol);
				} else {
				comando_valido = 0U;
			}
			break;

			case CMD_SET_PISO:
			if (len >= 1) {
				float nuevo_piso = (float)params[0];
				Clasificador_ConfigurarPiso(_clasificador, nuevo_piso);
				} else {
				comando_valido = 0U;
			}
			break;

			case CMD_MEDIR_CALIBRACION:
			Clasificador_IniciarCalibracionManual(_clasificador, cmd);
			comando_valido = 0U; // Responde con su propia trama de datos, no con ACK común
			break;

			default:
			comando_valido = 0U;
			break;
		}
	}

	// 4. RESPUESTA DE SEGURIDAD
	if (comando_valido) {
		uint8_t payload_ack[1];
		payload_ack[0] = cmd;
		Encode(CMD_ACK, payload_ack, 1U);
	}
}