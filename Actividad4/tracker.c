#include "tracker.h"
#include "protocolo.h"

#define CMD_STATUS_CINTA 0x5D

// Estructura de rastreo geográfico
typedef struct {
	uint8_t  id;
	uint8_t  tipo;
	uint8_t  zona;
	uint8_t  activa;
	uint32_t ticket; // Para saber cuál entró primero a la cinta
} Tracker_Caja_t;

static Tracker_Caja_t g_cajas[TRACKER_MAX_CAJAS_EN_CINTA];
static uint8_t g_id_contador = 0;
static uint32_t g_ticket_global = 0;

void Tracker_Init(void) {
	g_id_contador = 0;
	g_ticket_global = 0;
	for(uint8_t i=0; i<TRACKER_MAX_CAJAS_EN_CINTA; i++) {
		g_cajas[i].activa = 0;
	}
}

static void Tracker_EnviarHMI(uint8_t id, uint8_t tipo, uint8_t zona) {
	uint8_t payload[3];
	payload[0] = id;
	payload[1] = tipo;
	payload[2] = zona;
	Encode(CMD_STATUS_CINTA, payload, 3U);
}

// Se llama cuando el S0 termina de medir y sabemos qué caja es
void Tracker_NuevaCaja(Clasificador_TipoCaja_t tipo) {
	if (tipo == CAJA_NINGUNA) return;

	for(uint8_t i=0; i<TRACKER_MAX_CAJAS_EN_CINTA; i++) {
		if (!g_cajas[i].activa) {
			g_id_contador++;
			if (g_id_contador == 0) g_id_contador = 1; // Evitamos mandar ID 0

			g_cajas[i].id = g_id_contador;
			g_cajas[i].tipo = (uint8_t)tipo;
			g_cajas[i].zona = ZONA_1_S0_S1;
			g_cajas[i].activa = 1;
			g_cajas[i].ticket = ++g_ticket_global;

			Tracker_EnviarHMI(g_cajas[i].id, g_cajas[i].tipo, g_cajas[i].zona);
			return;
		}
	}
}

// Se llama cuando un sensor S1, S2 o S3 deja de ver la caja (flanco bajada)
void Tracker_TransicionSensor(uint8_t id_sensor) {
	uint8_t zona_origen = id_sensor;          // Ej: Si S1 dejó de verla, estaba en Zona 1...
	uint8_t zona_destino = id_sensor + 1;     // ...y acaba de pasar a la Zona 2.
	
	int8_t idx_mas_vieja = -1;
	uint32_t ticket_minimo = 0xFFFFFFFF;

	// Buscamos la caja más antigua en la zona de origen
	for(uint8_t i=0; i<TRACKER_MAX_CAJAS_EN_CINTA; i++) {
		if (g_cajas[i].activa && g_cajas[i].zona == zona_origen) {
			if (g_cajas[i].ticket < ticket_minimo) {
				ticket_minimo = g_cajas[i].ticket;
				idx_mas_vieja = i;
			}
		}
	}

		if (idx_mas_vieja != -1) {
			g_cajas[idx_mas_vieja].zona = zona_destino;
			Tracker_EnviarHMI(g_cajas[idx_mas_vieja].id, g_cajas[idx_mas_vieja].tipo, zona_destino);
	
			// Si acaba de pasar el S3, se cae al tacho de basura
			if (zona_destino == ZONA_4_DESCARTE) {
				// Si es una caja desconocida, sabemos que ningún brazo la va a patear.
				// La borramos de la RAM del microcontrolador al instante.
				if (g_cajas[idx_mas_vieja].tipo == (uint8_t)CAJA_DESCONOCIDA) {
					g_cajas[idx_mas_vieja].activa = 0;
				}
				// (Si fuera una caja GRANDE, la borraremos cuando el brazo 3 termine de golpearla).
			}
		}

	
	}	

// Se llama cuando un brazo eyector termina de golpearla
void Tracker_CajaEyectada(Clasificador_TipoCaja_t tipo_asignado_brazo) {
	int8_t idx_mas_vieja = -1;
	uint32_t ticket_minimo = 0xFFFFFFFF;

	// Buscamos la caja más vieja que coincida con el tipo de ese brazo
	for(uint8_t i=0; i<TRACKER_MAX_CAJAS_EN_CINTA; i++) {
		if (g_cajas[i].activa && g_cajas[i].tipo == (uint8_t)tipo_asignado_brazo) {
			if (g_cajas[i].ticket < ticket_minimo) {
				ticket_minimo = g_cajas[i].ticket;
				idx_mas_vieja = i;
			}
		}
	}

	if (idx_mas_vieja != -1) {
		Tracker_EnviarHMI(g_cajas[idx_mas_vieja].id, g_cajas[idx_mas_vieja].tipo, ZONA_EYECTADA);
		g_cajas[idx_mas_vieja].activa = 0; // Liberamos la memoria
	}
}