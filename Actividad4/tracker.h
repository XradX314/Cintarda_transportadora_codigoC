#ifndef TRACKER_H_
#define TRACKER_H_

#include <stdint.h>
#include "clasificador.h"

#define TRACKER_MAX_CAJAS_EN_CINTA 8U

/* Definición Geográfica de Zonas */
typedef enum {
	ZONA_1_S0_S1    = 0x01, // Nace al medir en S0 y viaja hacia S1
	ZONA_2_S1_S2    = 0x02, // Pasó de largo S1 y viaja hacia S2
	ZONA_3_S2_S3    = 0x03, // Pasó de largo S2 y viaja hacia S3
	ZONA_4_DESCARTE = 0x04, // Pasó de largo S3 (Cayó al tacho de basura final)
	ZONA_EYECTADA   = 0x05  // Comando especial: El brazo la empujó a su rampa
} Tracker_Zona_t;

void Tracker_Init(void);
void Tracker_NuevaCaja(Clasificador_TipoCaja_t tipo);
void Tracker_TransicionSensor(uint8_t id_sensor); // id_sensor: 1 (S1), 2 (S2), 3 (S3)
void Tracker_CajaEyectada(Clasificador_TipoCaja_t tipo_asignado_brazo);

#endif /* TRACKER_H_ */