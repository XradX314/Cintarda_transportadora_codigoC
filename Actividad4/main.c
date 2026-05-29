#define F_CPU 16000000u
#include <avr/io.h>
#include <avr/interrupt.h>
#include "protocolo.h"
#include "comandos.h"
#include "clasificador.h"
#include "botones.h"
#include "hcsr04.h"
#include "tcrt5000.h"
#include "sg90.h"
#include "modo_ciego.h"
#include "tracker.h"

static uint8_t time100ms;
static uint16_t aliveTimer = 0;
uint8_t ledMode = 0;
static volatile uint32_t g_now_us = 0;   /* microsegundos desde arranque */

SG90_t servo1, servo2, servo3;
HCSR04_t g_hcsr04;
IR_Sensor_t ir_s0, ir_s1, ir_s2, ir_s3;
Clasificador_Ctrl_t mi_clasificador;
// NUEVO: Instancia global para el motor cinemático del Modo Ciego
ModoCiego_Ctrl_t mi_modo_ciego;

/* Prototipos de funciones */
void On2Ms(void);
void initPort(void);
void initTMR0(void);
void initUSART0(void);
void init_hardware_hcsr04(void);
void on_resultado(float dis);
void on_s0_released(void);
void on_s1_released(void);
void on_s2_released(void);
void on_s3_released(void);
void on_s0_detected(void);
void on_s1_detected(void);
void on_s2_detected(void);
void on_s3_detected(void);
void hal_servo1(uint8_t state);
void hal_servo2(uint8_t state);
void hal_servo3(uint8_t state);
uint8_t hal_s0(void);
uint8_t hal_s1(void);
uint8_t hal_s2(void);
uint8_t hal_s3(void);
void hal_trig(uint8_t s);
uint8_t hal_echo(void);

// Interrupción del Timer 0 (Cada 200 µs según tu OCR0A = 49)
ISR(TIMER0_COMPA_vect) {
	static uint8_t contador = 0;

	g_now_us += 200;

	// Muestreo del debounce de los sensores ópticos en la ISR rápida
	IR_Tick(&ir_s0);
	IR_Tick(&ir_s1);
	IR_Tick(&ir_s2);
	IR_Tick(&ir_s3);

	// NUEVO: Tick asíncrono para el pulso del ultrasonido (0 bloqueos)
	HCSR04_TickISR(&g_hcsr04);

	contador++;
	if (contador >= 10) { // 10 * 200µs = 2ms
		contador = 0;
		GPIOR0 |= _BV(GPIOR00); // Set de bandera para el Main
	}
}

// Interrupción por comparación del Timer 1: Apaga el pulso de los servos sin jitter
ISR(TIMER1_COMPA_vect) {
	PORTD &= ~(1 << PORTD7);
	PORTB &= ~((1 << PORTB4) | (1 << PORTB3));
}

ISR(USART_RX_vect) {
	rx.rBuf.buf[rx.rBuf.iw++] = UDR0;
	rx.rBuf.iw &= (rx.rBuf.size-1);
}

// Interrupción por cambio de pin (PCINT) del Eco del ultrasonido
ISR(PCINT0_vect) {
	uint16_t timer_snapshot = TCNT1;
	uint8_t pin_state = (PINB & (1 << PORTB2)) ? 1U : 0U;
	HCSR04_UpdateFromISR(&g_hcsr04, timer_snapshot, pin_state);
}

// Tarea de tiempo real ejecutada cada 2ms en el loop principal
void On2Ms(void) {
	GPIOR0 &= ~_BV(GPIOR00);
	
	switch (ledMode){
		case 0:
		time100ms--;
		if (!time100ms) {
			time100ms = 100;
			PINB = (1 << PINB5); // Toggles LED
		}
		break;
		case 1:
		PORTB |= (1 << PINB5);
		break;
	}
	
	// Heartbeat (2500 ciclos * 2ms = 5 segundos)
	aliveTimer++;
	if (aliveTimer >= 250) {
		aliveTimer = 0;
		uint8_t empty[1] = {0};
		Encode(0xF0, empty, 0);
	}

	// Motor de tiempos no bloqueante para los brazos eyectores y colas FIFO
	Clasificador_Task_Brazos_ms(&mi_clasificador, 2U);
	Ciego_Task_Brazos_ms(&mi_modo_ciego, 2U); // Corre en paralelo de forma no bloqueante
	// Control de Timeout por software del ultrasonido
	static uint8_t timeout_cnt = 0;
	if (g_hcsr04.esperando_eco) {
		timeout_cnt++;
		if (timeout_cnt >= 18) {
			timeout_cnt = 0;
			g_hcsr04.esperando_eco = 0;
			g_hcsr04.captura_lista = 0;
			on_resultado(-1.0f);
		}
		} else {
		timeout_cnt = 0;
	}

	// Multiplexor temporal de PWM por software para servos (0 Jitter)
	static uint8_t servo_selector = 0;
	uint16_t current_tcnt = TCNT1;

	switch (servo_selector) {
		case 0:
		PORTD |= (1 << PORTD7);
		OCR1A = current_tcnt + SG90_GetTicks(&servo1);
		servo_selector = 1;
		break;
		case 1:
		PORTB |= (1 << PORTB4);
		OCR1A = current_tcnt + SG90_GetTicks(&servo2);
		servo_selector = 2;
		break;
		case 2:
		PORTB |= (1 << PORTB3);
		OCR1A = current_tcnt + SG90_GetTicks(&servo3);
		servo_selector = 0;
		break;
		default:
		servo_selector = 0;
		break;
	}

	TIFR1 |= (1 << OCF1A);
	TIMSK1 |= (1 << OCIE1A);
}

void initPort(void) {
	// PB1: Trig, PB3..PB5: Servos y LED. PB2: Entrada Echo
	DDRB |= (1<<PORTB1) | (1<<PORTB3) | (1<<PORTB4) | (1<<PORTB5);
	DDRB &= ~(1<<PORTB2);
	
	// PD7: Servo 1. PD2..PD5: Entradas de Infrarrojos s0 a s3
	DDRD |= (1<<PORTD7);
	DDRD &= ~((1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5));
	
	// Pull-ups activos en los sensores ópticos (Entradas Normal Alto)
	PORTD |= (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD5);
}

void initTMR0(void) {
	TCCR0A = (1 << WGM01);    // Modo CTC
	OCR0A = 49;               // 200 µs a 16MHz/64
	TIMSK0 = (1 << OCIE0A);
	TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64
}

void initUSART0(void) {
	UCSR0A = (1 << U2X0); // Doble velocidad
	UBRR0H = (uint8_t)(16 >> 8);
	UBRR0L = (uint8_t)16; // 115200 bps
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

void init_hardware_hcsr04(void) {
	DDRB &= ~(1 << DDB2);    // PB2 (Pin 10) -> Entrada Eco
	TCCR1A = 0;              // Timer 1 Modo Normal
	TCCR1B = (1 << CS11);    // Prescaler 8 (1 tick = 0.5 µs)
	PCICR |= (1 << PCIE0);   // Habilitar Pin Change Interrupt Grupo 0
	PCMSK0 |= (1 << PCINT2); // Asociar PCINT2 (PB2)
}

/* Callbacks de Eventos */
void on_resultado(float dis_mm) {
	// Manejo de errores de hardware (Time-out)
	if (dis_mm < 0.0f) {
		if (mi_clasificador.estado_master == ESTADO_CALIBRACION_MANUAL) {
			mi_clasificador.estado_master = ESTADO_APAGADO;
			uint8_t payload_err[1] = { 0xFF };
			Encode(0x13, payload_err, 1U);
		}
		return; // Descartamos la lectura basura
	}

	// --- YA NO MANDAMOS EL 0x5F ACÁ ---
	// --- YA NO RE-DISPARAMOS EL SENSOR ACÁ ---

	// Le pasamos la pelota a la máquina de estados
	Clasificador_Ultrasonico_Callback(&mi_clasificador, dis_mm);
}

void on_s0_detected(void) {
	// 1. CANDADO: Si la máquina está totalmente apagada, ignoramos el evento físico
	if (mi_clasificador.estado_master == ESTADO_APAGADO && !mi_modo_ciego.activo) {
			uint8_t payload[2] = {0, 1};
			Encode(0x5E, payload, 2);
		return;
	}
	// Pasamos el tiempo global en microsegundos que incrementa tu ISR del Timer 0
	Ciego_InfrarrojoEntrada_Callback(&mi_modo_ciego, 1U, g_now_us);
	uint8_t payload[2] = {0, 1};
	Encode(0x5E, payload, 2);
	Clasificador_InfrarrojoEntrada_Callback(&mi_clasificador, 1U);
	HCSR04_Trigger(&g_hcsr04); // Disparo inicial

}

void on_s0_released(void) {
// 1. CANDADO
if (mi_clasificador.estado_master == ESTADO_APAGADO && !mi_modo_ciego.activo) {
	return;
}

// 2. Transmisión a la HMI (Flanco bajada)
uint8_t payload[2] = {0, 0};
Encode(0x5E, payload, 2);

// 3. PRIMERO: El Clasificador. Cierra la ráfaga, calcula el promedio
// y le avisa al Modo Ciego qué tamaño de caja es (Ciego_AsignarTipoCajaActual).
Clasificador_InfrarrojoEntrada_Callback(&mi_clasificador, 0U);

// 4. SEGUNDO: El Modo Ciego. Como ya sabe qué caja es, ahora sí puede
// calcular el delta T, la velocidad y empujar el delay a la FIFO correcta.
Ciego_InfrarrojoEntrada_Callback(&mi_modo_ciego, 0U, g_now_us);
}

void on_s1_detected(void) {
	uint8_t payload[2] = {1, 1};
	Encode(0x5E, payload, 2);
	Clasificador_InfrarrojoEyector_Callback(&mi_clasificador, 0U, 1U);
}

void on_s1_released(void) {
	uint8_t payload[2] = {1, 0};
	Encode(0x5E, payload, 2);
	// Inyectamos el flanco de bajada (0U) para S1 (0U)
	Clasificador_InfrarrojoEyector_Callback(&mi_clasificador, 0U, 0U);
}

void on_s2_detected(void) {
	uint8_t payload[2] = {2, 1};
	Encode(0x5E, payload, 2);
	Clasificador_InfrarrojoEyector_Callback(&mi_clasificador, 1U, 1U);
}

void on_s2_released(void) {
	uint8_t payload[2] = {2, 0};
	Encode(0x5E, payload, 2);
	// Inyectamos el flanco de bajada (0U) para S2 (1U)
	Clasificador_InfrarrojoEyector_Callback(&mi_clasificador, 1U, 0U);
}

void on_s3_detected(void) {
	uint8_t payload[2] = {3, 1};
	Encode(0x5E, payload, 2);
	Clasificador_InfrarrojoEyector_Callback(&mi_clasificador, 2U, 1U);
}

void on_s3_released(void) {
	uint8_t payload[2] = {3, 0};
	Encode(0x5E, payload, 2);
	// Inyectamos el flanco de bajada (0U) para S3 (2U)
	Clasificador_InfrarrojoEyector_Callback(&mi_clasificador, 2U, 0U);
}

/* Capa de Abstracción de Hardware (HAL) */
void hal_servo1(uint8_t state) { if (state) PORTD |= (1 << PORTD7); else PORTD &= ~(1 << PORTD7); }
void hal_servo2(uint8_t state) { if (state) PORTB |= (1 << PORTB4); else PORTB &= ~(1 << PORTB4); }
void hal_servo3(uint8_t state) { if (state) PORTB |= (1 << PORTB3); else PORTB &= ~(1 << PORTB3); }

uint8_t hal_s0(void) { return (PIND & (1 << PORTD5)) ? 0U : 1U; }
uint8_t hal_s1(void) { return (PIND & (1 << PORTD2)) ? 0U : 1U; }
uint8_t hal_s2(void) { return (PIND & (1 << PORTD3)) ? 0U : 1U; }
uint8_t hal_s3(void) { return (PIND & (1 << PORTD4)) ? 0U : 1U; }

void hal_trig(uint8_t s) { if (s) PORTB |= (1 << PORTB1); else PORTB &= ~(1 << PORTB1); }
uint8_t hal_echo(void) { return (PINB & (1 << PORTB2)) ? 1U : 0U; }

int main(void) {
	cli();

	// 1. Inicialización de Protocolo y Clasificador
	Protocolo_Init();
	Clasificador_Init(&mi_clasificador, 180.0f);
	Comandos_Init(&mi_clasificador); // Enlace del parser modular
	Ciego_Init(&mi_modo_ciego);       // <--- NUEVA INICIALIZACIÓN
	// 2. Inicialización de Periféricos AVR
	initUSART0();
	initPort();
	initTMR0();
	init_hardware_hcsr04();
	
	// 3. Inicialización del Sensor Ultrasónico (CORREGIDO: Se agregó hal_echo)
	HCSR04_Init(&g_hcsr04, hal_trig, on_resultado);
	HCSR04_TuneParams(&g_hcsr04, 12U, 0.1717f);
	
	// 4. Inicialización de Sensores IR
	IR_Init(&ir_s0, hal_s0, on_s0_detected, on_s0_released);
	IR_Init(&ir_s1, hal_s1, on_s1_detected, on_s1_released);
	IR_Init(&ir_s2, hal_s2, on_s2_detected, on_s2_released);
	IR_Init(&ir_s3, hal_s3, on_s3_detected, on_s3_released);
	
	// Filtro digital antirrebote (6ms para todos)
	IR_TuneParams(&ir_s0, 30U);
	IR_TuneParams(&ir_s1, 30U);
	IR_TuneParams(&ir_s2, 30U);
	IR_TuneParams(&ir_s3, 30U);
	
	// 5. Inicialización de estructuras de Servos
	SG90_Init(&servo1, hal_servo1);
	SG90_Init(&servo2, hal_servo2);
	SG90_Init(&servo3, hal_servo3);
	
	sei(); // Habilitación global de interrupciones

	time100ms = 100;

	while (1) {
		if (GPIOR0 & _BV(GPIOR00)) {
			On2Ms();
		}
		
		if (rx.rBuf.ir != rx.rBuf.iw) {
			Decode();
		}
		
		HCSR04_EventHandler(&g_hcsr04); // Procesador asíncrono del eco
		
		// Gestor de transmisión serial por hardware
		if (tx.rBuf.iw != tx.rBuf.ir) {
			if (UCSR0A & _BV(UDRE0)) {
				UDR0 = tx.rBuf.buf[tx.rBuf.ir++];
				tx.rBuf.ir &= (tx.rBuf.size - 1);
			}
		}
	}
}