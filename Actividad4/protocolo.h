#ifndef PROTOCOLO_H
#define PROTOCOLO_H

#include <avr/io.h>

#define BUFRXSIZE 128
#define BUFTXSIZE 128
#define TOKEN_VAL ':'

typedef struct {
	uint8_t *buf;
	volatile uint8_t iw; 
	volatile uint8_t ir; 
	uint8_t size;
} _sRingbuf;

typedef struct {
	_sRingbuf rBuf;
	uint8_t timeout;
	uint8_t hdrState;
	uint8_t nBytes;
	uint8_t cks;
} _sRx;

typedef struct {
	_sRingbuf rBuf;
	uint8_t cks;
} _sTx;

extern _sRx rx;
extern _sTx tx;
extern uint8_t bufRX[BUFRXSIZE];
extern uint8_t bufTX[BUFTXSIZE];

typedef void (*CmdParserCallback)(uint8_t cmd, uint8_t* params, uint8_t len);

void Protocolo_Init(void);
void Protocolo_SetCmdParser(CmdParserCallback cb);
void Decode(void);
void Encode(uint8_t cmd, uint8_t* payload, uint8_t n);

#endif /* PROTOCOLO_H_ */