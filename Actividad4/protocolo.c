#include "protocolo.h"

uint8_t bufRX[BUFRXSIZE];
uint8_t bufTX[BUFTXSIZE];
static uint8_t payloadBuffer[64];
static uint8_t pIdx = 0;
_sRx rx;
_sTx tx;

static CmdParserCallback _cmdParser = 0;

void Protocolo_Init(void) {
	rx.rBuf.buf = bufRX;
	rx.rBuf.size = BUFRXSIZE;
	rx.rBuf.ir = 0;
	rx.rBuf.iw = 0;
	rx.hdrState = 0;
	tx.rBuf.buf = bufTX;
	tx.rBuf.size = BUFTXSIZE;
	tx.rBuf.ir = 0;
	tx.rBuf.iw = 0;
}

void Protocolo_SetCmdParser(CmdParserCallback cb) {
	_cmdParser = cb;
}

void Decode(void) {
	while (rx.rBuf.ir != rx.rBuf.iw) {
		uint8_t b = rx.rBuf.buf[rx.rBuf.ir++];
		rx.rBuf.ir &= (rx.rBuf.size - 1);

		switch (rx.hdrState) {
			case 0: if (b == 'U') { rx.hdrState = 1; rx.cks = b; } break;
			case 1: if (b == 'N') { rx.hdrState = 2; rx.cks ^= b; } else rx.hdrState = 0; break;
			case 2: if (b == 'E') { rx.hdrState = 3; rx.cks ^= b; } else rx.hdrState = 0; break;
			case 3: if (b == 'R') { rx.hdrState = 4; rx.cks ^= b; } else rx.hdrState = 0; break;
			case 4:
			rx.nBytes = b;
			rx.cks ^= b;
			rx.hdrState = 5;
			pIdx = 0;
			break;
			case 5:
			if (b == TOKEN_VAL) { rx.cks ^= b; rx.hdrState = 6; }
			else rx.hdrState = 0;
			break;
			case 6:
			rx.cks ^= b;
			if (rx.nBytes > 1) payloadBuffer[pIdx++] = b;
			rx.nBytes--;
			if (rx.nBytes == 0) {
				if (rx.cks == 0 && _cmdParser)
				_cmdParser(payloadBuffer[0], &payloadBuffer[1], pIdx - 1);
				rx.hdrState = 0;
			}
			break;
		}
	}
}

void Encode(uint8_t cmd, uint8_t* payload, uint8_t n) {
	uint8_t checksum = 0;
	uint8_t length = 1 + n + 1;

	tx.rBuf.buf[tx.rBuf.iw++] = 'U'; tx.rBuf.iw &= (tx.rBuf.size-1); checksum = 'U';
	tx.rBuf.buf[tx.rBuf.iw++] = 'N'; tx.rBuf.iw &= (tx.rBuf.size-1); checksum ^= 'N';
	tx.rBuf.buf[tx.rBuf.iw++] = 'E'; tx.rBuf.iw &= (tx.rBuf.size-1); checksum ^= 'E';
	tx.rBuf.buf[tx.rBuf.iw++] = 'R'; tx.rBuf.iw &= (tx.rBuf.size-1); checksum ^= 'R';
	tx.rBuf.buf[tx.rBuf.iw++] = length; tx.rBuf.iw &= (tx.rBuf.size-1); checksum ^= length;
	tx.rBuf.buf[tx.rBuf.iw++] = ':';    tx.rBuf.iw &= (tx.rBuf.size-1); checksum ^= ':';
	tx.rBuf.buf[tx.rBuf.iw++] = cmd;    tx.rBuf.iw &= (tx.rBuf.size-1); checksum ^= cmd;

	for (uint8_t i = 0; i < n; i++) {
		tx.rBuf.buf[tx.rBuf.iw++] = payload[i];
		tx.rBuf.iw &= (tx.rBuf.size-1);
		checksum ^= payload[i];
	}

	tx.rBuf.buf[tx.rBuf.iw++] = checksum;
	tx.rBuf.iw &= (tx.rBuf.size-1);
}