/*
 * SPI_f.h
 *
 *  Created on: Apr 17, 2020
 *      Author: rq
 */

#ifndef INC_SPI_SX_H_
#define INC_SPI_SX_H_

#include <stdint.h>
#include <stddef.h>
#include <main.h>


void SxLoraBegin(void);

void SetRadioFrequency(long frequency);

void writeFifo(uint8_t *message, uint8_t lenght);
void writeFifoDisplay(uint8_t *message, uint8_t lenght, uint8_t zerar);
void readFifo(uint8_t *message, uint8_t lenght);

void opcodeTx(uint8_t *msg, uint16_t size);
void opcodeTxRx(uint8_t *msg, uint8_t *rsp, uint8_t size);

// -------------------------------------------------------------------------------

/* Escreve em um registrador.
 *
 * O primeiro bit do endereço deve obrigatóriamente
 *  ser 1 para estar no modo de ESCRITA
 */
void writeRegister(uint8_t address, uint8_t value);


/* Retorna o valor de um registrador.
 *
 * O primeiro bit do endereço deve obrigatóriamente
 *  ser 0 para estar no modo de LEITURA
 */
uint8_t readRegister(uint8_t address);

// -------------------------------------------------------------------------------



#endif /* INC_SPI_SX_H_ */
