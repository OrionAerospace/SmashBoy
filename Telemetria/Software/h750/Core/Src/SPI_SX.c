/*
 * SPI_f.c
 *
 *  Created on: Apr 17, 2020
 *      Author: rq
 */

#include "SPI_SX.h"

extern SPI_HandleTypeDef hspi1;
uint8_t singleTransfer(uint8_t address, uint8_t value);

_Bool waitBusy = 0;


void SxLoraBegin(void)
{
	writeRegister(0x01, 0x00);	// Sleep
	writeRegister(0x01, 0x80);	// Lora Mode
	writeRegister(0x09, 0x00);	// Power output -1dBm
	writeRegister(0x0A, 0x09);	// PaRamp 40us
	writeRegister(0x0B, 0x3B);  // max Current
	writeRegister(0x0C, 0x23);	// Gain G1, LNA Boost

	writeRegister(0x0D, 0x00);	// SPI address pointer FIFO
	writeRegister(0x0E, 0x00);	// write base address in FIFO data buffer for TX modulator, daqui pra cima é a parte do buffer que vai ser escrita para ser transmitida
	writeRegister(0x0F, 0x00);	// read base address in FIFO data buffer for RX demodulator, daqui até 0x7F é onde vai ser salvo o que for recebido

	writeRegister(0x39, 0xA1);	// SyncWord     Value 0x34 is reserved for LoRaWAN networks

	writeRegister(0x1D, 0x0A);	// 125kHz, 4/5, Explicit, CRC ON, LowDataRateOptmize OFF
	writeRegister(0x1E, 0x74);	// SF 7, AGC ON

	writeRegister(0x20, 0x00);	// Preamble length MSB
	writeRegister(0x21, 0x08);	// Preamble length LSB
	writeRegister(0x22, 0x01);	// Payload length in bytes.
	writeRegister(0x23, 0xFF);	// Maximum payload length; if header payload length exceeds value a header CRC error is generated. Allows filtering of packet with a bad size.
	writeRegister(0x24, 0x00);	// Freq Hopping Period
	writeRegister(0x4B, 0xAE);	// FastHop

	writeRegister(0x31, 0xC3);	// LoRa detection Optimize:    0x03 -> SF7 to SF12   /   0x05 -> SF6
	writeRegister(0x37, 0x0A);	// LoRa detection threshold:   0x0A -> SF7 to SF12   /   0x0C -> SF6

	writeRegister(0x01, 0x01);	// STDBY

}

void SetRadioFrequency(long frequency)
{
	if(frequency < 860e6)
		frequency = 860e6;

	else if(frequency > 1020e6)
		frequency = 1020e6;

	uint64_t Frf = ((uint64_t)frequency << 19) / 32000000;

	writeRegister(0x06, (uint8_t)(Frf >> 16));
	writeRegister(0x07, (uint8_t)(Frf >> 8));
	writeRegister(0x08, (uint8_t)(Frf >> 0));
}


void writeFifo(uint8_t *message, uint8_t lenght)
{
	writeRegister(0x22, lenght);	// Payload length in bytes.

	for(int i = 0; i < lenght; i++)
	{
		writeRegister(0x00, *(message + i) );
	}

}

void writeFifoDisplay(uint8_t *message, uint8_t lenght, uint8_t zerar)
{
	static uint8_t totalLenght = 0;

	if(zerar == 0)
	{
		totalLenght += lenght;
		writeRegister(0x22, totalLenght);	// Payload length in bytes.

		for(int i = 0; i < lenght; i++)
		{
			writeRegister(0x00, *(message + (2*i)) );
		}

	}
	else
	{
		totalLenght = 0;
	}
}

void readFifo(uint8_t *message, uint8_t lenght)
{
	if(lenght < 256)
	{
		for(int i = 0; i < lenght; i++)
		{
			*(message + i) = readRegister(0x00);
		}
	}

}


uint8_t readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);		// primeiro bit = 0, significa leitura (padrão do SX)
}

void writeRegister(uint8_t address, uint8_t value)
{
	singleTransfer(address | 0x80, value);			// primeiro bit = 1, significa escrita (padrão do SX)
}

uint8_t singleTransfer(uint8_t address, uint8_t value)
{
	uint8_t response = 0;

	HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);		// CS em LOW
	HAL_SPI_Transmit(&hspi1, &address, 1, HAL_MAX_DELAY);									// Transmite o endereço
	HAL_SPI_TransmitReceive(&hspi1, &value, &response, 1, HAL_MAX_DELAY);					// Transmite o comando e le o MISO
	HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);		// CS em HIGH

	return response;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=  SX1262  -=-=-=-=-=-=-=-=-=-=-=-=-=-=

void opcodeTx(uint8_t *msg, uint16_t size)
{
	while( (waitBusy == 1) || ( 0x40 == (GPIOE->IDR & 0x40) ) )   { }		// Aguarda o Busy e o timer
	waitBusy = 1;

	GPIOC->BSRR = 0x00100000;												// CS em LOW
	HAL_SPI_Transmit(&hspi1, msg, size, HAL_MAX_DELAY);						// Transmite o comando
	GPIOC->BSRR = 0x00000010;												// CS em HIGH

	TIM5->CNT = 0x00;														// Zera o timer
	TIM5->CR1 = 0x01;														// Inicia o timer
}

void opcodeTxRx(uint8_t *msg, uint8_t *rsp, uint8_t size)
{
	while( (waitBusy == 1) || ( 0x40 == (GPIOE->IDR & 0x40) ) )   { }		// Aguarda o Busy e o timer
	waitBusy = 1;

	GPIOC->BSRR = 0x00100000;												// CS em LOW
	HAL_SPI_TransmitReceive(&hspi1, msg, rsp, size, HAL_MAX_DELAY);			// Transmite o comando e le o MISO
	GPIOC->BSRR = 0x00000010;												// CS em HIGH

	TIM5->CNT = 0x00;														// Zera o timer
	TIM5->CR1 = 0x01;														// Inicia o timer
}
