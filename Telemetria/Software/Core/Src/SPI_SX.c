/*
 * SPI_f.c
 *
 *  Created on: Apr 17, 2020
 *      Author: rq
 */

#include "SPI_SX.h"
#include "cmsis_os.h"

uint8_t SetStandby[2] = {0x80, 0x00};
uint8_t SetDIO3AsTCXOCtrl[5] = {0x97, 0x07, 0x00, 0x00, 0x32};							// Detectar via software se acontece erro
uint8_t CalibrateFunction[2] = {0x89, 0x7F};
uint8_t CalibrateImage[3] = {0x98, 0xE1, 0xE9};
uint8_t ClearDeviceErrors[3] = {0x07, 0, 0};
uint8_t GetDeviceErrors[4] = {0x17, 0, 0, 0};

uint8_t SetPacketType[2] = {0x8A, 0x01};
uint8_t SetFrequency[5] = {0x86, 0x39, 0x30, 0x00, 0x00};
uint8_t SetPaConfig[5] = {0x95, 0x02, 0x02, 0x00, 0x01}; 								// output Power 14dBm
uint8_t SetTxParam[3] = {0x8E, 0x0E, 0x03};												// 14dBm
uint8_t SetBufferBaseAddress[3] = {0x8F, 0x7F, 0x00};									// TX: 0x7F	 RX: 0x00
uint8_t GetRxBufferStatus[4] = {0x13, 0, 0, 0};
uint8_t GetPacketStatus[5] = {0x14, 0, 0, 0, 0};

uint8_t SetModulationParams[9] = {0x8B, 0x07, 0x04, 0x01, 0x00, 0, 0, 0, 0};
uint8_t SetPacketParam[10] = {0x8C, 0x00, 0x08, 0x00, 0xFF, 0x01, 0x00, 0, 0, 0};
uint8_t SetDioIrqParams[9] = {0x08, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t SyncWord[5] = {0x0D, 0x07, 0x40, 0x47, 0x76};
uint8_t SetTx[4] = {0x83, 0x00, 0x00, 0x00};
uint8_t SetRx[4] = {0x82, 0x00, 0x00, 0x00};

uint8_t GetIrqStatus[4] = {0x12, 0, 0, 0};
uint8_t ClearIrqStatus[3] = {0x2, 0x00, 0x00};


extern SPI_HandleTypeDef hspi1;
uint8_t singleTransfer(uint8_t address, uint8_t value);


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

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);		// CS em LOW
	HAL_SPI_Transmit(&hspi1, &address, 1, HAL_MAX_DELAY);									// Transmite o endereço
	HAL_SPI_TransmitReceive(&hspi1, &value, &response, 1, HAL_MAX_DELAY);					// Transmite o comando e le o MISO
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);		// CS em HIGH

	return response;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=  SX1262  -=-=-=-=-=-=-=-=-=-=-=-=-=-=

_Bool waitBusy = 0;


void opcodeTx(uint8_t *msg, uint16_t size)
{
	while( (waitBusy == 1) || ( 0x40 == (GPIOE->IDR & 0x40) ) )   { }		// Aguarda o Busy e o timer
	waitBusy = 1;

	GPIOC->BSRR = 0x00100000;												// CS em LOW
	HAL_SPI_Transmit(&hspi1, msg, size, HAL_MAX_DELAY);						// Transmite o comando
	GPIOC->BSRR = 0x00000010;												// CS em HIGH

	TIM5->CR1 = 0x05;														// Start timer
}

void opcodeTxRx(uint8_t *msg, uint8_t *rsp, uint16_t size)
{
	while( (waitBusy == 1) || ( 0x40 == (GPIOE->IDR & 0x40) ) )   { }		// Aguarda o Busy e o timer
	waitBusy = 1;

	GPIOC->BSRR = 0x00100000;												// CS em LOW
	HAL_SPI_TransmitReceive(&hspi1, msg, rsp, size, HAL_MAX_DELAY);			// Transmite o comando e le o MISO
	GPIOC->BSRR = 0x00000010;												// CS em HIGH

	TIM5->CR1 = 0x05;														// Inicia o timer
}

void SX_SetStandby(uint8_t StdbyConfig)
{
	SetStandby[1] = StdbyConfig;
	opcodeTx(SetStandby, 2);
}

void SX_ClearDeviceErrors()
{
	opcodeTx(ClearDeviceErrors, 3);
}

void SX_SetDIO3AsTCXOCtrl(uint8_t tcxoVoltage, uint32_t delay)
{
	SetDIO3AsTCXOCtrl[1] = tcxoVoltage;
	SetDIO3AsTCXOCtrl[2] = delay & 0xFF0000;
	SetDIO3AsTCXOCtrl[3] = delay & 0xFF00;
	SetDIO3AsTCXOCtrl[4] = delay & 0xFF;

	opcodeTx(SetDIO3AsTCXOCtrl, 5);
}

void SX_CalibrateFunction(uint8_t calibParam)
{
	CalibrateFunction[1] = calibParam & 0x7F;
	opcodeTx(CalibrateFunction, 2);
}

void SX_CalibrateImage()
{
	opcodeTx(CalibrateImage, 3);
}

int16_t SX_GetDeviceErrors()
{
	uint8_t  resp[4];
	opcodeTxRx(GetDeviceErrors, resp, 4);

	return (resp[2] << 8) | resp[3];
}

void SX_SetPacketType(uint8_t PacketType)
{
	SetPacketType[1] = PacketType;

	opcodeTx(SetPacketType, 2);
}

void SX_SetFrequency(uint64_t Frequency)
{
	uint64_t RFreq = (Frequency * (1 << 25)) / 32000000;

	SetFrequency[1] = (RFreq >> 24) & 0xFF;
	SetFrequency[2] = (RFreq >> 16) & 0xFF;
	SetFrequency[3] = (RFreq >> 8) & 0xFF;
	SetFrequency[4] = RFreq & 0xFF;

	opcodeTx(SetFrequency, 5);
}

void SX_SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel)
{
	SetPaConfig[1] = paDutyCycle;
	SetPaConfig[2] = hpMax;
	SetPaConfig[3] = deviceSel;

	opcodeTx(SetPaConfig, 5);
}

void SX_SetTxParam(uint8_t power, uint8_t RampTime)
{
	SetTxParam[1] = power;
	SetTxParam[2] = RampTime;

	opcodeTx(SetTxParam, 3);
}

void SX_SetBufferBaseAddress(uint8_t TX_base_address, uint8_t RX_base_address)
{
	SetBufferBaseAddress[1] = TX_base_address;
	SetBufferBaseAddress[2] = RX_base_address;

	opcodeTx(SetBufferBaseAddress, 3);
}

void SX_WriteRegister(uint16_t address, uint8_t *data, uint16_t size)
{
	uint8_t *header;

	header = pvPortMalloc(sizeof(uint8_t) * (size + 3));

	*header = 0x0D;
	*(header + 1) = (address >> 8) & 0xFF;
	*(header + 2) = address & 0xFF;

	for(int i = 0; i < size; i++)
		*(header + 3 + i) = *(data + i);

	opcodeTx(header, size+3);

	vPortFree(header);
}

void SX_WriteRegisterByte(uint16_t address, uint8_t data)
{
	uint8_t header[4];

	header[0] = 0x0D;
	header[1] = (address >> 8) & 0xFF;
	header[2] = address & 0xFF;
	header[3] = data;

	opcodeTx(header, 4);
}

uint8_t SX_ReadRegisterByte(uint16_t address)
{
	uint8_t header[5], resp[5];

	header[0] = 0x1D;
	header[1] = (address >> 8) & 0xFF;
	header[2] = address & 0xFF;
	header[3] = 0;
	header[4] = 0;

	opcodeTxRx(header, resp, 5);
	return resp[4];
}

void SX_WriteBuffer(uint8_t offset, uint8_t *data, uint16_t size)
{
	uint8_t *header;

	header = pvPortMalloc(sizeof(uint8_t) * (size + 2));

	*header = 0x0E;
	*(header + 1) = offset;

	for(int i = 0; i < size; i++)
		*(header + 2 + i) = *(data + i);

	opcodeTx(header, size+2);

	vPortFree(header);
}

void SX_ReadBuffer(uint8_t **payload, uint8_t offset, uint16_t size)
{
	// Os dados da memoria do SX1262 vão ser copiados para o endereço que payload apontar

	static uint8_t buffer[259];
	uint8_t header[2] = {0x1E, offset};

	opcodeTxRx(header, buffer, size+3);

	*payload = &buffer[3];

}

void SX_SetModulationParamsLoRa(uint8_t SF, uint8_t BW, uint8_t CR, uint8_t LDR_Opt)
{
	SetModulationParams[1] = SF;
	SetModulationParams[2] = BW;
	SetModulationParams[3] = CR;
	SetModulationParams[4] = LDR_Opt;
	SetModulationParams[5] = 0;
	SetModulationParams[6] = 0;
	SetModulationParams[7] = 0;
	SetModulationParams[8] = 0;

	opcodeTx(SetModulationParams, 9);
}

void SX_SetPacketParamsLoRa(uint16_t PreambleLength, uint8_t HeaderType, uint8_t Payloadlength, uint8_t CRCType, uint8_t InvertIQ)
{
	SetPacketParam[1] = (PreambleLength >> 8) & 0xFF;
	SetPacketParam[2] = PreambleLength & 0xFF;
	SetPacketParam[3] = HeaderType;
	SetPacketParam[4] = Payloadlength;
	SetPacketParam[5] = CRCType;
	SetPacketParam[6] = InvertIQ;
	SetPacketParam[7] = 0;
	SetPacketParam[8] = 0;
	SetPacketParam[9] = 0;

	opcodeTx(SetPacketParam, 10);

	// Workaround 2

	uint8_t value;
	value = SX_ReadRegisterByte(0x08D8);
	value = value | 0x1E;
	SX_WriteRegisterByte(0x08D8, value);
}

void SX_SetDioIrqParams(uint16_t Irq_Mask, uint16_t DIO1_Mask, uint16_t DIO2_Mask, uint16_t DIO3_Mask)
{
	SetDioIrqParams[1] = Irq_Mask >> 8;
	SetDioIrqParams[2] = Irq_Mask & 0xFF;
	SetDioIrqParams[3] = DIO1_Mask >> 8;
	SetDioIrqParams[4] = DIO1_Mask & 0xFF;
	SetDioIrqParams[5] = DIO2_Mask >> 8;
	SetDioIrqParams[6] = DIO2_Mask & 0xFF;
	SetDioIrqParams[7] = DIO3_Mask >> 8;
	SetDioIrqParams[8] = DIO3_Mask & 0xFF;

	opcodeTx(SetDioIrqParams, 9);
}

void SX_ClearIrqStatus(uint16_t ClearIrqParam)
{
	ClearIrqStatus[1] = (ClearIrqParam >> 8) & 0xFF;
	ClearIrqStatus[2] = ClearIrqParam & 0xFF;

	opcodeTx(ClearIrqStatus, 3);
}

void SX_GetRxBufferStatus(uint8_t *status, uint8_t *PayloadLengthRx, uint8_t *RxStartBufferPointer)
{
	uint8_t rsp[4];
	opcodeTxRx(GetRxBufferStatus, rsp, 4);

	*status = rsp[1];
	*PayloadLengthRx = rsp[2];
	*RxStartBufferPointer = rsp[3];
}


int SX_TX(uint32_t timeout)
{
	HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET);

	// Workaround 1

	if(SetPacketType[1] == 0x01) {					// LoRa
			if(SetModulationParams[2] == 0x06)		// BW = 500 KHz
			{
				uint8_t value;
				value = SX_ReadRegisterByte(0x0889);
				value = value & 0xFB;
				SX_WriteRegisterByte(0x0889, value);
			}
		}
		else {
			uint8_t value;
			value = SX_ReadRegisterByte(0x0889);
			value = value | 0x04;
			SX_WriteRegisterByte(0x0889, value);
		}

	SetTx[1] = (timeout >> 16) & 0xFF;
	SetTx[2] = (timeout >> 8) & 0xFF;
	SetTx[3] = timeout & 0xFF;

	opcodeTx(SetTx, 4);

	return 1;
}

int SX_RX(uint32_t timeout)
{
	HAL_GPIO_WritePin(RXEN_GPIO_Port, RXEN_Pin, GPIO_PIN_SET);

	SetRx[1] = (timeout >> 16) & 0xFF;
	SetRx[2] = (timeout >> 8) & 0xFF;
	SetRx[3] = timeout & 0xFF;

	opcodeTx(SetRx, 4);

	return 1;
}
