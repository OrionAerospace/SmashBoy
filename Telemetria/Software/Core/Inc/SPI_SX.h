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
void opcodeTxRx(uint8_t *msg, uint8_t *rsp, uint16_t size);

void SX_WriteBuffer(uint8_t offset, uint8_t *data, uint16_t size);
void SX_WriteRegister(uint16_t address, uint8_t *data, uint16_t size);
void SX_WriteRegisterByte(uint16_t address, uint8_t data);
uint8_t SX_ReadRegisterByte(uint16_t address);
void SX_ReadBuffer(uint8_t **payload, uint8_t offset, uint16_t size);

int SX_TX(uint32_t timeout);
int SX_RX(uint32_t timeout);

void SX_ClearDeviceErrors();
void SX_SetDIO3AsTCXOCtrl(uint8_t tcxoVoltage, uint32_t delay);
void SX_CalibrateFunction(uint8_t calibParam);
void SX_CalibrateImage();
int16_t SX_GetDeviceErrors();

void SX_SetStandby(uint8_t StdbyConfig);
void SX_SetPacketType(uint8_t PacketType);
void SX_SetFrequency(uint64_t Frequency);
void SX_SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel);
void SX_SetTxParam(uint8_t power, uint8_t RampTime);
void SX_SetBufferBaseAddress(uint8_t TX_base_address, uint8_t RX_base_address);
void SX_SetModulationParamsLoRa(uint8_t SF, uint8_t BW, uint8_t CR, uint8_t LDR_Opt);
void SX_SetPacketParamsLoRa(uint16_t PreambleLength, uint8_t HeaderType, uint8_t Payloadlength, uint8_t CRCType, uint8_t InvertIQ);
void SX_SetDioIrqParams(uint16_t Irq_Mask, uint16_t DIO1_Mask, uint16_t DIO2_Mask, uint16_t DIO3_Mask);
void SX_GetRxBufferStatus(uint8_t *status, uint8_t *PayloadLengthRx, uint8_t *RxStartBufferPointer);

void SX_ClearIrqStatus(uint16_t ClearIrqParam);

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
