/*
 * Jacki_Tachometer.h
 *
 *  Created on: Apr 16, 2021
 *      Author: NHG11
 */

#ifndef JACKI_TACHOMETER_H_
#define JACKI_TACHOMETER_H_

void PeriodMeasure0(uint16_t time);

void PeriodMeasure1(uint16_t time);

int32_t getSpeed0(void);

int32_t getSpeed1(void);

int32_t getEdges0(void);

int32_t getEdges1(void);

void Collect(void);

void Debug_FlashInit(void);

void Tach_Init(void);

#endif /* JACKI_TACHOMETER_H_ */
