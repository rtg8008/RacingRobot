/*
 * Tachometer.c
 *
 *  Created on: Apr 16, 2021
 *      Author: NHG11
 */



// Lab16_Tachmain.c
// Runs on MSP432
// Test the operation of the tachometer by implementing
// a simple DC motor speed controller.
// Daniel Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// See Bump.c for bumper connections (Port 8 or Port 4)

// Debug heartbeat connected to P2.0 (built-in red LED)
// Debug heartbeat connected to P2.4

// Pololu kit v1.1 connections:
// Left Encoder A connected to P10.5 (J5)
// Left Encoder B connected to P5.2 (J2.12)
// Right Encoder A connected to P10.4 (J5)
// Right Encoder B connected to P5.0 (J2.13)

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

// Negative logic bump sensors defined in Bump.c (use Port 4)
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// Debug heartbeat connected to P2.0 (built-in red LED)
// Debug heartbeat connected to P1.0 (built-in LED1)

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/Motor.h"
#include "../inc/Tachometer.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/TExaS.h"
#include "../inc/FlashProgram.h"
#include "../inc/Bump.h"
#include "../inc/EUSCIA0.h"
#include "../inc/LPF.h"

uint16_t Duty;
int32_t* DutyBuffer;
uint16_t Time;
uint32_t flashAddr;
int LPF_N = 64;     //number of samples in low pass filters

uint16_t Period0;               // 1.67us units
int32_t Speed0;
uint16_t First0;                // Timer A3 first edge, P10.4
uint8_t Done0;                  // set each rising
int32_t Edges0;                 // Simple counter to count number of edges
int32_t* SpeedArr0;
int first_read0 = 0;
void PeriodMeasure0(uint16_t time){

  Period0 = (time - First0)&0xFFFF; // 16 bits, 1.67us resolution
  if((P5->IN)&(0x01))
  {
      Edges0++;
      Speed0 = 100000/Period0;
  }
  else
  {
      Edges0--;
      Speed0 = -100000/Period0;
  }
  First0 = time;               // setup for next
  Done0 = 1;

  if( first_read0 == 0) {
      LPF_Init(Speed0, LPF_N);
      first_read0 = 1;
  }
  else {
      Speed0 = LPF_Calc(Speed0);
  }
}

uint16_t Period1;               // 1.67us units
int32_t Speed1;
uint16_t First1;                // Timer A3 first edge, P10.5
uint8_t Done1;                  // set each rising
int32_t Edges1;                 //
int32_t* SpeedArr1;
int first_read1 = 0;
void PeriodMeasure1(uint16_t time){
  Period1 = (time - First1)&0xFFFF; // 16 bits, 1.67us resolution
  if((P5->IN)&(0x04))
    {
        Edges1++;
        Speed1 = 100000/Period1;
    }
    else
    {
        Edges1--;
        Speed1 = -100000/Period1;
    }
  First1 = time;               // setup for next
  Done1 = 1;

  if( first_read1 == 0) {
      LPF_Init2(Speed0, LPF_N);
      first_read1 = 1;
  }
  else {
      Speed1 = LPF_Calc2(Speed1);
  }
}

int32_t getSpeed0(void) {       //Right Motor
    return Speed0;
}

int32_t getSpeed1(void) {       //Left Motor
    return Speed1;
}

void Collect(void){
  if(Done0==0) Period0 = 65534; // stopped
  if(Done1==0) Period1 = 65534; // stopped
  Done0 = Done1 = 0;   // set on subsequent
  if(Time==200){       // 2 sec
    Duty = 7500;
    Motor_Forward(Duty, Duty);  // 50%
  }
  if(Time==400){       // 4 sec
    Duty = 11250;
    Motor_Forward(Duty, Duty);// 75%
  }
  if(Time==600){       // 6 sec
    Duty = 14998;
    Motor_Forward(Duty, Duty);  // 100%
  }
  if(Time==800){       // 8 sec
    Duty = 3750;
    Motor_Forward(Duty, Duty);  // 25%
  }
  if(Time<2000){        // 20 sec
      if(Period1 == 65535)
      {
          SpeedArr1[Time%10] = 0;
      }
      else
      {
          SpeedArr1[Time%10] = Speed1;
      }
      if(Period0 == 65535)
      {
          SpeedArr0[Time%10] = 0;
      }
      else
      {
          SpeedArr0[Time%10] = Speed0;
      }
      DutyBuffer[Time%10] = Duty;
      Time++;
      if(!(Time%10)&&(Time>0))
      {
          //EUSCIA0_OutSDec(SpeedArr0[0]);
          //EUSCIA0_OutString("\n");
          uint32_t count = Flash_WriteArray(SpeedArr0,flashAddr,10);
          Flash_WriteArray(SpeedArr1,flashAddr+10000,10);
          Flash_WriteArray(DutyBuffer,flashAddr+20000,10);
          flashAddr += 4*count;
      }
  }
  if((Time>=2000)||Bump_Read()){
    Duty = 0;
    Motor_Stop();      // 0%
    TIMER_A1->CTL &= ~0x0030; //Stop the clock
  }
}


void Debug_FlashInit(void){
    flashAddr = 0x00020000;
    while(flashAddr < 0x0003FFFF)
    {
        Flash_Erase(flashAddr);
        flashAddr += 4096;
    }
    flashAddr = 0x00020000;
}

void Tach_Init(void){ //main1(void){
  First0 = First1 = 0; // first will be wrong
  Done0 = Done1 = 0;   // set on subsequent
  Time = 0; Duty = 3750;
  SpeedArr0 = (int32_t*)malloc(20*sizeof(int32_t));
  SpeedArr1 = (int32_t*)malloc(20*sizeof(int32_t));
  DutyBuffer = (uint16_t*)malloc(20*sizeof(uint16_t));
}

