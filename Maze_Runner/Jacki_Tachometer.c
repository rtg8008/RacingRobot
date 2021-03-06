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
#include "../inc/Bump.h"
#include "../inc/EUSCIA0.h"
#include "../inc/LPF.h"


#define LPF_N 16     //number of samples in low pass filters

uint16_t Period0;               // 1.67us units
int32_t Speed0;
uint16_t First0;                // Timer A3 first edge, P10.4
int32_t Edges0;                 // Simple counter to count number of edges
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

  if( first_read0 == 0) {
      LPF_Init4(Speed0, LPF_N);
      first_read0 = 1;
  }
  else {
      Speed0 = LPF_Calc4(Speed0);
  }
}

uint16_t Period1;               // 1.67us units
int32_t Speed1;
uint16_t First1;                // Timer A3 first edge, P10.5
int32_t Edges1;                 // Simple counter to count number of edges
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

  if( first_read1 == 0) {
      LPF_Init5(Speed1, LPF_N);
      first_read1 = 1;
  }
  else {
      Speed1 = LPF_Calc5(Speed1);
  }
}

int32_t getSpeed0(void) {       //Right Motor
    return Speed0;
}

int32_t getSpeed1(void) {       //Left Motor
    return Speed1;
}

int32_t getEdges0(void) {       //right motor
    return Edges0;
}

int32_t getEdges1(void) {       //left motor
    return Edges1;
}


void Tach_Init(void){ //main1(void){
  First0 = First1 = 0; // first will be wrong
}

