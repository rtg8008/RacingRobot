// Jackimain.c
// Runs on MSP432
// Main program for the robot to complete a particular
// challenge where it navigates around a track while 
// avoiding running into the walls.
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


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/Motor.h"
#include "../inc/Tachometer.h"
#include "../inc/FlashProgram.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/TimerA1.h"
#include "../inc/FlashProgram.h"
#include "../inc/Bump.h"
#include "../inc/Jacki_Tachometer.h"
#include "../inc/Jacki_PID.h"
#include "../inc/Reflectance.h"
#include "../inc/LPF.h"
#include "../inc/Ultrasound.h"
#include "../inc/EUSCIA0.h"
#include "../inc/Launchpad.h"

int bump_trigger = 0;

void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

void Port1_Init2(void){
  P1->SEL0 &= ~0x13;
  P1->SEL1 &= ~0x13;   // 1) configure P1.4  P1.1 P1.0 as GPIO
  P1->DIR &= ~0x12;    // 2) make P1.4 and P1.1 in
  P1->DIR |= 0x01;     // 2) make P1.0 out
  P1->REN |= 0x12;     // 3) enable pull resistors on P1.4 and P1.1
  P1->OUT |= 0x12;     //    P1.4 and P1.1 are pull-up
}

void main(void){
    DisableInterrupts();
    Debug_FlashInit();   // initialize flash memory for storage
    Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
    Port1_Init2();      //enable light for solved state
    LaunchPad_Init();
    Ultrasound_Init();   // inititialize ultrasound sensors
    Motor_Init();        // activate Lab 12 software
    TimerA3Capture_Init(&PeriodMeasure0, &PeriodMeasure1, &Wheel_Controller, &Position_Controller);    // initialize tachometers and PID controllers
    Motor_Forward(5000,5000);
    TimerA1_Init(&Reflectance_Start, &Reflectance_End);     // initialize line sensors
    Bump_Edge_Init();         // initialize bump sensors
    Tach_Init();         // Initialize Tachometers
    EUSCIA0_Init();     // initialize UART
    EnableInterrupts();

  while(1){};
}

void PORT4_IRQHandler(void) {
    P4->IFG &= ~0xED;          // clear interrupt flag
    bump_trigger = 1;
    Motor_Stop();
}
