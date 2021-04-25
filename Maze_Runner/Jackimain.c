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
#include "../inc/TA3InputCapture.h"
#include "../inc/TimerA1.h"
#include "../inc/Bump.h"
#include "../inc/Jacki_Tachometer.h"
#include "../inc/Jacki_PID.h"
#include "../inc/Reflectance.h"
#include "../inc/LPF.h"
#include "../inc/Ultrasound.h"
#include "../inc/EUSCIA0.h"
#include "../inc/Launchpad.h"
#include "../inc/UART1.h"
#include "../inc/AP.h"

int bump_trigger = 0;

void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

//light for solved state
void Port1_Init2(void){
  P1->SEL0 &= ~0x01;
  P1->SEL1 &= ~0x01;   // configure P1.0 as GPIO
  P1->DIR |= 0x01;     // make P1.0 out
  P1->OUT &= ~0x01;    // set to low initially
}

void Port2_Init2(void){
  P2->SEL0 &= ~0x07;
  P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
  P2->DIR |= 0x07;      // 2) make P2.2-P2.0 out
  P2->DS |= 0x07;       // 3) activate increased drive strength
  P2->OUT &= ~0x07;     //    all LEDs off
}

void main(void){
    Go = 1;
    DisableInterrupts();
    Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
    Port1_Init2();      //enable light for solved state
    Port2_Init2();
    LaunchPad_Init();
    Ultrasound_Init();   // inititialize ultrasound sensors
    Motor_Init();        // activate Lab 12 software
    TimerA3Capture_Init(&PeriodMeasure0, &PeriodMeasure1, &Wheel_Controller, &Position_Controller);    // initialize tachometers and PID controllers
    Motor_Forward(5000,5000);
    //TimerA1_Init(&Reflectance_Start, &Reflectance_End);     // initialize line sensors
    Bump_Edge_Init();         // initialize bump sensors
    Tach_Init();         // Initialize Tachometers
    EUSCIA0_Init();     // initialize UART
    UART0_Init();
    EnableInterrupts();
    uint32_t time=0;
    BLE_Init();

    while(1){
      time++;
      AP_BackgroundProcess();  // handle incoming SNP frames
      if(time>100000){
        time = 0;
        lDist = getLeftDistance();
        if(AP_GetNotifyCCCD(0)){
          AP_SendNotification(0);
        }
        rDist = getRightDistance();
        if(AP_GetNotifyCCCD(1)){
          AP_SendNotification(1);
        }
        fDist = getFrontDistance();
        if(AP_GetNotifyCCCD(2)){
          AP_SendNotification(2);
        }
        if(AP_GetNotifyCCCD(3)){
          AP_SendNotification(3);
        }
      }
    }
}

void PORT4_IRQHandler(void) {
    P4->IFG &= ~0xED;          // clear interrupt flag
    bump_trigger = 1;
    Motor_Stop();
}
