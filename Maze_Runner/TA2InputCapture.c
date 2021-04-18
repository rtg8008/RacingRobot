// TA2InputCapture.c
// Runs on MSP432
// Use Timer A2 in capture mode to request interrupts on both
// edges of P5.6 (TA2CCP1) and call a user function.
// Daniel Valvano
// April 18, 2017
// Warning: CC3100 uses Timer A2

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

// external signal connected to P5.6 (TA2CCP1) (trigger on both edges)

#include <stdint.h>
#include "../inc/CortexM.h"
#include "msp.h"

#define SENSOR_0 0
#define SENSOR_1 1
#define SENSOR_2 2

void ta2dummy(uint16_t t, int sensor){};       // dummy function
void (*CaptureTaskA2)(uint16_t time, int sensor) = ta2dummy;// user function

//------------TimerA2Capture_Init------------
// Initialize Timer A2 in edge time mode to request interrupts on
// both edges of P5.6 (TA2CCP1).  The interrupt service routine
// acknowledges the interrupt and calls a user function.
// Input: task is a pointer to a user function called when edge occurs
//             parameter is 16-bit up-counting timer value when edge occurred (units of 0.083 usec)
// Output: none
void TimerA2Capture_Init(void(*task)(uint16_t time, int sensor)){
  CaptureTaskA2 = task;             // user function
  // initialize P5.6&7 and make it both edges (P5.6&7 TA2CCP1)
  P5->SEL0 |= 0xC0;
  P5->SEL1 &= ~0xC0;               // configure P5.6&7 as TA2CCP1
  P5->DIR &= ~0xC0;                // make P5.6&7 input
  // initialize P6.6 and make it both edges
  P6->SEL0 |= 0x40;
  P6->SEL1 &= ~0x40;               // configure P6.6 as TA2CCP1
  P6->DIR &= ~0x40;                // make P6.6 input

  TIMER_A2->CTL &= ~0x0030;        // halt Timer A2

  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=00,       input clock divider /1
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           interrupt disable
  // bit0=0,           clear interrupt pending
  TIMER_A2->CTL = 0x0200; //Use 12MHz Clock; No Prescale
  TIMER_A2->EX0 = 0x0003; //Prescale by 4. Uses 3MHz Clock. Period is 333.33ns

  // bits15-14=11,     capture on both edges
  // bits13-12=00,     capture/compare input on CCI2A
  // bit11=1,          synchronous capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=1,           capture mode
  // bits7-5=XXX,      output mode
  // bit4=1,           enable capture/compare interrupt
  // bit3=X,           read capture/compare input from here
  // bit2=X,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending

  TIMER_A2->CCTL[1] = 0xC910;
  TIMER_A2->CCTL[2] = 0xC910;
  TIMER_A2->CCTL[3] = 0xC910;

  TIMER_A2->CCTL[0] = 0x0010; //Enable Interrupt; Compare Mode
  TIMER_A2->CCR[0] = 300; // First Occurance at 100us

  TIMER_A2->CCTL[4] = 0x0010; //Enable Interrupt; Compare Mode
  TIMER_A2->CCR[4] = 330; // First Occurance at 110us

  NVIC->IP[3] = (NVIC->IP[3]&0xFFFF000F)|0x00004040; // priority 2
// interrupts enabled in the main program after all devices initialized
  NVIC->ISER[0] = 0x00003000;      // enable interrupt 12 and 13 in NVIC
  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=00,       input clock divider /1
  // bits5-4=10,       continuous count up mode
  // bit3=X,           reserved
  // bit2=1,           set this bit to clear
  // bit1=0,           interrupt disable (no interrupt on rollover)
  // bit0=0,           clear interrupt pending
  TIMER_A2->CTL |= 0x0024;         // reset and start Timer A2 in continuous up mode
}

void TA2_0_IRQHandler(void) {
    TIMER_A2->CCTL[0] &= ~0x0001; //Clear Interrupt Flag and overflow
    TIMER_A2->CCR[0] = (TIMER_A2->CCR[0] + 30000) % 0xFFFF; //Set next occurrence to happen in 10ms
    P6->OUT |= 0x20;
}

void TA2_N_IRQHandler(void) {

  switch(TIMER_A2->IV) {

      case 0x02: //CCR1
          TIMER_A2->CCTL[1] &= ~0x0001;    // acknowledge capture/compare interrupt 1
          (*CaptureTaskA2)(TIMER_A2->CCR[1], SENSOR_0); // execute user task
          break;

      case 0x04: //CCR2
          TIMER_A2->CCTL[2] &= ~0x0001;    // acknowledge capture/compare interrupt 2
          (*CaptureTaskA2)(TIMER_A2->CCR[2], SENSOR_1); // execute user task
          break;

      case 0x06: //CCR3
          TIMER_A2->CCTL[3] &= ~0x0001;    // acknowledge capture/compare interrupt 3
          (*CaptureTaskA2)(TIMER_A2->CCR[3], SENSOR_2); // execute user task
          break;

      case 0x08: //CCR4
          TIMER_A2->CCTL[4] &= ~0x0001; //Clear Interrupt Flag and overflow
          TIMER_A2->CCR[4] = (TIMER_A2->CCR[4] + 30000) % 0xFFFF; //Set next occurrence to happen in 10ms
          P6->OUT &= ~0x20;
          break;

      default:
          break;
  }
}
