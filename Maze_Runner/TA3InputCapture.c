// TA3InputCapture.c
// Runs on MSP432
// Use Timer A3 in capture mode to request interrupts on rising
// edges of P10.4 (TA3CCP0) and P8.2 (TA3CCP2) and call user
// functions.
// Use Timer A3 in capture mode to request interrupts on rising
// edges of P10.4 (TA3CCP0) and P10.5 (TA3CCP1) and call user
// functions.
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

// external signal connected to P10.5 (TA3CCP1) (trigger on rising edge)
// external signal connected to P10.4 (TA3CCP0) (trigger on rising edge)

#include <stdint.h>
#include "msp.h"

void ta3dummy(uint16_t t){};       // dummy function
void ta3dummy2(void){}             // dummy function
void (*CaptureTask0)(uint16_t time) = ta3dummy; // user function
void (*CaptureTask1)(uint16_t time) = ta3dummy; // user function
void (*CaptureTask2)(void) = ta3dummy; // user function
void (*CaptureTask3)(void) = ta3dummy; // user function


//------------TimerA3Capture_Init01------------
// Initialize Timer A3 in edge time mode to request interrupts on
// the rising edges of P10.4 (TA3CCP0) and P10.5 (TA3CCP1).  The
// interrupt service routines acknowledge the interrupt and call
// a user function.
// Input: task0 is a pointer to a user function called when P10.4 (TA3CCP0) edge occurs
//              parameter is 16-bit up-counting timer value when P10.4 (TA3CCP0) edge occurred (units of 0.083 usec)
//        task1 is a pointer to a user function called when P10.5 (TA3CCP1) edge occurs
//              parameter is 16-bit up-counting timer value when P10.5 (TA3CCP1) edge occurred (units of 0.083 usec)
// Output: none
// Assumes: low-speed subsystem master clock is 12 MHz
void TimerA3Capture_Init01(void(*task0)(uint16_t time), void(*task1)(uint16_t time)){
    CaptureTask0 = task0;
    CaptureTask1 = task1;

    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30; //Select Timer Functionality for Tachometer Pins
    P10->DIR &= ~0x30; //Send Pins 10.4 and 10.5 to TA3 CCIS0A and TA3 CCIS1A respectively

    P5->SEL0 &= ~0x05;
    P5->SEL1 &= ~0x05; //Set pins 5.0 and 5.2 to GPIO
    P5->DIR &= ~0x05; //Inputs
    P5->REN &= ~0x05; //Disable pull-up/pull-down; There are external pull-ups for these

    TIMER_A3->CTL &= ~0x0030; //Stop the clock
    TIMER_A3->CTL = 0x0280; //Use 3MHz Clock; Prescale by 4
    TIMER_A3->EX0 = 0x0004; //Prescale by another 5. Uses 600kHz Clock. Period is 1.67us

    TIMER_A3->CCTL[0] = 0x4910; //Capture Mode, Rising Edge, looks at CCIS0A (Pin 10.4), Enable Interrupts
    TIMER_A3->CCTL[1] = 0x4910; //Capture Mode, Rising Edge, looks at CCIS1A (Pin 10.5), Enable Interrupts

    NVIC->IP[3] = (NVIC->IP[3] & ~0xE0E00000)|0x40400000; //All Timer A3 Submodules are Priority 2
    NVIC->ISER[0] |= 0x0000C000; //Enable all Timer A3 interrupts

    TIMER_A3->CTL |= 0x0024; //Reset and Start Timer in Continuous Mode
}

void TimerA3Capture_Init(void(*task0)(uint16_t time), void(*task1)(uint16_t time), void(*task2)(void), void(*task3)(void)){
    CaptureTask0 = task0;
    CaptureTask1 = task1;
    CaptureTask2 = task2;
    CaptureTask3 = task3;

    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30; //Select Timer Functionality for Tachometer Pins
    P10->DIR &= ~0x30; //Send Pins 10.4 and 10.5 to TA3 CCIS0A and TA3 CCIS1A respectively

    P5->SEL0 &= ~0x05;
    P5->SEL1 &= ~0x05; //Set pins 5.0 and 5.2 to GPIO
    P5->DIR &= ~0x05; //Inputs
    P5->REN &= ~0x05; //Disable pull-up/pull-down; There are external pull-ups for these

    TIMER_A3->CTL &= ~0x0030; //Stop the clock
    TIMER_A3->CTL = 0x0280; //Use 3MHz Clock; Prescale by 4
    TIMER_A3->EX0 = 0x0004; //Prescale by another 5. Uses 600kHz Clock. Period is 1.67us

    TIMER_A3->CCTL[0] = 0x4910; //Capture Mode, Rising Edge, looks at CCIS0A (Pin 10.4), Enable Interrupts
    TIMER_A3->CCTL[1] = 0x4910; //Capture Mode, Rising Edge, looks at CCIS1A (Pin 10.5), Enable Interrupts

    TIMER_A3->CCTL[2] = 0x0010; //Enable Interrupt; Compare Mode
    TIMER_A3->CCR[2] = 6000; // First Occurance at 10ms

    TIMER_A3->CCTL[3] = 0x0010; //Enable Interrupt; Compare Mode
    TIMER_A3->CCR[3] = 6000; // First Occurance at 10ms

    NVIC->IP[3] = (NVIC->IP[3] & ~0xE0E00000)|0x40400000; //All Timer A3 Submodules are Priority 2
    NVIC->ISER[0] |= 0x0000C000; //Enable all Timer A3 interrupts

    TIMER_A3->CTL |= 0x0024; //Reset and Start Timer in Continuous Mode
}


void TA3_0_IRQHandler(void)
{
    TIMER_A3->CCTL[0] &= ~0x0001; //Clear Interrupt Flag and overflow
    (*CaptureTask0)(TIMER_A3->CCR[0]); // execute user task
}

void TA3_N_IRQHandler(void)
{
    switch(TIMER_A3->IV)
    {
        case 2: //CCR1
            TIMER_A3->CCTL[1] &= ~0x0001; //Clear Interrupt Flag and overflow
            (*CaptureTask1)(TIMER_A3->CCR[1]); // execute user task
            break;

        case 4: //CCR2
            TIMER_A3->CCTL[2] &= ~0x0001; //Clear Interrupt Flag and overflow
            TIMER_A3->CCR[2] = (TIMER_A3->CCR[2] + 6000) % 0xFFFF; //Set next occurrence to happen in 10ms
            (*CaptureTask2)();
            break;

        case 6: //CCR3
            TIMER_A3->CCTL[3] &= ~0x0001; //Clear Interrupt Flag and overflow
            TIMER_A3->CCR[3] = (TIMER_A3->CCR[3] + 6000) % 0xFFFF; //Set next occurrence to happen in 10ms
            (*CaptureTask3)();
            break;

        default:
            break;
    }
}

// old robot code

//------------TimerA3Capture_Init02------old robot version------
// Initialize Timer A3 in edge time mode to request interrupts on
// the rising edges of P10.4 (TA3CCP0) and P8.2 (TA3CCP2).  The
// interrupt service routines acknowledge the interrupt and call
// a user function.
// Input: task0 is a pointer to a user function called when P10.4 (TA3CCP0) edge occurs
//              parameter is 16-bit up-counting timer value when P10.4 (TA3CCP0) edge occurred (units of 0.083 usec)
//        task2 is a pointer to a user function called when P8.2 (TA3CCP2) edge occurs
//              parameter is 16-bit up-counting timer value when P8.2 (TA3CCP2) edge occurred (units of 0.083 usec)
// Output: none
// Assumes: low-speed subsystem master clock is 12 MHz
void TimerA3Capture_Init02(void(*task0)(uint16_t time), void(*task2)(uint16_t time)){
// old robot code
}
