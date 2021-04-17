// TimerA1.c
// Runs on MSP432
// Use Timer A1 in periodic mode to request interrupts at a particular
// period.
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

void (*TimerA1Task)(void);   // user function

// ***************** TimerA1_Init ****************
// Activate Timer A1 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (24/SMCLK), 16 bits
// Outputs: none
// With SMCLK 12 MHz, period has units 2us
void TimerA1_Init(void(*task)(void)){
    TimerA1Task = task;

    TIMER_A1->CTL = 0x0280; //Configure to use 3Mz clock; Stop Clock; No Overarching interrupts
    TIMER_A1->EX0 = 0x0007; // Further Prescale -> minimize clock to 375kHz (least switching to achieve 1ms value by integer number); 375 cycles/1ms

    //Evaluate Data
    TIMER_A1->CCTL[0] = 0x0010; //Compare mode; enable interrupt
    TIMER_A1->CCR[0] = 3750; //Every 10ms with no offset (will add onto this value each time in handler)

    //NVIC Stuff
    NVIC->IP[2] = (NVIC->IP[2] & ~0xE0E00000)|0x00000000; //Priority 0
    NVIC->ISER[0] |= 0x00000C00; //Enable all Timer A1 interrupts

    //Reset & Start Timer in Continuous Mode
    TIMER_A1->CTL |= 0x0024;
}

void TA1_0_IRQHandler(void)
{
    TIMER_A1->CCTL[0] &= ~0x0001; //Clear Interrupt Flag
    TIMER_A1->CCR[0] = (TIMER_A1->CCR[0] + 3750) % 0xFFFF; //Set next occurrence to happen in 10ms
    (*TimerA1Task)();   // execute user task
}

void TA1_N_IRQHandler(void)
{
    /*
    switch(TIMER_A1->IV)
    {
        default:
            break;
    }
    */
}


// ------------TimerA1_Stop------------
// Deactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void TimerA1_Stop(void){
    // write this as part of Lab 13

 
}

