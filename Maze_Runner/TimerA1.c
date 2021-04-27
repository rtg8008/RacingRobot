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
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"

void ta1dummy(void){};      // dummy function
uint8_t ta1dummy2(void){return 0;};  //dummy function
void (*TimerA1Task0)(void) = ta1dummy; // user function
uint8_t (*TimerA1Task1)(void) = ta1dummy2; // user function

uint8_t light;

// ***************** TimerA1_Init ****************
// Activate Timer A1 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (24/SMCLK), 16 bits
// Outputs: none
// With SMCLK 12 MHz, period has units 2us
void TimerA1_Init(void(*task0)(void), uint8_t(*task1)(void)){
    TimerA1Task0 = task0;
    TimerA1Task1 = task1;

    TIMER_A1->CTL = 0x0280; //Configure to use 3Mz clock; Stop Clock; No Overarching interrupts
    TIMER_A1->EX0 = 0x0007; // Further Prescale -> minimize clock to 375kHz (least switching to achieve 1ms value by integer number); 375 cycles/1ms

    //Light Sensor 1
    TIMER_A1->CCTL[2] = 0x0010; //Compare mode; enable interrupt
    TIMER_A1->CCR[2] = 37500; //offset of 100 ms

    //Light Sensor 2
    TIMER_A1->CCTL[3] = 0x0010; //Compare mode; enable interrupt
    TIMER_A1->CCR[3] = 37875; //offset of 101 ms (1ms from Light Sensor 1)

    //NVIC Stuff
    NVIC->IP[2] = (NVIC->IP[2] & ~0xE0E00000)|0x40000000; //Priority 2
    NVIC->ISER[0] |= 0x00000C00; //Enable all Timer A1 interrupts

    //Reset & Start Timer in Continuous Mode
    TIMER_A1->CTL |= 0x0024;
}

void TA1_0_IRQHandler(void)
{

}

void TA1_N_IRQHandler(void)
{
    switch(TIMER_A1->IV)
    {
        case 4: //CCR2; Light Sensor 1
            TIMER_A1->CCTL[2] &= ~0x0001; //Clear Interrupt Flag
            TIMER_A1->CCR[2] = (TIMER_A1->CCR[2] + 37500) % 0xFFFF; //Set next occurrence to happen in 100ms
            (*TimerA1Task0)();;
            break;
        case 6: //CCR3; Light Sensor 2
            TIMER_A1->CCTL[3] &= ~0x0001; //Clear Interrupt Flag
            TIMER_A1->CCR[3] = (TIMER_A1->CCR[3] + 37500) % 0xFFFF; //Set next occurrence to happen in 100ms
            light = (*TimerA1Task1)();;

            if( light < 0xFF) {    //over top of the white area (solved state)
                while(1) {
                    DisableInterrupts();
                    Motor_Stop();
                    P1->OUT ^= 0x01;    //toggle red LED
                    Clock_Delay1ms(500);
                }
            }
            break;
        default:
            break;
    }
}


// ------------TimerA1_Stop------------
// Deactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void TimerA1_Stop(void){
    // write this as part of Lab 13

 
}

