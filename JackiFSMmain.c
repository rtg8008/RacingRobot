//*****************************************************************************
//
// Jacki FSM test main
// MSP432 with Jacki
// Daniel and Jonathan Valvano
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
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/SysTick.h"
#include "../inc/TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
#include "../inc/FlashProgram.h"

#define DEBUG_ARRAY_LEN 100
#define MAX_DUTY 5000     //7500

uint32_t *debugArr;
uint16_t debugIdx;
uint32_t flashAddr;
uint8_t light;
uint8_t bumpers;
int16_t position;
int16_t last = 1;       //initialize last to be different from position
uint8_t bump_trigger;
uint8_t state;


void main(void){
    Clock_Init48MHz();
    SysTick_Init();
    Motor_Init();
    Debug_Init();
    Debug_FlashInit();
    TimerA1_Init();
    Bump_Edge_Init();
    state = 1;
    EnableInterrupts();
    while(1){
        if( last == position) {
            continue;
        }

        last = position;
        state = getNextState();
        switch(state)
        {
            case 0:
                SysTick_Wait(48000*500);
                state = getNextState();

                if( state == 0) {
                    Motor_Backward(MAX_DUTY, MAX_DUTY);
                    DisableInterrupts();
                    SysTick_Wait(48000*2);
                    EnableInterrupts();
                }
                break;
            case 1: //toward center of line
                Motor_Forward(MAX_DUTY, MAX_DUTY);
                break;
            case 2: //a little off to the left; positive position
                Motor_Forward(MAX_DUTY, MAX_DUTY*7/8);
                break;
            case 3: //medium off to the left
                Motor_Forward(MAX_DUTY, MAX_DUTY*3/4);
                break;
            case 4: //far off to the left
                Motor_Right(MAX_DUTY, MAX_DUTY*5/6);
                break;
            case 5: //a little off to the right; negative position
                Motor_Forward(MAX_DUTY*7/8, MAX_DUTY);
                break;
            case 6: //medium off the right
                Motor_Forward(MAX_DUTY*3/4, MAX_DUTY);
                break;
            case 7: //far off to the right
                Motor_Left(MAX_DUTY*5/6, MAX_DUTY);
                break;
            default:
                Motor_Stop();
                break;
        }
    }

}

void Debug_Init(void){
    debugIdx = 0;
    debugArr = (uint32_t*)malloc(DEBUG_ARRAY_LEN/2*sizeof(uint32_t));
}

void Debug_Dump(uint8_t x, uint8_t y){
    //ARM is Little Endian --> make all more recent data have higher address values
    if(debugIdx >= DEBUG_ARRAY_LEN) // this should never happen, but still test for it
    {
        return; //I choose to not loop back around as it would go against the assumption that higher addresses represent more recent data.
    }
    uint32_t temp;
    if(!(debugIdx%2))
    {
        temp = 0xFFFF0000; //when writing to new array location, set top 2 bytes same value as rest of flash
        temp |= (x << 8);
        temp |= y;
    }
    else
    {
        temp = debugArr[debugIdx/2];
        temp &= 0x0000FFFF; // clear top 2 bytes if writing to location that already has some data
        temp |= (x << 24);
        temp |= (y << 16);
    }
    debugArr[debugIdx/2] = temp;
    debugIdx++;
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

void Debug_FlashRecord(void){
    uint16_t count = debugIdx/2;
    if(flashAddr>=0x0003FFFF)
    {
        return;
    }
    count = Flash_WriteArray(debugArr,flashAddr,count);
    flashAddr += 4*count;
    debugIdx = 0;
}
void TimerA1_Init(void){
    //Order here will determine Priority; 0 will have the highest priority (and is more urgent than bumpers)
    //While increasing number will have decreasing urgency (1 will be handled before 2)
    //Values 1 and higher will be handled with less urgency than the bumpers (at Priority 2)

    TIMER_A1->CTL = 0x0280; //Configure to use 3Mz clock; Stop Clock; No Overarching interrupts
    TIMER_A1->EX0 = 0x0007; // Further Prescale -> minimize clock to 375kHz (least switching to achieve 1ms value by integer number); 375 cycles/1ms

    //Debug to ROM
    TIMER_A1->CCTL[0] = 0x0010; //Compare mode; enable interrupt
    TIMER_A1->CCR[0] = 37500; //Every 100ms with no offset (will add onto this value each time in handler)

    //Debug to RAM
    TIMER_A1->CCTL[1] = 0x0010; //Compare mode; enable interrupt
    TIMER_A1->CCR[1] = 4500; //Every 10ms with offset of 2 ms

    //Light Sensor 1
    TIMER_A1->CCTL[2] = 0x0010; //Compare mode; enable interrupt
    TIMER_A1->CCR[2] = 5250; //Every 10ms with offset of 4 ms

    //Light Sensor 2
    TIMER_A1->CCTL[3] = 0x0010; //Compare mode; enable interrupt
    TIMER_A1->CCR[3] = 5625; //Every 10ms with offset of 5 ms (1ms from Light Sensor 1)

    //NVIC Stuff
    NVIC->IP[2] = (NVIC->IP[2] & ~0xE0E00000)|0x40000000; //Priority 0 for ROM; Priority 2 for everything else
    NVIC->ISER[0] |= 0x00000C00; //Enable all Timer A1 interrupts

    //Reset & Start Timer in Continuous Mode
    TIMER_A1->CTL |= 0x0024;
}

void TA1_0_IRQHandler(void)
{
    TIMER_A1->CCTL[0] &= ~0x0001; //Clear Interrupt Flag
    TIMER_A1->CCR[0] = (TIMER_A1->CCR[0] + 37500) % 0xFFFF; //Set next occurrence to happen in 100ms
    Debug_FlashRecord();
}

void TA1_N_IRQHandler(void)
{
    switch(TIMER_A1->IV)
    {
        case 0:
            break;
        case 2: //CCR1; Debug to RAM
            TIMER_A1->CCTL[1] &= ~0x0001; //Clear Interrupt Flag
            TIMER_A1->CCR[1] = (TIMER_A1->CCR[1] + 3750) % 0xFFFF; //Set next occurrence to happen in 10ms
            bumpers = Bump_Read();
            Debug_Dump(light,bumpers);
            break;
        case 4: //CCR2; Light Sensor 1
            TIMER_A1->CCTL[2] &= ~0x0001; //Clear Interrupt Flag
            TIMER_A1->CCR[2] = (TIMER_A1->CCR[2] + 1875) % 0xFFFF; //Set next occurrence to happen in 5ms
            Reflectance_Start();
            break;
        case 6: //CCR3; Light Sensor 2
            TIMER_A1->CCTL[3] &= ~0x0001; //Clear Interrupt Flag
            TIMER_A1->CCR[3] = (TIMER_A1->CCR[3] + 1875) % 0xFFFF; //Set next occurrence to happen in 5ms
            light = Reflectance_End();
            position = Reflectance_Position(light);
            break;
        default:
            break;
    }
}

void PORT4_IRQHandler(void) {
    P4->IFG &= ~0xED;          // clear interrupt flag
    bumpers = Bump_Read();
    bump_trigger = 1;
    Motor_Stop();
    Motor_Backward(MAX_DUTY/2,MAX_DUTY/2);
    SysTick_Wait(500*48000);
    Motor_Forward(MAX_DUTY/2, MAX_DUTY/2);
}

uint16_t map(int16_t x) {
    x = abs(x);
    return 89*x;
}

uint8_t getNextState()
{
    uint8_t state;

    if( (position > 9000)||(light == 0xFF) ) {
        state = 0;           //lost
    }
    else if( position >= -10 && position <= 10) {
        state = 1;           //toward center of line
    }
    else if( position > 10 && position < 200) {
        state = 2;           //a little to the left
    }
    else if( position >= 200 && position <= 250) {
        state = 3;           //medium off to the left
    }
    else if( position > 250) {
        state = 4;           //far off to the left
    }
    else if( position < -10 && position > -200) {
        state = 5;            //a little to the right
    }
    else if( position <= -200 && position >= -250) {
        state = 6;            //medium off to the right
    }
    else if( position < -250) {
        state = 7;            //far off to the right
    }
    return state;
}
