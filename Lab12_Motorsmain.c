// Lab12_Motorsmain.c
// Runs on MSP432
// Solution to Motors lab
// Daniel and Jonathan Valvano
// December 17, 2018

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



//**************RSLK1.1***************************
// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\MotorSimple.h"
#include "..\inc\CortexM.h"
#include "..\inc\Reflectance.h"

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
    //Debug_FlashRecord();
}

uint8_t sensor = 0;
void TA1_N_IRQHandler(void)
{
    switch(TIMER_A1->IV)
    {
        case 0:
            break;
        case 2: //CCR1; Debug to RAM
            TIMER_A1->CCTL[1] &= ~0x0001; //Clear Interrupt Flag
            TIMER_A1->CCR[1] = (TIMER_A1->CCR[1] + 3750) % 0xFFFF; //Set next occurrence to happen in 10ms
            //Debug_Dump(light,bumper);
            break;
        case 4: //CCR2; Light Sensor 1
            TIMER_A1->CCTL[2] &= ~0x0001; //Clear Interrupt Flag
            TIMER_A1->CCR[2] = (TIMER_A1->CCR[2] + 3750) % 0xFFFF; //Set next occurrence to happen in 10ms
            Reflectance_Start();
            break;
        case 6: //CCR3; Light Sensor 2
            TIMER_A1->CCTL[3] &= ~0x0001; //Clear Interrupt Flag
            TIMER_A1->CCR[3] = (TIMER_A1->CCR[3] + 3750) % 0xFFFF; //Set next occurrence to happen in 10ms
            sensor = Reflectance_End();         //after sensor cap dissipates, collect data
            break;
        default:
            break;
    }
}

// Driver test
void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

int main(void) {
    Clock_Init48MHz();
    LaunchPad_Init();                   // built-in switches and LEDs
    SysTick_Init();                     // initialize systick timer
    Reflectance_Init();
    Bump_Edge_Init();                   // edge triggered bump switches
    Motor_InitSimple();                 // your function
    TimerA1_Init();                     // timer initilization

    EnableInterrupts();
    while(1){
        /*
      Pause();
      Motor_ForwardSimple(5000,2000);  // your function
      Pause();
      Motor_BackwardSimple(5000,2000); // your function
      Pause();
      Motor_LeftSimple(5000,2000);     // your function
      Pause();
      Motor_RightSimple(5000,2000);    // your function
      */
    }
}

int systick_main(void){
  Clock_Init48MHz();
  uint32_t cycles = 48/100*1000000;   // 48/<frequency(Hz)>*1000000
  SysTickInt_Init(cycles,3);        // set up SysTick for 8 Hz interrupts
  LaunchPad_Init();                 // built-in switches and LEDs
  Bump_Init();                      // bump switches
  Motor_InitSimple();               // your function

  EnableInterrupts();
  while(1){
    Pause();
    Motor_ForwardSimple(5000,2000);  // your function
    Pause();
    Motor_BackwardSimple(5000,2000); // your function
    Pause();
    Motor_LeftSimple(5000,2000);     // your function
    Pause();
    Motor_RightSimple(5000,2000);    // your function
  }
}

// Voltage current and speed as a function of duty cycle
int not_needed(void){ //Program12_2(void){
  uint16_t duty;
  Clock_Init48MHz();
  LaunchPad_Init();   // built-in switches and LEDs
  Bump_Init();        // bump switches
  Motor_InitSimple(); // initialization
  while(1){
    for(duty=2000; duty<=8000; duty=duty+2000){
      Motor_StopSimple();   // measure current
      Pause();
      Motor_LeftSimple(duty,6000);  // measure current
    }
  }
}

int Program12_3(void){
  Clock_Init48MHz();
  LaunchPad_Init();   // built-in switches and LEDs
  Bump_Init();        // bump switches
  Motor_InitSimple(); // initialization
  while(1){
    Pause();
    Motor_ForwardSimple(9900,15000); // max speed 15 s
  }
}

// does the robot move straight?
int Program12_4(void){ // Program12_4, RSLK version 1.1
  Clock_Init48MHz();
  LaunchPad_Init();   // built-in switches and LEDs
  Bump_Init();        // bump switches
  Motor_InitSimple(); // initialization
  while(1){
  //  Pause(); // start on SW1 or SW2
    LaunchPad_Output(0x02);
    Motor_ForwardSimple(5000,350);  // 3.5 seconds and stop
    LaunchPad_Output(0x00);
    Motor_StopSimple(); Clock_Delay1ms(500);
    LaunchPad_Output(0x01);
    Motor_BackwardSimple(3000,200); // reverse 2 sec
    LaunchPad_Output(0x03);
    Motor_LeftSimple(3000,200);     // right turn 2 sec
    if(Bump_Read()){
      LaunchPad_Output(0x01);
      Motor_BackwardSimple(3000,100);// reverse 1 sec
      LaunchPad_Output(0x03);
      Motor_LeftSimple(3000,200);   // right turn 2 sec
    }
  }
}


