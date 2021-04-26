/*
 * Jacki_PID.c
 *
 *  Created on: Apr 16, 2021
 *      Author: NHG11
 */



// Lab17_Control.c
// Runs on MSP432
// Implementation of the control system.
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
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/TimerA1.h"
#include "../inc/LPF.h"
#include "../inc/SysTickInts.h"
#include "../inc/Tachometer.h"
#include "../inc/Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Jacki_Tachometer.h"
#include "../inc/EUSCIA0.h"
#include "../inc/Ultrasound.h"
#include "../inc/AP.h"

//******************************************************
// Distance to wall proportional control
// Incremental speed control
// Integral control, Line follower


#define RPMNOMINAL 50
#define RED       0x01
#define GREEN     0x02
#define BLUE      0x04

extern uint8_t Go;


int32_t deltaT = 10;     //time between tach readings (ms)

int32_t Xstar0 = 100;      //desired speed (rpm)
int32_t Ki0 = 80;          //integral controller coefficient
int32_t UR, Error0, Xprime0;
int32_t Xstar1 = 100;
int32_t Ki1 = 80;
int32_t UL, Error1, Xprime1;
int j = 0, turn = 0;  //buffer is value for manually controlled turns
void Wheel_Controller(void) { //CONTROLS SPEED OF BOTH WHEELS

    if( turn != 0) {
        return;
    }

    Xprime0 = getSpeed0();       //measured speed
    Error0 = Xstar0 - Xprime0;
    UR += (Ki0*Error0*deltaT)/(1000);      //integral controller

    Xprime1 = getSpeed1();
    Error1 = Xstar1 - Xprime1;
    UL += (Ki1*Error1*deltaT)/(1000);
    j++;

    if(UL < 0) {UL = 0;}
    if(UR < 0) {UR = 0;}
    if(UL > 14998) {UL = 14998;}
    if(UR > 14998) {UR = 14998;}
    Motor_Forward(UL, UR);
}

#define sideThreshold 300
#define frontThreshold 150
int32_t Desired_Position = 0;   // (distance from left wall) - (distance from right wall) in cm
int32_t Kp = 100;
int32_t Ki = 0;
int32_t Kd = 10;
int32_t Up, Ui, Ud, U, Error, last_error = -1000, Xprime, left_distance, right_distance, front_distance, initial_edge, current_edge;
int k = 0;
int turn_threshold = 340;

void Position_Controller(void) {
    if(!Go) {
        turn = 7;
        Motor_Stop();
        return;
    }
    if( turn != 0) {
        switch(turn)
        {
            case 1: //turning right
                current_edge = getEdges1();
                turn_threshold = 185;
                P2->OUT = (P2->OUT&0xF8)|(BLUE+RED);
                break;
            case 2: //turning left
                current_edge = getEdges0();
                turn_threshold = 185;
                P2->OUT = (P2->OUT&0xF8)|BLUE;
                break;
            case 3: //turning 180
                current_edge = getEdges0();
                turn_threshold = 370;
                P2->OUT = (P2->OUT&0xF8)|GREEN;
                break;
            case 4: //going straight
                current_edge = getEdges1();
                turn_threshold = 380;
                P2->OUT = (P2->OUT&0xF8)|(GREEN+BLUE);
                break;
            case 5: //going straight
                current_edge = getEdges0();
                turn_threshold = 380;
                P2->OUT = (P2->OUT&0xF8)|(GREEN+RED);
                break;
            case 6: //going straight before right turn
                current_edge = getEdges1();
                turn_threshold = 180;
                P2->OUT = (P2->OUT&0xF8)|RED;
                break;
            case 7: // Go Disabled (Bluetooth)
                break;
            case 8: // going straight before left turn
                current_edge = getEdges0();
                turn_threshold = 100;
                P2->OUT = (P2->OUT&0xF8)|RED;
                break;
        }

        if( current_edge - initial_edge >= turn_threshold) {
            if( turn == 1) {   //move forward after a left or right turn
                initial_edge = current_edge;
                turn = 4;
                Motor_Forward(3500,3500);
            }
            else if( turn == 6) {
                initial_edge = current_edge;
                turn = 1;
                Motor_Left(3500, 3500);
            }
            else if( turn == 8) {
                initial_edge = current_edge;
                turn = 2;
                Motor_Right(3500, 3500);
            }
            else if( turn == 2 || turn == 3) {
                initial_edge = current_edge;
                turn = 5;
                Motor_Forward(3500,3500);
            }
            else if( turn == 4 || turn == 5) {
                turn = 0;
                Ui = 0;
                last_error = -1000;
                UR = 3500;
                UL = 3500;
            }
        }
        return;
    }

    //get current filtered measurements from ultrasonics
    left_distance = getLeftDistance();
    right_distance = getRightDistance();
    front_distance = getFrontDistance();
    P2->OUT = (P2->OUT&0xF8);

    if( right_distance > sideThreshold) {
        turn = 6;
        //Motor_Stop();
        initial_edge = getEdges1();
        Motor_Forward(3500,3500);
    }
    else if( front_distance < frontThreshold) {  //check if there's anything in front of us
        if ( left_distance > sideThreshold) { //only left path is open
            turn = 8;
            initial_edge = getEdges0();
            Motor_Forward(3500,3500);
        }
        else {      //at a dead end
            turn = 3;
            initial_edge = getEdges0();
            Motor_Right(3500,3500);   //turn 180 degrees in place
        }
    }
    else {  //straight away
        Error = right_distance - 80;    //hug the right wall at distance of 20cm

        Up = (Kp*Error)/1000;    //proportional
        Ui += (Ki*Error*deltaT)/1000;   //integral
        Ud = Kd*(Error - last_error)*1000/deltaT; //derivative
        if( last_error == -1000) {
            Ud = 0;
        }
        U = Up + Ui + Ud;    //PID controller
        last_error = Error;

        k++;
        if(U>=RPMNOMINAL) {
            U = RPMNOMINAL;
        }
        if(U<= -1*RPMNOMINAL) {
            U = -1*RPMNOMINAL;
        }
        Xstar0 = RPMNOMINAL - U;    //set values for right and left wheel in rpm
        Xstar1 = RPMNOMINAL + U;
    }
}
