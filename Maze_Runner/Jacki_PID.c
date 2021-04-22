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

//******************************************************
// Distance to wall proportional control
// Incremental speed control
// Integral control, Line follower


#define RPMNOMINAL 50


int32_t deltaT = 10;     //time between tach readings (ms)

int32_t Xstar0 = 100;      //desired speed (rpm)
int32_t Ki0 = 50;          //integral controller coefficient
int32_t UR, Error0, Xprime0;
int32_t Xstar1 = 100;
int32_t Ki1 = 50;
int32_t UL, Error1, Xprime1;
int j = 0;
void Wheel_Controller(void) { //CONTROLS SPEED OF BOTH WHEELS
    Xprime0 = getSpeed0();       //measured speed
    Error0 = Xstar0 - Xprime0;
    UR += (Ki0*Error0*deltaT)/(1000);      //integral controller

    Xprime1 = getSpeed1();
    Error1 = Xstar1 - Xprime1;
    UL += (Ki1*Error1*deltaT)/(1000);

//    if( j % 100 == 0) {
//        EUSCIA0_OutString("UR = ");
//        EUSCIA0_OutSDec(UR);
//        EUSCIA0_OutString("\nUL = ");
//        EUSCIA0_OutSDec(UL);
//        EUSCIA0_OutString("\n");
//    }
    j++;
    Motor_Forward(UL, UR);
}


int32_t Desired_Position = 0;   // (distance from left wall) - (distance from right wall) in cm
int32_t Kp = 75;
int32_t Ki = 10;
int32_t Up, Ui, U, Error, Xprime, left_distance, right_distance, front_distance;
int k = 0;
void Position_Controller(void) {
    //get current filtered measurements from ultrasonics
    left_distance = getLeftDistance();
    right_distance = getRightDistance();
    front_distance = getFrontDistance();

    Error = right_distance - 10;    //hug the right wall at distance of 10cm

    if( front_distance < 20) {  //check if there's anything in front of us
        if ( left_distance > 30) { //only left path is open
            //turn left
            Error = left_distance - right_distance; //redefine distance so that we turn left (done by setting error to the distance from center of wall)
        }
        else {      //at a dead end
            //turn right until we see a right path
            Xstar0 = 0;
            Xstar1 = 75;
            Ui = 0;  //reset the integral controller
        }
    }
    else {  //right path open or straight away

        Up = (Kp*Error*deltaT)/1000;    //proportional
        Ui += (Ki*Error*deltaT)/1000;   //integral
        U = Up + Ui;    //PI controller

//        if ( k % 100 == 0) {
//            EUSCIA0_OutString("U = ");
//            EUSCIA0_OutSDec(U);
//            EUSCIA0_OutString("\n");
//        }
        k++;
        if(U>=50) {
            U = 50;
        }
        if(U<=-50) {
            U = -50;
        }
        Xstar0 = RPMNOMINAL - U;
        Xstar1 = RPMNOMINAL + U;
    }
}

void old_Position_Controller(void) {
    //get current filtered measurements from ultrasonics
    left_distance = getLeftDistance();
    right_distance = getRightDistance();

    Xprime = left_distance - right_distance;
    Error = Desired_Position - Xprime;

    Up = (Kp*Error*deltaT)/1000;    //proportional
    Ui += (Ki*Error*deltaT)/1000;   //integral
    U = Up + Ui;    //PI controller

//    if ( k % 100 == 0) {
//        EUSCIA0_OutString("U = ");
//        EUSCIA0_OutSDec(U);
//        EUSCIA0_OutString("\n");
//    }
    k++;
    if(U>=50) {
        U = 50;
    }
    if(U<=-50) {
        U = -50;
    }
    Xstar0 = RPMNOMINAL - U;
    Xstar1 = RPMNOMINAL + U;
}
