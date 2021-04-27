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
#include "../inc/LPF.h"
#include "../inc/Tachometer.h"
#include "../inc/Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Jacki_Tachometer.h"
#include "../inc/EUSCIA0.h"
#include "../inc/Ultrasound.h"
#include "../inc/AP.h"

//******************************************************


#define RPMNOMINAL 50
#define SIDETHRESHOLD 300   //mm
#define FRONTTHRESHOLD 150  //mm
#define IDEALDISTANCE 80    //mm
#define DUTYNOMINAL 3500
#define RED       0x01
#define GREEN     0x02
#define BLUE      0x04

extern uint8_t Go;

void Wheel_Controller(void);
void Position_Controller(void);
void Turn(void);


int32_t deltaT = 10;     //time between tach readings (ms)
int32_t Xstar0 = 100;      //desired speed (rpm)
int32_t Ki0 = 80;          //integral controller coefficient
int32_t UR, Error0, Xprime0;
int32_t Xstar1 = 100;
int32_t Ki1 = 80;
int32_t UL, Error1, Xprime1;
int j = 0;
int turn = 0;  //turn is value for manually controlled turns

/* Controls duty cycle of wheels to achieve a desired rpm value using a feedback loop */
void Wheel_Controller(void) {

    if( turn != 0) {    //do nothing if performing a manual move
        return;
    }

    Xprime0 = getSpeed0();                 //measured speed
    Error0 = Xstar0 - Xprime0;
    UR += (Ki0*Error0*deltaT)/(1000);      //integral controller

    Xprime1 = getSpeed1();
    Error1 = Xstar1 - Xprime1;
    UL += (Ki1*Error1*deltaT)/(1000);
    j++;

    //check that duty cycle is limited within reason
    if(UL < 0) {UL = 0;}
    if(UR < 0) {UR = 0;}
    if(UL > 14998) {UL = 14998;}
    if(UR > 14998) {UR = 14998;}

    Motor_Forward(UL, UR);  //final output to motors
}


int32_t Kp = 100;   //proportional coefficient
int32_t Ki = 0;     //integral coefficient
int32_t Kd = 10;    //derivative coefficient
int32_t Up, Ui, Ud, U, Error, last_error = -1000, Xprime, left_distance, right_distance, front_distance, initial_edge, current_edge;
int k = 0;
int turn_threshold;   //amount of tachometer edge readings needed to complete move

/* Main PID and manual move controller to solve the maze */
void Position_Controller(void) {
    if(!Go) {
        turn = 8;
        Motor_Stop();   //bluetooth command has been called to stop robot
        return;
    }
    if( turn != 0) {
        Turn(); //manual turn occurring, and skipping over position PID
        return;
    }

    //get current filtered measurements from ultrasonics
    left_distance = getLeftDistance();
    right_distance = getRightDistance();
    front_distance = getFrontDistance();
    P2->OUT = (P2->OUT&0xF8);

    if( right_distance > SIDETHRESHOLD) {   //always turn right if possible
        turn = 1;
        initial_edge = getEdges1();
        Motor_Forward(DUTYNOMINAL,DUTYNOMINAL);
    }
    else if( front_distance < FRONTTHRESHOLD) {  //check if there's anything in front of us
        if ( left_distance > SIDETHRESHOLD) { //only left path is open
            turn = 2;
            initial_edge = getEdges0();
            Motor_Forward(DUTYNOMINAL,DUTYNOMINAL);
        }
        else {      //at a dead end
            turn = 5;
            initial_edge = getEdges0();
            Motor_Right(DUTYNOMINAL,DUTYNOMINAL);   //turn 180 degrees in place
        }
    }
    else {  //straight away

        Error = right_distance - IDEALDISTANCE;     //hug the right wall at distance of 80mm

        Up = (Kp*Error)/1000;                       //proportional
        Ui += (Ki*Error*deltaT)/1000;               //integral
        Ud = Kd*(Error - last_error)*1000/deltaT;   //derivative

        if( last_error == -1000) {
            Ud = 0;
        }

        U = Up + Ui + Ud;    //PID controller
        last_error = Error;
        k++;

        //make sure U doesn't go negative
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

/* Handler for which manual move is next and how long each should last */
void Turn(void) {

    switch(turn)
    {
        case 1: //going straight before right turn
            current_edge = getEdges1();
            turn_threshold = 180;
            P2->OUT = (P2->OUT&0xF8)|RED;
            break;
        case 2: //going straight before left turn
            current_edge = getEdges0();
            turn_threshold = 100;
            P2->OUT = (P2->OUT&0xF8)|RED;
            break;
        case 3: //turning right
            current_edge = getEdges1();
            turn_threshold = 185;
            P2->OUT = (P2->OUT&0xF8)|(BLUE+RED);    //Purple
            break;
        case 4: //turning left
            current_edge = getEdges0();
            turn_threshold = 185;
            P2->OUT = (P2->OUT&0xF8)|BLUE;
            break;
        case 5: //turning 180
            current_edge = getEdges0();
            turn_threshold = 370;
            P2->OUT = (P2->OUT&0xF8)|GREEN;
            break;
        case 6: //going straight after right turn
            current_edge = getEdges1();
            turn_threshold = 380;
            P2->OUT = (P2->OUT&0xF8)|(GREEN+BLUE);  //Cyan
            break;
        case 7: //going straight after left turn
            current_edge = getEdges0();
            turn_threshold = 380;
            P2->OUT = (P2->OUT&0xF8)|(GREEN+RED);   //Yellow
            break;
        case 8: // Go Disabled (Bluetooth)
            break;
    }

    if( current_edge - initial_edge >= turn_threshold) {    //if wheel has turned specified amount, transition to next phase in manual move
        switch(turn) {
            case 1: //transition from straight to right turn
                initial_edge = current_edge;
                turn = 3;
                Motor_Left(DUTYNOMINAL,DUTYNOMINAL);
                break;
            case 2: //transition from straight to left turn
                initial_edge = current_edge;
                turn = 4;
                Motor_Right(DUTYNOMINAL,DUTYNOMINAL);
                break;
            case 3: //transition from right turn to straight
                initial_edge = current_edge;
                turn = 6;
                Motor_Forward(DUTYNOMINAL,DUTYNOMINAL);
                break;
            case 4: //transition from left turn to straight
            case 5: //transition from 180 turn to straight
                initial_edge = current_edge;
                turn = 7;
                Motor_Forward(DUTYNOMINAL,DUTYNOMINAL);
                break;
            case 6: //transition out of manual move (i.e. 90 or 180 turn)
            case 7:
                turn = 0;
                //Reset values of position and wheel controllers
                Ui = 0;
                last_error = -1000;
                UR = DUTYNOMINAL;
                UL = DUTYNOMINAL;
                break;
        }
    }
}
