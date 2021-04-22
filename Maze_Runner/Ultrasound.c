// Ultrasound.c
// Runs on MSP432
// Provide mid-level functions that initialize ports, start
// an ultrasonic sensor measurement, and finish an ultrasonic
// sensor measurement using the HC-SR04 ultrasonic distance
// sensor.
// Daniel Valvano
// May 2, 2017

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

// Pololu #3543 Vreg (5V regulator output) connected to HC-SR04 Vcc (+5V) and MSP432 +5V (J3.21)
// 22k top connected to HC-SR04 Echo (digital output from sensor)
// 22k bottom connected to 33k top and MSP432 P6.6 (J4.36) (digital input to MSP432)
// 33k bottom connected to ground
// Pololu ground connected to HC-SR04 ground and MSP432 ground (J3.22)
// MSP432 P5.6 (J4.37) (digital output from MSP432) connected to HC-SR04 trigger

#include <stdint.h>
#include "../inc/Clock.h"
#include "../inc/TA2InputCapture.h"
#include "../inc/LPF.h"
#include "msp.h"

#define NUMBER_OF_SENSORS 3

uint16_t first_edge[NUMBER_OF_SENSORS], second_edge[NUMBER_OF_SENSORS], width[NUMBER_OF_SENSORS];
int Ultrasound_Count[NUMBER_OF_SENSORS] = {0};          //incremented with every interrupt
int first_read[NUMBER_OF_SENSORS] = {1, 1, 1};
double duration[NUMBER_OF_SENSORS];
uint32_t distance[NUMBER_OF_SENSORS], LPF_distance[NUMBER_OF_SENSORS];

int N = 10;        //N = number of samples


void ultrasoundint(uint16_t currenttime, int sensor_number) {

    if((Ultrasound_Count[sensor_number]%2) == 0) {
        first_edge[sensor_number] = currenttime;
    }
    else {
        second_edge[sensor_number] = currenttime;

        width[sensor_number] = second_edge[sensor_number] - first_edge[sensor_number];
        duration[sensor_number] = (width[sensor_number])*(1.0/3);              //take average an divide by 3 to put in terms of microseconds
        distance[sensor_number] = (duration[sensor_number]*343) / (2*10000);   //convert microseconds to distance

        /*LOW PASS FILTER*/
        if( first_read[sensor_number] == 1) {
            switch(sensor_number) {
                case 0: LPF_Init(distance[sensor_number], N); break;
                case 1: LPF_Init2(distance[sensor_number], N); break;
                case 2: LPF_Init3(distance[sensor_number], N); break;
            }
            first_read[sensor_number] = 0;
        }

        switch(sensor_number) { //put distance calculation through low pass filter
            case 0: LPF_distance[sensor_number] = LPF_Calc(distance[sensor_number]); break;
            case 1: LPF_distance[sensor_number] = LPF_Calc2(distance[sensor_number]); break;
            case 2: LPF_distance[sensor_number] = LPF_Calc3(distance[sensor_number]); break;
        }
    }

    Ultrasound_Count[sensor_number]++;
}


uint32_t getLeftDistance(void) {
    return LPF_distance[0];
}

uint32_t getRightDistance(void) {
    return LPF_distance[1];
}


// ------------Ultrasound_Init------------
// Initialize a GPIO pin for output, which will be
// used to trigger the ultrasonic sensor.
// Initialize the input capture interface, which
// will be used to take the measurement.
// Input: none
// Output: none
void Ultrasound_Init(void){
  // initialize P6.4 and make it GPIO
  P6->SEL0 &= ~0x20;
  P6->SEL1 &= ~0x20;               // configure P6.5 as GPIO
  P6->DIR |= 0x20;                 // make P6.5 output
  P6->OUT &= ~0x20;
  TimerA2Capture_Init(&ultrasoundint);
}
