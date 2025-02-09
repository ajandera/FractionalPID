/*
  Fractional PID example implementation

  Created by Ing. Ales Jandera.
  Last update: 9.2.2025.

  Depends on AeroShield

  Based on:

  AeroShield closed-loop PID response example

  PID feedback control of pendulum angle of the AeroShield.

  This example initialises the sampling and PID control
  subsystems from the AutomationShield library and allows user
  to select whether the reference is given by the potentiometer
  or by a predetermined reference trajectory. Upload the code to
  your board and open the Serial Plotter tool in Arduino IDE.

  Tested on Arduino UNO Rev3 and Mega 2560 Rev3.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Peter Tibenský.
  Last update: 16.1.2023.
*/

#include "AeroShield.h"               // Include main library  
#include <Sampling.h>                 // Include sampling library

#include "FractionalPID.h"            // Include fractional PID library

#define MANUAL 0                      // Choose manual reference using potentiometer (1)  automatic reference trajectory (0) 
#define KP 1                        // PID Kp constant
#define TI 1.8182                       // PID Ti constant
#define TD 0.25                       // PID Td constant
#define LAMBDA 1                    // order of fractional integral for fractional PID
#define DELTA 1                    // order of fractional derivative for fractional PID


unsigned long Ts = 5;                 // Sampling period in milliseconds
//unsigned long k = 1;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation

int i = 0;                            // Section counter
int T = 1000;                         // Section length in ms
float R[] = {50.0}; // Reference trajectory
float r = 50.0;                        // Reference (Wanted pendulum angle)
float y = 0.0;                        // Output (Current pendulum angle)
float u = 0.0;                        // Input (motor power)
int K0 =  1;                     // K0 of the nonlinear function f(e(t))=K0+(1-K0)*|e(t)| is:

float Kp=1.0;
float Ki=1.0;
float Kd=0.7;
float Ti=0.55;
float Td=0.25;
float lambda=0.6;
float delta=0.5;
float e=0.0;
float e_history[100];  // Stores past error values for fractional differentiation
int k = 1;  // Time step counter

void setup() {                                  //  Setup - runs only once
  Serial.begin(115200);                        //  Begin serial communication
  AeroShield.begin();                           //  Initialise AeroShield board
  AeroShield.calibrate();                       //  Calibrate AeroShield board + store the 0° value of the pendulum
  Sampling.period(Ts * 1000);                   // Set sampling period in milliseconds
  Sampling.interrupt(stepEnable);               // Set interrupt function
}

void loop() {
  if (y > 100.0) {                              // If pendulum agle too big
    AeroShield.actuatorWrite(0);                // Turn off motor
    while (1);                                  // Stop program
  }
  if (nextStep) {                               // If ISR enables step flag
    step();                                     // Run step function
    nextStep = false;                           // Disable step flag
  }
}

void stepEnable() {                                // ISR
  if (nextStep == true) {                          // If previous sample still running
    realTimeViolation = true;                      // Real-time has been violated
    Serial.println("Real-time samples violated."); // Print error message
    analogWrite(5, 0);                             // Turn off the motor
    while (1);                                     // Stop program execution
  }
  nextStep = true;                                // Enable step flag
}

void step() {                                    // Define step function

  #if MANUAL                                       // If Manual mode is active
    r = AeroShield.referenceRead();                // Read reference from potentiometer
  #else 
    // If Automatic mode is active
    if (i > (sizeof(R) / sizeof(R[0]))) {          // If at end of trajectory
      analogWrite(5, 0);                           // Turn off the motor
      while (1);                                   // Stop program execution
    } else if (k % (T * i) == 0) {                 // If at the end of section
      r = R[i];                                    // Progress in trajectory
      i++;                                         // Increment section counter
    } 
  #endif

  y = AeroShield.sensorRead();    //  read pendulum angle in %
  u = FractionalPID.compute(r - y, 0.0, 100.0);
  AeroShield.actuatorWrite(u);                    // Actuate
  k++;                                            // Increment index
}

float constrainFloat(float x, float min_x, float max_x) {
  if (x<=min_x)
    return min_x;
  else if (x>=max_x)
    return max_x;
  return x;
}