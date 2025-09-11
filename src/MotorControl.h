#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#pragma once
#include <FastAccelStepper.h>
#include "config.h"

long azToSteps(float az);
float stepsToAz(long steps);
long elToSteps(float el);
float stepsToEl(long steps);

// Declare your FastAccelStepper pointers and other functions/flags
extern FastAccelStepper *azMotor;
extern FastAccelStepper *elMotor1;
extern FastAccelStepper *elMotor2;


// Constants
extern float stepsPerDegree;
extern bool elGangedDrive;

// Functions
void moveAzimuthDeg(float degrees);
void moveElevationDeg(float degrees);
void moveAzimuthToPosition(float degrees);
void moveElevationToPosition(float degrees);
void azMotorStop();
void elMotorStop();

long azToSteps(float az);
float stepsToAz(long steps);
long elToSteps(float el);
float stepsToEl(long steps);

bool areMotorsReady();

#endif
