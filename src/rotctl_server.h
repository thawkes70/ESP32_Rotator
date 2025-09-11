#pragma once
#include <WiFi.h>
#include <AsyncTCP.h>
#include <FastAccelStepper.h>

// Make your motors available
extern FastAccelStepper *azMotor;
extern FastAccelStepper *elMotor1;
extern FastAccelStepper *elMotor2;
extern bool elGangedDrive;

// Limits
extern const int MIN_AZ;
extern const int MAX_AZ;
extern const int MIN_EL;
extern const int MAX_EL;

// Conversion functions
extern long azToSteps(float az);
extern long elToSteps(float el);

// Rotator position
void setRotatorPosition(float az, float el);
void startRotctlServer(uint16_t port = 4533);

extern bool rotctlConnected;  // updated in rotctl_server.cpp
