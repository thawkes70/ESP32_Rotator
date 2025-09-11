#pragma once
#include <Arduino.h>
#include "FastAccelStepper.h"
#include "LSM303Receiver.h"
#include "MotorControl.h"
#include "WebLogger.h"

extern LSM303Receiver lsm303;   // tells compiler "this exists somewhere else"

// --- Limit Pins ---
extern const int AZ_LIMIT_PIN;
extern const int EL_LIMIT_PIN;

// --- Motors ---
extern FastAccelStepper* azMotor;
extern FastAccelStepper* elMotor1;
extern FastAccelStepper* elMotor2;
extern bool elGangedDrive;

// --- Homing state ---
enum HomingStage {
    HOMING_IDLE,
    HOMING_AZ_MOVING,
    HOMING_AZ_PRE_HOME,
    HOMING_EL_MOVING,
    HOMING_COMPLETE
};

extern HomingStage homingStage;
extern bool azHomed;
extern bool elHomed;

// --- Homing directions ---
extern int azHomingDir;  // +1 or -1
extern int elHomingDir;  // +1 or -1

// --- Maximum homing distance in steps ---
inline constexpr long MAX_HOMING_STEPS = 15000;

// --- Functions ---
void homeAzimuth();
void homeElevation();
void updateHoming();
