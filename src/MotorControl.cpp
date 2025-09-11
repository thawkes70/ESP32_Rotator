#include "MotorControl.h"
#include "WebLogger.h"  // Ensure logging works
#include "MotorControl.h"
#include "Calibration.h"
#include "WebInterface.h"

extern Calibration calib;


// Azimuth control
void moveAzimuthDeg(float deg) {
    if (!azMotor) return;
    float originalDeg = deg;
    long steps = deg * stepsPerDegree;
    azMotor->move(steps);
    WEB_LOG_DEBUGF("Motor", "moveAzimuthDeg called: %f deg -> %ld steps", deg, steps);
}


void moveElevationDeg(float deg) {
    if (!elMotor1 || !elMotor2) return;
    float originalDeg = deg;
    long steps = deg * stepsPerDegree;
    elMotor1->move(steps);
    if (elGangedDrive && elMotor2) elMotor2->move(steps);
    WEB_LOG_DEBUGF("Motor", "moveElevationDeg called: %f deg -> %ld steps", deg, steps);
}



// MoveTo() version
void moveAzimuthToPosition(float degrees) {
    float originalDeg = degrees;
    long targetSteps = azToSteps(degrees);
    azMotor->moveTo(targetSteps);
}

void moveElevationToPosition(float degrees) {
    float originalDeg = degrees;
    long targetSteps = elToSteps(degrees);
    if (elGangedDrive) {
        elMotor1->moveTo(targetSteps);
        elMotor2->moveTo(targetSteps);
    } else {
        elMotor1->moveTo(targetSteps);
    }
}

void emergencyStop() {
  if (azMotor) azMotor->forceStop();
  if (elMotor1) elMotor1->forceStop();
  if (elMotor2) elMotor2->forceStop();
  calib.reset();
  azHomed = false;
  elHomed = false;

  WEB_LOG_WARN("Motor","Emergency stop executed");
}
void azMotorStop() {
  if (azMotor) azMotor->forceStop();
}

void elMotorStop() {
  if (elMotor1) elMotor1->forceStop();
  if (elGangedDrive && elMotor2) elMotor2->forceStop();
}
bool areMotorsReady() {
    bool azReady = (azMotor && !azMotor->isRunning());
    bool el1Ready = (elMotor1 && !elMotor1->isRunning());
    bool el2Ready = (elMotor2 && !elMotor2->isRunning());

    return azReady && el1Ready && el2Ready;
}

// Unit conversions
long azToSteps(float az) {
  return az * stepsPerDegree;
}

float stepsToAz(long steps) {
  return steps / stepsPerDegree;
}

long elToSteps(float el) {
  return el * stepsPerDegree;
}

float stepsToEl(long steps) {
  return steps / stepsPerDegree;
}
