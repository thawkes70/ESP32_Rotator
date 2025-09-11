#include "Calibration.h"
#include "MotorControl.h"   // for moveAzimuthDeg / moveElevationDeg
#include <Arduino.h>
#include "Config.h"
Calibration::Calibration(LSM303Receiver* lsm) : _lsm(lsm) {}

//float elOffset = 0.0f;

void Calibration::begin() {
    azMin = 360; azMax = 0;
    elMin = 180; elMax = 0;
    calStage = CAL_IDLE;
    running = false;
    step = 0;
}

void Calibration::start() {
    Serial.println("[CAL] Starting calibration");
    WEB_LOG_INFO("[CAL]", "Starting Calibration");
    if (_lsm) _lsm->startCalibration();
    begin();
    running = true;
    calStage = CAL_AZ_SWEEP;
    //move El to Zero
    moveElevationDeg(0);
    // Move to start of AZ sweep
    moveAzimuthDeg(MAX_AZ);
}

void Calibration::stop() {
    running = false;
    calStage = CAL_IDLE;
    if (_lsm) _lsm->stopCalibration();
    Serial.println("[CAL] Stopped calibration");
    WEB_LOG_INFO("[CAL]", "Stopped Calibration");
}

bool Calibration::isRunning() const {
    return running;
}

void Calibration::reset() {
    calStage = CAL_IDLE;
    Serial.println("[CAL] Reset to IDLE (emergency stop)");
    WEB_LOG_INFO("[CAL]", "Reset to IDLE (emergency stop)");
}




void Calibration::update() {
    if (!running) return;  // Only act if calibration is running

    // --- Record LSM303 min/max safely ---
    if (_lsm) {
        float currentAz = _lsm->getAzimuth();    // raw readings
        float currentEl = _lsm->getElevation();

        if (currentAz < azMin) azMin = currentAz;
        if (currentAz > azMax) azMax = currentAz;
        if (currentEl < elMin) elMin = currentEl;
        if (currentEl > elMax) elMax = currentEl;
    }

    // --- Stepper-controlled sweep ---
    switch (calStage) {
        case CAL_AZ_SWEEP:
            if (!azMotor->isRunning()) {
                Serial.println("[CAL] AZ sweep complete");
                WEB_LOG_INFO("[CAL]", "AZ Sweep Complete");
                calStage = CAL_EL_SWEEP;
                moveElevationDeg(MAX_EL);
            }
            break;

        case CAL_EL_SWEEP:
            if (!elMotor1->isRunning() && (!elGangedDrive || !elMotor2->isRunning())) {
                Serial.println("[CAL] EL sweep complete");
                WEB_LOG_INFO("[CAL]", "EL Sweep Complete");
                if (_lsm) _lsm->stopCalibration();
                // Compute offset to zero EL at horizontal
                float measuredZeroEl = elMin;  // the lowest EL measured during calibration
                _lsm->setElHomeOffset(-measuredZeroEl);

                Serial.printf("[CAL] EL offset applied: %.2f (raw=%.2f → corrected=%.2f)\n",
                                -measuredZeroEl,
                                _lsm->getElevation(),
                                _lsm->getElCorrected());

                WEB_LOG_INFOF("[CAL]", "EL offset applied: %.2f (raw=%.2f → corrected=%.2f)", 
                                -measuredZeroEl, 
                                _lsm->getElevation(), 
                                _lsm->getElCorrected());

                calStage = CAL_DONE;
              
                Serial.printf("[CAL] Done. AZ: %.2f–%.2f  EL: %.2f–%.2f  EL offset: %.2f\n",
                              azMin, azMax, elMin, elMax, _lsm->getElCorrected() - _lsm->getElevation());
                WEB_LOG_INFOF("[CAL]", "Done.  AZ: %.2f–%.2f  EL: %.2f–%.2f  EL offset: %.2f", 
                                azMin, azMax, elMin, elMax, _lsm->getElCorrected() - _lsm->getElevation());

            }
            break;

          case CAL_DONE:
              Serial.println("[CAL] Calibration complete, moving AZ off endstop for homing...");
              WEB_LOG_INFO("[CAL]", "Calibration complete, moving AZ off endstop for homing...");
          
            if (azMotor) {
                long backoffSteps = -180 * stepsPerDegree;  // relative CCW move
                azMotor->move(backoffSteps);
                calStage = CAL_BACKOFF;
            }

              break;
          
          case CAL_BACKOFF:
              // Wait until AZ motor stops moving before starting homing
              if (!azMotor->isRunning()) {
                  Serial.println("[CAL] AZ backoff complete, starting homing...");
                  WEB_LOG_INFO("[CAL]","AZ backoff complete, starting homing...");
                  homeAzimuth();      // your existing homing routine
                  calStage = CAL_IDLE; // calibration sequence complete
              }
              break;



        case CAL_IDLE:
        default:
            break;
    }
}
