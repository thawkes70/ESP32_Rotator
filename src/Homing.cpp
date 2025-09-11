#include "Homing.h"
extern LSM303Receiver lsmReceiver;

// --- Homing state variables ---
HomingStage homingStage = HOMING_IDLE;
bool azHomed = false;
bool elHomed = false;
int azHomingDir = -1; // example: CCW
int elHomingDir = -1; // example: DOWN

// --- Debounced AZ limit globals ---
unsigned long azLimitLastChange = 0;
const unsigned long AZ_LIMIT_DEBOUNCE_MS = 5;  // ms
bool azLimitState = false; // debounced state

// --- Debounced EL limit globals ---
unsigned long elLimitLastChange = 0;
const unsigned long EL_LIMIT_DEBOUNCE_MS = 5;  // ms
bool elLimitState = false; // debounced state

//Homing delay globals
unsigned long elStopStartTime = 0;
bool elStopStarted = false;

void updateAzLimit() {
    bool raw = digitalRead(AZ_LIMIT_PIN) == LOW;
    unsigned long now = millis();
    if (raw != azLimitState && now - azLimitLastChange >= AZ_LIMIT_DEBOUNCE_MS) {
        azLimitState = raw;
        azLimitLastChange = now;
        Serial.print("[AZ LIMIT] "); Serial.println(azLimitState ? "TRIGGERED" : "CLEAR");
    }
}

void updateElLimit() {
    bool raw = digitalRead(EL_LIMIT_PIN) == LOW;
    unsigned long now = millis();
    if (raw != elLimitState && now - elLimitLastChange >= EL_LIMIT_DEBOUNCE_MS) {
        elLimitState = raw;
        elLimitLastChange = now;
        Serial.print("[EL LIMIT] "); Serial.println(elLimitState ? "TRIGGERED" : "CLEAR");
    }
}

void homeAzimuth() {
    if (!azMotor) return;
    homingStage = HOMING_AZ_PRE_HOME;
    azHomed = false;
    long target = azHomingDir * MAX_HOMING_STEPS;
    Serial.println("[HOMING] Starting azimuth homing...");
    Serial.print("[HOMING] Moving AZ motor towards "); 
    Serial.print(azHomingDir > 0 ? "CW" : "CCW");
    Serial.print(" for max "); Serial.print(MAX_HOMING_STEPS); Serial.println(" steps");
    azMotor->moveTo(target, false);
}

void homeElevation() {
    if (!elMotor1) return;
    homingStage = HOMING_EL_MOVING;
    elHomed = false;
    long target = elHomingDir * MAX_HOMING_STEPS;
    Serial.println("[HOMING] Starting elevation homing...");
    Serial.print("[HOMING] Moving EL motors towards "); 
    Serial.print(elHomingDir > 0 ? "UP" : "DOWN");
    Serial.print(" for max "); Serial.print(MAX_HOMING_STEPS); Serial.println(" steps");
    elMotor1->moveTo(target, false);
    if (elGangedDrive && elMotor2) elMotor2->moveTo(target, false);
}

void updateHoming() {
    // --- Update limit switches ---
    updateAzLimit();
    updateElLimit();

    switch (homingStage) {
        case HOMING_AZ_PRE_HOME:
        // Check if we need to move az to safe pre-home position
        {
            float azDeg = stepsToAz(azMotor->getCurrentPosition());
            if (azDeg < 0 || azDeg > 360) {
                WEB_LOG_WARNINGF("[HOMING]", "AZ out of range (%.1f°), moving to 180° before homing", azDeg);
                moveAzimuthToPosition(180.0);  // queue movement
            } else {
                WEB_LOG_INFO("[HOMING]", "AZ within safe range, skipping pre-home move");
                homingStage = HOMING_AZ_MOVING; // move on to normal az homing
            }
        }

        // Non-blocking check: only advance when az has finished moving
        if (!azMotor->isRunning()) {
            homingStage = HOMING_AZ_MOVING;
        }
        break;

        case HOMING_AZ_MOVING:
        if (azLimitState) {  // debounced limit triggered
            azMotor->stopMove();
            delay(800);   // allow motion to fully cease
            azMotor->setCurrentPosition(0);  // reset home
            azHomed = true;
            Serial.println("[HOMING] Azimuth limit reached, position set to 0");
            WEB_LOG_INFO("[HOMING]", "Azimuth limit reached, position set to 0");
            homeElevation();  // move to elevation homing next
        }
        break;


/*        case HOMING_EL_MOVING:
        if (!azHomed) break; // safety: wait until azimuth is homed
        if (elLimitState) {
            // Stop motors
            elMotor1->stopMove();
            delay(800);   // allow motion to fully cease
            elMotor1->setCurrentPosition(0);
            if (elGangedDrive && elMotor2) elMotor2->setCurrentPosition(0);

            // Store home reference: horizon = 0°
            float rawEl = lsmReceiver.getElevation();  
            lsmReceiver.setElHomeRaw(90.0f - rawEl);  
            lsmReceiver.resetElSmoothing();          

            elHomed = true;

            Serial.printf("[HOMING] Elevation homed. Raw=%.2f, Corrected=%.2f\n",
                            lsmReceiver.getElevation(), lsmReceiver.getElCorrected());
            Serial.println("[HOMING] Elevation limit reached, position set to 0");

            homingStage = HOMING_COMPLETE;

            // Move to 90° (zenith) after homing
            moveElevationDeg(90.0f);
            Serial.println("[HOMING] Elevation homed, moving to 90 degrees...");
            Serial.println("[HOMING] Homing sequence complete!");
        }
        break;
*/


            case HOMING_EL_MOVING:
                if (!azHomed) break; // safety: wait until azimuth is homed
                if (elLimitState) {
                    if (!elStopStarted) {
                        // Stop the motor and record the time
                        elMotor1->stopMove();
                        if (elGangedDrive && elMotor2) elMotor2->stopMove();
                        elStopStartTime = millis();
                        elStopStarted = true;
                        break;  // exit to next loop iteration
                    }

                    // Wait non-blocking for 800ms
                    if (millis() - elStopStartTime < 800) break;

                    // Now safe to reset position counters
                    elMotor1->setCurrentPosition(0);
                    if (elGangedDrive && elMotor2) elMotor2->setCurrentPosition(0);

                    lsmReceiver.setElHomeRaw(lsmReceiver.getElevation());  // store raw home value
                    lsmReceiver.resetElSmoothing();                        // reset smoothing

                    elHomed = true;

                    Serial.printf("[HOMING] Elevation homed. Raw=%.2f, Corrected=%.2f",
                                    lsmReceiver.getElevation(), lsmReceiver.getElCorrected());
                    WEB_LOG_INFOF("[HOMING]", "Elevation homed. Raw=%.2f, Corrected=%.2f",
                                    lsmReceiver.getElevation(), lsmReceiver.getElCorrected());
                    Serial.println("[HOMING] Elevation limit reached, position set to 0");
                    WEB_LOG_INFO("[HOMING]", "Elevation limit reached, position set to 0");
                    homingStage = HOMING_COMPLETE;
/*
                    moveElevationDeg(90.0f);
                    Serial.println("[HOMING] Elevation homed, moving to 90 degrees...");
                    Serial.println("[HOMING] Homing sequence complete!");
*/
                    // Reset the stop state for next homing
                    elStopStarted = false;
                }
                break;



        case HOMING_COMPLETE:
            // nothing to do
            break;

        default:
            break;
    }
}
