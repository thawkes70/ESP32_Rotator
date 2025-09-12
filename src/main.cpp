// ESP32 sketch adapted to receive AZ/EL data from Nano over Serial
#include <Arduino.h>
#include <WiFi.h>
#include <FastAccelStepper.h>
#include <EEPROM.h>
#include "WebInterface.h"
#include "WebLogger.h"
#include "MotorControl.h"
#include "esp_task_wdt.h"
#include "rotctl_server.h"
#include "Homing.h"
#include <ArduinoOTA.h>
#include "LSM303Receiver.h"
#include "Calibration.h"
#include <ElegantOTA.h>

// --- Hardware and Firmware Info for ElegantOTA ---
const char* HARDWARE_ID = "ESP32 Rotator";
const char* FIRMWARE_VERSION = "v1.2.0";  // update this with each firmware release

// === Wi-Fi Credentials ===
const char* ssid = "";
const char* password = "";

// === TCP Server ===
WiFiClient client;

// === Web Logger ===
WebLogger webLogger;

// === Stepper Motor Pins ===
#define AZ_STEP_PIN 12
#define AZ_DIR_PIN 13
#define EL1_STEP_PIN 5
#define EL1_DIR_PIN 18
#define EL2_STEP_PIN 26
#define EL2_DIR_PIN 27

// === Limit Switch Pins ===
const int AZ_LIMIT_PIN = 23;
const int EL_LIMIT_PIN = 19;

// === Limits and Mechanical Constants ===
float motorStepAngle = 1.8;
int gearRatio = 72;
int microstepsPerRevolution = 400;
float stepsPerRevolution = microstepsPerRevolution * gearRatio;
float stepsPerDegree = stepsPerRevolution / 360.0;

bool useLSMforEl = false;  // default: use stepper

// === State ===

bool elGangedDrive = true;
float currentAzimuth = 0.0, currentElevation = 0.0;


bool rotctlConnected = false;

LSM303Receiver lsmReceiver(4210);  // 4210 = UDP port


Calibration calib(&lsmReceiver);



// === FastAccelStepper Setup ===
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *azMotor = NULL;
FastAccelStepper *elMotor1 = NULL;
FastAccelStepper *elMotor2 = NULL;

// === Task Handle ===
TaskHandle_t StepperTask;

// === Stepper Core Task ===
void stepperTaskCode(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1)); // just yield a little time
  }
}

// === Setup Function ===
void setup() {
    Serial.begin(115200);
    delay(100);

    // ----------------------
    // Limit Switch Pins
    // ----------------------
    pinMode(AZ_LIMIT_PIN, INPUT);
    pinMode(EL_LIMIT_PIN, INPUT);

    // ----------------------
    // Wi-Fi
    // ----------------------
    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Wi-Fi connected, IP = ");
    Serial.println(WiFi.localIP());

    // ----------------------
    // Start LSM303Receiver
    // ----------------------
    lsmReceiver.begin();

    // ----------------------
    // Web server
    // ----------------------
    setupWebServer();

    // ----------------------
    // Initialize FastAccelStepper engine
    // ----------------------
    engine.init();

    azMotor = engine.stepperConnectToPin(AZ_STEP_PIN);
    if (azMotor) {
        azMotor->setDirectionPin(AZ_DIR_PIN);
        azMotor->setSpeedInHz(800);
        azMotor->setAcceleration(1000);
    }

    elMotor1 = engine.stepperConnectToPin(EL1_STEP_PIN);
    if (elMotor1) {
        elMotor1->setDirectionPin(EL1_DIR_PIN);
        elMotor1->setSpeedInHz(800);
        elMotor1->setAcceleration(1000);
    }

    elMotor2 = engine.stepperConnectToPin(EL2_STEP_PIN);
    if (elMotor2) {
        elMotor2->setDirectionPin(EL2_DIR_PIN);
        elMotor2->setSpeedInHz(800);
        elMotor2->setAcceleration(1000);
    }

    // ----------------------
    // Stepper core task
    // ----------------------
    xTaskCreatePinnedToCore(stepperTaskCode, "StepperTask", 10000, NULL, 1, &StepperTask, 0);

    // ----------------------
    // Home rotator
    // ----------------------
    homeAzimuth();
   // homeElevation();

    // ----------------------
    // Start rotctl server
    // ----------------------
    startRotctlServer();

    Serial.println("Setup complete.");
}

void loop() {
    // ----------------------
    // Web UI
    // ----------------------
   // handleWebServer();
    // ----------------------
    // Homing updates
    // ----------------------
    updateHoming();

    // ----------------------
    // Update stepper positions to rotctl
    // ----------------------
   // --- Report current rotator position ---
    float azPos = stepsToAz(azMotor->getCurrentPosition());  // always from stepper

    float elPos;
    if (useLSMforEl) {
        // Use LSM-corrected elevation
        elPos = lsmReceiver.getElCorrected();
    } else {
        // Use stepper position
        elPos = stepsToEl(elMotor1->getCurrentPosition());
    }

    // Update rotctl position
    setRotatorPosition(azPos, elPos);


    // ----------------------
    // Update LSM303Receiver
    // ----------------------
    lsmReceiver.update();
    
    // ----------------------
    // Update LSM303Receiver
    // ----------------------
     if (calib.isRunning()) {
        calib.update();
    }
   

}
