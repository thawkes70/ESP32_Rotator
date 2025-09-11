#ifndef WEBINTERFACE_H
#define WEBINTERFACE_H

#include <WebServer.h>
#include <WiFi.h>
#include "Calibration.h"
#include "LSM303Receiver.h"   
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// External state variables
extern float currentAzimuth;
extern float currentElevation;
extern float serialAz;
extern float serialEl;
extern bool useMagnetometer;
extern volatile bool azHomingActive;
extern volatile bool elHomingActive;
extern bool isHoming;
extern int gearRatio;
extern int microstepping;
extern float stepsPerRevolution;
extern float stepsPerDegree;
extern const int AZ_LIMIT_PIN;
extern const int EL_LIMIT_PIN;
extern bool elGangedDrive;
extern bool rotctlConnected;  // updated in rotctl_server.cpp
extern float smoothingAlpha;
extern Calibration calib;
extern LSM303Receiver lsmReceiver;
extern bool useLSMforEl;  // true = use LSM for elevation control
extern const char* HARDWARE_ID;
extern const char* FIRMWARE_VERSION;
// External functions from your main sketch
extern void moveAzimuth(int steps);
extern void moveElevation(int steps);
extern void homeAzimuth();
extern void homeElevation();
extern void azMotorStop();
extern void elMotorStop();
extern void updateLimitStates();
extern void handleRotctlCommand(WiFiClient& client, String command);

// Web server instance
extern AsyncWebServer webServer; 

// Setup and request handling
void setupWebServer();
void handleWebServer();

#endif // WEBINTERFACE_H
