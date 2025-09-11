#include "LSM303Receiver.h"
#include <math.h>
#include <Arduino.h>
#include "Config.h"
#include "MathUtils.h"
#include "WebLogger.h"
#define SMOOTHING_ALPHA 0.2f

float magneticDeclinationDeg = MAGNETIC_DECLINATION;


LSM303Receiver::LSM303Receiver(uint16_t port) : _port(port) {}

void LSM303Receiver::begin() {
    if (_udp.begin(_port)) {
        Serial.printf("LSM303Receiver listening on UDP port %d\n", _port);
        _ready = true;
    } else {
        Serial.println("Failed to start LSM303Receiver UDP");
    }
}

void LSM303Receiver::startCalibration() {
    _calibrating = true;
    azMin = 360.0f; azMax = 0.0f;
    elMin = 90.0f; elMax = -90.0f;
    Serial.println("[LSM303Receiver] Calibration started");
    WEB_LOG_INFOF("[LSM303Receiver]", "Calibration started");
   
}
void LSM303Receiver::stopCalibration() {
    _calibrating = false;
    azOffset = (azMin + azMax) / 2.0f;
    azScale = 360.0f / (azMax - azMin);

    elOffset = -elMin;
    elScale = 180.0f / (elMax - elMin);

    Serial.printf("[LSM303Receiver] Calibration finished: azOffset=%.2f, azScale=%.2f, elOffset=%.2f, elScale=%.2f\n",
                  azOffset, azScale, elOffset, elScale);
    WEB_LOG_INFOF("[LSM303Receiver]", 
              "Calibration finished: azOffset=%.2f, azScale=%.2f, elOffset=%.2f, elScale=%.2f",
              azOffset, azScale, elOffset, elScale);
    
}

void LSM303Receiver::resetCalibration() {
    azOffset = 0.0f; elOffset = 0.0f;
    azScale = 1.0f; elScale = 1.0f;
    Serial.println("[LSM303Receiver] Calibration reset");
    WEB_LOG_INFOF("[LSM303Receiver]", "Calibration reset");
}

void LSM303Receiver::update() {
    if (!_ready) return;
    int packetSize = _udp.parsePacket();
    if (packetSize > 0) {
        char packet[255];
        int len = _udp.read(packet, sizeof(packet) - 1);
        if (len > 0) {
            packet[len] = '\0';
            processPacket(packet, len);
        }
    }
}

void LSM303Receiver::processPacket(const char* packet, int len) {
    float mx,my,mz,ax,ay,az_raw;
    if (sscanf(packet,"MAG:%f,%f,%f;ACC:%f,%f,%f",&mx,&my,&mz,&ax,&ay,&az_raw)==6) {

        // tilt compensation
        float ax_n=ax, ay_n=ay, az_n=az_raw;
        float norm = sqrt(ax*ax+ay*ay+az_raw*az_raw);
        if (norm>0.0f){ ax_n/=norm; ay_n/=norm; az_n/=norm; }

        float pitch = asin(-ax_n);
        float roll  = atan2(ay_n,az_n);

        float xh = mx*cos(pitch)+mz*sin(pitch);
        float yh = mx*sin(roll)*sin(pitch)+my*cos(roll)-mz*sin(roll)*cos(pitch);

        float heading = atan2(yh,xh)*180.0f/PI;
        if (heading<0) heading+=360.0f;

        float elevation = atan2(az_raw, sqrt(ax*ax+ay*ay))*180.0f/PI;

        // --- Calibration capture ---
        if(_calibrating){
            bool updated = false;
            if(heading<azMin) azMin=heading;
            if(heading>azMax) azMax=heading;
            if(elevation<elMin) elMin=elevation;
            if(elevation>elMax) elMax=elevation;
            if(updated){
                WEB_LOG_INFOF("[LSM303Receiver]",
                              "Calibrating: azMin=%.1f azMax=%.1f elMin=%.1f elMax=%.1f",
                              azMin, azMax, elMin, elMax);
            }
        }


        // Apply calibration
        heading = (heading - azOffset) * azScale;
        elevation = (elevation - elOffset) * elScale;

        // Smooth
        _az = SMOOTHING_ALPHA*heading + (1.0f-SMOOTHING_ALPHA)*_az;
        _el = SMOOTHING_ALPHA*elevation + (1.0f-SMOOTHING_ALPHA)*_el;

        _lastUpdate = millis();
    }
}
float LSM303Receiver::getAzimuth() const { return _az; }
float LSM303Receiver::getElevation() const { return _el; }
unsigned long LSM303Receiver::getLastUpdate() const { return _lastUpdate; }
float LSM303Receiver::getElCorrected() const {
    // Map raw LSM reading to physical 0–180°
    // 0° = horizon, 90° = zenith, 180° = straight down
    float physicalEl = 90.0f - _el;      
    return physicalEl;
}

void LSM303Receiver::setElHomeOffset(float offset) {
    _elHomeOffset = offset;
}
void LSM303Receiver::resetElSmoothing() {
    _el = 0.0f;                // zero the smoothed elevation
}

