#pragma once
#include <WiFiUdp.h>

class LSM303Receiver {
public:
    LSM303Receiver(uint16_t port);

    void begin();
    void update();
    float getAzimuth() const;
    float getElevation() const;
    float getElCorrected() const;
    unsigned long getLastUpdate() const;

    void setElHomeOffset(float offset);
    float getElHomeOffset() const { return _elHomeOffset; }
    void setElHomeRaw(float raw) { _elHomeRaw = raw; }
    void resetElSmoothing();

    float getAzMin() const { return azMin; }
    float getAzMax() const { return azMax; }
    float getElMin() const { return elMin; }
    float getElMax() const { return elMax; }

    // --- Calibration functions ---
    void startCalibration();
    void stopCalibration();
    void resetCalibration();
    bool isCalibrating() const { return _calibrating; }

private:
    void processPacket(const char* packet, int len);

    uint16_t _port;
    WiFiUDP _udp;
    bool _ready = false;

    float _az = 0.0f;
    float _el = 0.0f;
    unsigned long _lastUpdate = 0;

    // --- Calibration ---
    bool _calibrating = false;
    float azMin = 360.0f, azMax = 0.0f;
    float elMin = 90.0f, elMax = -90.0f;
    float _elHomeOffset = 0.0f;
    float azOffset = 0.0f;
    float elOffset = 0.0f;
    float azScale = 1.0f;
    float elScale = 1.0f;

    float _elHomeRaw = 0.0f;   // raw EL value at home
};
