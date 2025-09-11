#pragma once
#include "LSM303Receiver.h"
#include "Homing.h"

//extern float elOffset; // degrees

enum CalStage {
    CAL_IDLE,
    CAL_AZ_SWEEP,
    CAL_EL_SWEEP,
    CAL_DONE,
    CAL_BACKOFF
};

class Calibration {
public:
    Calibration(LSM303Receiver* lsm);

    void begin();
    void start();
    void stop();
    void update();
    bool isRunning() const;
    void reset();

    float getAzMin() const { return azMin; }
    float getAzMax() const { return azMax; }
    float getElMin() const { return elMin; }
    float getElMax() const { return elMax; }

private:
    LSM303Receiver* _lsm;

    bool running = false;
    int step = 0;
    CalStage calStage = CAL_IDLE;

    float azMin = 360, azMax = 0;
    float elMin = 180, elMax = 0;
};
