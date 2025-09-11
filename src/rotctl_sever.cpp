#include "rotctl_server.h"
#include "MotorControl.h"
#include "Config.h"
#include "WebInterface.h"


// AsyncTCP server
AsyncServer* rotctlServer = nullptr;

// Current position reported to clients
float currentAz = 0.0;
float currentEl = 0.0;

void setRotatorPosition(float az, float el) {
    currentAz = az;
    currentEl = el;
}

void startRotctlServer(uint16_t port) {
    rotctlServer = new AsyncServer(port);

    rotctlServer->onClient([](void *s, AsyncClient* c){
        Serial.println("Rotctl client connected");
        rotctlConnected = true;  // on client connect
        c->onData([c](void *s, AsyncClient* c2, void *data, size_t len){
            String cmd = "";
            for(size_t i = 0; i < len; i++) cmd += ((char*)data)[i];
            cmd.trim();

        if (cmd.equalsIgnoreCase("p")) {
            float azOut = currentAz;
            float elOut;

            if (useLSMforEl) {
                // Report corrected LSM elevation
                elOut = lsmReceiver.getElCorrected();
                Serial.printf("[rotctl] Reporting LSM elevation: %.2f°\n", elOut);
            } else {
                // Report stepper elevation
                elOut = currentEl;
                Serial.printf("[rotctl] Reporting stepper elevation: %.2f°\n", elOut);
            }

            String msg = String(azOut, 2) + "\n" + String(elOut, 2) + "\n";
            c2->write(msg.c_str(), msg.length());
        }
            else if (cmd.startsWith("P ")) {
                float az, el;
                if (sscanf(cmd.c_str(), "P %f %f", &az, &el) == 2) {
                    // Constrain to min/max limits
                    az = constrain(az, MIN_AZ, MAX_AZ);
                    el = constrain(el, MIN_EL, MAX_EL);

                    // Move motors
                    moveAzimuthToPosition(az);
                    moveElevationToPosition(el);


                    // Respond success
                    const char* ok = "RPRT 0\n";
                    c2->write(ok, strlen(ok));
                } else {
                    const char* err = "RPRT -1\n";
                    c2->write(err, strlen(err));
                }
            }
            else {
                const char* err = "RPRT -1\n";
                c2->write(err, strlen(err));
            }
        }, nullptr);

        c->onDisconnect([](void *s, AsyncClient* c2){
            Serial.println("Rotctl client disconnected");
            rotctlConnected = false; // on disconnect
        }, nullptr);

        c->onError([](void *s, AsyncClient* c2, int8_t error){
            Serial.printf("Rotctl client error %d\n", error);
        }, nullptr);

        c->onTimeout([](void *s, AsyncClient* c2, uint32_t time){
            Serial.println("Rotctl client timeout");
        }, nullptr);

    }, nullptr);

    rotctlServer->begin();
    Serial.printf("Rotctl server started on port %d\n", port);
}
