// WebInterface.cpp - updated UI with degree-input jogs + Emergency Stop
#include "WebInterface.h"
#include "WebLogger.h"
#include <ESPmDNS.h>
#include <WiFi.h>
#include "MotorControl.h"
#include "LSM303Receiver.h"
#include "Calibration.h"
#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "MathUtils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"

// externs - make sure these match your main sketch
extern float currentAzimuth;
extern float currentElevation;
extern float serialAz;     // LSM303 az read (from Nano/adapter)
extern float serialEl;     // LSM303 el read
//extern bool useMagnetometer;
extern bool azHomed;
extern float stepsPerRevolution;
extern float stepsPerDegree;
extern const int AZ_LIMIT_PIN;
extern const int EL_LIMIT_PIN;


extern void moveAzimuthDeg(float degrees);
extern void moveElevationDeg(float degrees);
extern void moveAzimuthToPosition(float degrees);
extern void moveElevationToPosition(float degrees);
extern void emergencyStop();
extern void homeAzimuth();
extern void homeElevation();

extern FastAccelStepper *azMotor;
extern FastAccelStepper *elMotor1;
extern FastAccelStepper *elMotor2;

extern bool rotctlConnected;
extern LSM303Receiver lsmReceiver;
float smoothingAlpha = 0.20f;
AsyncWebServer webServer(80);

float azTrue = magneticToTrue(lsmReceiver.getAzimuth());

// helper to build JSON of tasks (kept minimal)
String getTaskInfoJSON() {
  String json = "\"tasks\":[]"; // keep simple for now
  return json;
}

void setupWebServer() {
    // Root page - full UI
webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>ESP32 Rotator Control</title>
<style>
  body{font-family:Arial;background:#111;color:#eee;padding:20px}
  .panel{background:#222;border-radius:8px;padding:20px;margin:10px}
  .big-stop{background:#c62828;color:#fff;padding:16px 24px;border-radius:8px;font-size:20px;border:none;cursor:pointer}
  .input-row{margin:8px 0}
  input[type="number"]{width:80px;padding:4px}
  button{padding:6px 10px;margin-left:6px}
  .status{font-family:monospace}
</style>
</head>
<body>
<div class="panel">
<h1>ESP32 Rotator</h1>
<div class="input-row">
  <strong>Hardware:</strong> <span id="hardware">--</span><br>
  <strong>Firmware:</strong> <span id="firmware">--</span>
</div>
<div class="card">
  <h3>Smoothing Factor</h3>
  <input type="range" id="alphaSlider" min="0" max="100" value="20" step="1" 
          oninput="updateAlpha(this.value)">
  <span id="alphaValue">0.20</span>
</div>
<div class="input-row">
  <strong>Azimuth (Mag):</strong> <span id="lsmAz">--</span>°
  <strong>Azimuth (True):</strong> <span id="lsmAzTrue">--</span>°
  <strong>LSM EL (raw):</strong> <span id="lsmEl">--</span>&deg; &nbsp;
  <strong>LSM EL (corrected):</strong> <span id="lsmElCorr">--</span>&deg;
</div>

<div class="input-row">
  <strong>Stepper AZ:</strong> <span id="az">--</span>&deg; &nbsp;
  <strong>Stepper EL:</strong> <span id="el">--</span>&deg;
</div>

<div class="input-row">
  <label>
    <input type="checkbox" id="useLSM" onchange="toggleElSource()">
    Use LSM303 for Elevation
  </label>
</div>

<div class="input-row">
  <strong>Azimuth Limit:</strong> <span id="azLimitStatus" style="color:gray">--</span><br>
  <strong>Elevation Limit:</strong> <span id="elLimitStatus" style="color:gray">--</span>
</div>

<div class="input-row">
  <strong>Rotctl:</strong> <span id="rotctlStatus">Disconnected</span>
</div>

<hr>

<h3>Jog Controls (degrees)</h3>
<div class="input-row">
  <label>AZ step: <input type="number" id="jogAz" step="0.1" value="1.0"></label>
  <button onclick="jogAz(1)">Left</button>
  <button onclick="jogAz(-1)">Right</button>
</div>
<div class="input-row">
  <label>EL step: <input type="number" id="jogEl" step="0.1" value="1.0"></label>
  <button onclick="jogEl(1)">Down</button>
  <button onclick="jogEl(-1)">Up</button>
</div>

<hr>

<h3>Homing</h3>
<div class="input-row">
  <button onclick="homeAz()">Home Azimuth</button>
  <button onclick="homeEl()">Home Elevation</button>
</div>

<hr>

<h3>Move to absolute position</h3>
<form id="moveForm">
  <label>AZ &deg;: <input type="number" id="moveAz" step="0.1" value="0.0"></label>
  <label>EL &deg;: <input type="number" id="moveEl" step="0.1" value="90.0"></label>
  <button type="button" onclick="moveTo()">Move</button>
</form>

<hr>

<div class="input-row">
  <button id="estop" class="big-stop" onclick="emergencyStop()">Emergency Stop</button>
  <button id="resetEsp" class="big-stop" style="background-color: orange;" onclick="resetESP()">Reset ESP32</button>
</div>

<div style="margin-top:8px;color:#ffcccb">
  Warning: Emergency Stop will immediately stop all motor movement
</div>

<hr>

<div class="input-row">
  <button id="autoCal" class="big-stop" style="background-color: #ff9800; color: white; font-size: 18px; padding: 10px; border-radius: 10px;"
    onclick="startAutoCalibration()">Automatic Calibration</button>
  <button id="otaUpdate" class="big-stop" style="background-color: #4caf50; color: white; font-size: 18px; padding: 10px; border-radius: 10px;"
    onclick="startOTA()">OTA Update</button>
</div>

<div style="margin-top:8px;color:#ffcccb">
  Warning: Automatic Calibration will move the rotator through a full pattern. Ensure the area is clear.
</div>

<div class="input-row" style="margin-top:6px;">
  <strong>Cal Min/Max:</strong> <span id="calValues">AZ -- / -- , EL -- / --</span>
</div>

<h3>Log</h3>
<div id="logContainer" style="background:#111;color:#0f0;padding:8px;height:200px;overflow-y:scroll;font-family:monospace;font-size:12px"></div>
<h2>Task List</h2>
<pre id="taskList">Loading...</pre>

<script>
function toggleElSource() {
  const useLSM = document.getElementById("useLSM").checked ? 1 : 0;
  fetch(`/setElSource?value=${useLSM}`);
}

function updateAlpha(val) {
  let alpha = (val / 100).toFixed(2);
  document.getElementById("alphaValue").innerText = alpha;
  fetch(`/setAlpha?value=${alpha}`);
}

function updateStatus() {
  fetch('/status').then(r=>r.json()).then(data=>{
    document.getElementById('hardware').innerText = data.hardware || '--';
    document.getElementById('firmware').innerText = data.firmware || '--';

    document.getElementById('lsmAz').innerText = data.lsmAz!==null? data.lsmAz.toFixed(1) : '--';
    document.getElementById('lsmEl').innerText = data.lsmEl!==null? data.lsmEl.toFixed(1) : '--';
    document.getElementById('lsmElCorr').innerText = data.lsmElCorr!==null? data.lsmElCorr.toFixed(1) : '--';
    if (document.getElementById('lsmAzTrue')) {
      document.getElementById('lsmAzTrue').innerText =
        data.lsmAzTrue !== null ? data.lsmAzTrue.toFixed(1) : '--';
    }
    document.getElementById('az').innerText = data.az!==null? data.az.toFixed(1) : '--';
    document.getElementById('el').innerText = data.el!==null? data.el.toFixed(1) : '--';
    document.getElementById('azLimitStatus').innerText = data.azLimit===0?"TRIGGERED":"Clear";
    document.getElementById('azLimitStatus').style.color = data.azLimit===0?"red":"green";
    document.getElementById('elLimitStatus').innerText = data.elLimit===0?"TRIGGERED":"Clear";
    document.getElementById('elLimitStatus').style.color = data.elLimit===0?"red":"green";
    document.getElementById('rotctlStatus').innerText = data.rotctl?"Connected":"Disconnected";
    document.getElementById('rotctlStatus').style.color = data.rotctl?"green":"red";
  }).catch(e=>{});
}

function updateLogs() {
  fetch('/logs').then(r=>r.json()).then(data=>{
    const container = document.getElementById('logContainer');
    container.innerHTML='';
    data.forEach(entry=>{
      let color="#0f0";
      if(entry.level==1) color="#0af";
      else if(entry.level==2) color="#ff0";
      else if(entry.level==3) color="#f00";
      const ts=new Date(entry.timestamp+performance.now()-performance.timing.navigationStart).toLocaleTimeString();
      container.innerHTML+=`[${ts}] ${entry.source}: <span style="color:${color}">${entry.message}</span><br>`;
    });
    container.scrollTop=container.scrollHeight;
  }).catch(e=>{});
}

function jogAz(dir) {
  const deg=parseFloat(document.getElementById('jogAz').value)||0.0;
  const payload=new URLSearchParams();
  payload.append('azStep',(deg*dir).toString());
  fetch('/jog',{method:'POST',body:payload}).then(()=>updateStatus());
}

function jogEl(dir) {
  const deg=parseFloat(document.getElementById('jogEl').value)||0.0;
  const payload=new URLSearchParams();
  payload.append('elStep',(deg*dir).toString());
  fetch('/jog',{method:'POST',body:payload}).then(()=>updateStatus());
}

function moveTo() {
  const az=parseFloat(document.getElementById('moveAz').value)||0.0;
  const el=parseFloat(document.getElementById('moveEl').value)||0.0;
  const payload=new URLSearchParams();
  payload.append('moveAz',az.toString());
  payload.append('moveEl',el.toString());
  fetch('/move',{method:'POST',body:payload}).then(()=>updateStatus());
}

function emergencyStop() { fetch('/estop',{method:'POST'}); }
function homeAz() { fetch('/homeAz',{method:'POST'}); }
function homeEl() { fetch('/homeEl',{method:'POST'}); }

function startAutoCalibration() {
  fetch('/autoCal').then(r=>r.text()).then(msg=>{}).catch(err=>{alert("Failed to start calibration.");});
}

function updateCalibrationDisplay() {
  fetch('/cal/status').then(r=>r.json()).then(data=>{
    document.getElementById('calValues').innerText=
      `AZ ${data.azMin.toFixed(1)} / ${data.azMax.toFixed(1)}, EL ${data.elMin.toFixed(1)} / ${data.elMax.toFixed(1)}`;
  }).catch(e=>{});
}

function resetESP() {
  fetch('/reset').then(r=>r.text()).then(msg=>{alert("ESP32 is restarting...");}).catch(err=>console.error(err));
}

function updateTasks() {
  fetch('/tasks')
    .then(r => r.text())
    .then(data => {
      document.getElementById('taskList').innerText = data;
    });
}
setInterval(updateTasks, 5000); // refresh every 5s

function startOTA() {
      fetch('/update')
          .then(resp => {
            window.open('/update', '_blank');
          })
          .catch(err => {
              alert("Failed to start OTA: " + err);
          });
  }
setInterval(updateCalibrationDisplay,1000);
setInterval(updateStatus,800);
setInterval(updateLogs,1000);
updateStatus();
updateLogs();
</script>
</div>
</body>
</html>
)rawliteral");
});

    // --- Status endpoint ---
    webServer.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        const bool fresh = (millis() - lsmReceiver.getLastUpdate() < 2000);
        float safeLsmAz        = fresh ? lsmReceiver.getAzimuth()   : 0.0;
        float safeLsmElRaw     = fresh ? lsmReceiver.getElevation() : 0.0;
        float safeLsmElCorrected = lsmReceiver.getElCorrected();
        float safeLsmAzTrue    = fresh ? magneticToTrue(safeLsmAz) : 0.0;

        json += "\"lsmAz\":"     + String(safeLsmAz, 1) + ",";
        json += "\"lsmAzTrue\":" + String(safeLsmAzTrue, 1) + ",";   
        json += "\"lsmEl\":"     + String(safeLsmElRaw, 1) + ",";
        json += "\"lsmElCorr\":" + String(safeLsmElCorrected, 1) + ",";

        long azSteps = azMotor ? azMotor->getCurrentPosition() : 0;
        long elSteps = elMotor1 ? elMotor1->getCurrentPosition() : 0;
        float azDeg = stepsToAz(azSteps);
        float elDeg = stepsToEl(elSteps);

        json += "\"az\":" + String(azDeg, 1) + ",";
        json += "\"el\":" + String(elDeg, 1) + ",";
        json += "\"azLimit\":" + String((AZ_LIMIT_PIN >= 0) ? digitalRead(AZ_LIMIT_PIN) : 0) + ",";
        json += "\"elLimit\":" + String((EL_LIMIT_PIN >= 0) ? digitalRead(EL_LIMIT_PIN) : 0) + ",";
        json += "\"azHomed\":" + String(azHomed ? "true" : "false") + ",";
        json += "\"tasks\":[],";
        json += "\"rotctl\":" + String(rotctlConnected ? "true" : "false") + ",";

        json += "\"hardware\":\"" + String(HARDWARE_ID) + "\",";
        json += "\"firmware\":\"" + String(FIRMWARE_VERSION) + "\"";


        json += "}";

        request->send(200, "application/json", json);
    });

    // --- Set smoothing factor ---
    webServer.on("/setAlpha", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            String value = request->getParam("value")->value();
            smoothingAlpha = value.toFloat();
            smoothingAlpha = constrain(smoothingAlpha, 0.0f, 1.0f);
            Serial.printf("[WebUI] Smoothing factor updated: %.2f\n", smoothingAlpha);
        }
        request->send(200, "text/plain", "OK");
    });

    // --- Use LSM for Elevation ---
    webServer.on("/setElSource", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            int val = request->getParam("value")->value().toInt();
            useLSMforEl = (val != 0);
            WEB_LOG_INFOF("WebUI", "Use LSM for Elevation: %s", useLSMforEl ? "YES" : "NO");
        }
        request->send(200, "text/plain", "OK");
    });

    // --- Jog (POST) ---
    webServer.on("/jog", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasParam("azStep", true)) {
            float azStep = request->getParam("azStep", true)->value().toFloat();
            moveAzimuthDeg(-1 * azStep);
            WEB_LOG_INFOF("WebUI","Jog AZ %f deg", azStep);
        }
        if (request->hasParam("elStep", true)) {
            float elStep = request->getParam("elStep", true)->value().toFloat();
            moveElevationDeg(-1 * elStep);
            WEB_LOG_INFOF("WebUI","Jog EL %f deg", elStep);
        }
        request->redirect("/"); 
    });

    // --- Move to absolute position ---
    webServer.on("/move", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasParam("moveAz", true)) {
            float az = request->getParam("moveAz", true)->value().toFloat();
            moveAzimuthToPosition(az);
            WEB_LOG_INFOF("WebUI","Move to AZ %f deg", az);
        }
        if (request->hasParam("moveEl", true)) {
            float el = request->getParam("moveEl", true)->value().toFloat();
            moveElevationToPosition(el);
            WEB_LOG_INFOF("WebUI","Move to EL %f deg", el);
        }
        request->redirect("/"); 
    });

    // --- Emergency Stop ---
    webServer.on("/estop", HTTP_POST, [](AsyncWebServerRequest *request) {
        emergencyStop();
        WEB_LOG_WARN("WebUI", "Emergency stop requested");
        request->send(200, "text/plain", "Stopped");
    });

    // --- Home endpoints ---
    webServer.on("/homeAz", HTTP_POST, [](AsyncWebServerRequest *request) {
        homeAzimuth();
        WEB_LOG_INFO("WebUI","Started homing AZ");
        request->send(200, "text/plain", "Homing Azimuth started");
    });
    webServer.on("/homeEl", HTTP_POST, [](AsyncWebServerRequest *request) {
        homeElevation();
        WEB_LOG_INFO("WebUI","Started homing EL");
        request->send(200, "text/plain", "Homing Elevation started");
    });

    // --- Logs ---
    webServer.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
        String logsJson = webLogger.getLogsAsJSON();
        request->send(200, "application/json", logsJson);
    });

    // --- Automatic Calibration ---
    webServer.on("/autoCal", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!calib.isRunning()) {
            calib.start();
            request->send(200, "text/plain", "Automatic Calibration started.");
        } else {
            request->send(200, "text/plain", "Calibration already running.");
        }
    });

    // --- Calibration status ---
    webServer.on("/cal/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"azMin\":" + String(lsmReceiver.getAzMin()) + ",";
        json += "\"azMax\":" + String(lsmReceiver.getAzMax()) + ",";
        json += "\"elMin\":" + String(lsmReceiver.getElMin()) + ",";
        json += "\"elMax\":" + String(lsmReceiver.getElMax());
        json += "}";
        request->send(200, "application/json", json);
    });

    // --- Reset ESP ---
    webServer.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "ESP32 is restarting...");
        delay(100);
        ESP.restart();

    });
   webServer.on("/tasks", HTTP_GET, [](AsyncWebServerRequest *request){
        // Buffer for FreeRTOS task list
        static char taskList[2048];  
        memset(taskList, 0, sizeof(taskList));

        // Generate task list into buffer
        vTaskList(taskList);

        // Send to client
        request->send(200, "text/plain", taskList);
    });


    // --- OTA support ---
    ElegantOTA.begin(&webServer);  // no password

    webServer.begin();
    WEB_LOG_INFO("WebServer", "Web server started on port 80");
}
