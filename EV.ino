#include "WiFiS3.h"

// =================================================================
// -------------------- PIN DEFINITIONS ----------------------------
// =================================================================
const int ENA = 9;   
const int IN1 = 7;   
const int IN2 = 8;   

const int ENC_A = 2; 
const int ENC_B = 3; 

const int RESET_SWITCH = 4;

// =================================================================
// -------------------- GLOBAL SETTINGS ----------------------------
// =================================================================

// --- CALIBRATION ---
float cmPerCount = 0.0712; 

// --- TUNING VARIABLES (Mutable via WiFi) ---
float targetDistanceCm = 700.0;
float targetTimeSeconds = 11.0; // 0.0 = Disabled (Max Speed), >0 = Target Time in Seconds
float slowDistanceCm   = 40.0;   
float deadbandCm       = 0.5;    // Stop if within this range

int initialMaxPWM  = 255;  
int approachMaxPWM = 220; 
int minStallPWM    = 160;  

float Kp = 30.0;   
float Ki = 1.5;    
float Kd = 5.0;    

// --- WIFI SETTINGS ---
char ssid[] = "Yash EV";
char pass[] = "";        // Open network
int status = WL_IDLE_STATUS;
WiFiServer server(80);

// =================================================================
// -------------------- SYSTEM STATE -------------------------------
// =================================================================
volatile long encoderCount = 0;

// Motor State
bool isRunning = false;
bool runCommand = false;
bool stopCommand = false;
bool resetCommand = false;

// PID State Variables
float integral = 0;
float lastError = 0;
float currentPwmOutput = 0; 

unsigned long lastPidTime = 0;
const float integralLimit = 300.0; 
unsigned long stableStartTime = 0;
unsigned long runStartTime = 0; // For tracking target time

// =================================================================
// -------------------- HTML INTERFACE -----------------------------
// =================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate" />
  <title>R4 Motor Control</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background: #121212; color: #e0e0e0; text-align: center; padding: 10px; margin: 0; }
    h2 { color: #03dac6; margin-bottom: 5px; }
    .card { background: #1e1e1e; padding: 15px; border-radius: 12px; margin-bottom: 15px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
    
    .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; font-size: 1.1rem; margin-bottom: 10px; }
    .val { color: #bb86fc; font-weight: bold; font-size: 1.3rem; }
    .label { color: #888; font-size: 0.8rem; text-transform: uppercase; }
    .sec-row .val { color: #03dac6; font-size: 1.1rem; }

    .bar-bg { width: 100%; height: 24px; background: #333; border-radius: 12px; overflow: hidden; margin: 10px 0; border: 1px solid #444; }
    .bar-fill { height: 100%; background: linear-gradient(90deg, #03dac6, #018786); width: 0%; transition: width 0.2s ease-out; }

    .input-group { display: flex; justify-content: space-between; align-items: center; margin: 8px 0; border-bottom: 1px solid #333; padding-bottom: 5px; }
    label { font-size: 0.9rem; color: #aaa; text-align: left; }
    input[type=number] { width: 80px; padding: 8px; background: #2c2c2c; color: white; border: 1px solid #444; border-radius: 6px; font-size: 1rem; font-weight: bold; text-align: center; }

    button { width: 100%; padding: 15px; font-size: 1.1rem; border: none; border-radius: 8px; color: white; cursor: pointer; margin-top: 5px; font-weight: bold; -webkit-tap-highlight-color: transparent; }
    .btn-row { display: flex; gap: 10px; margin-top: 10px; }
    .btn-start { background: #03dac6; color: #000; }
    .btn-stop { background: #cf6679; color: #000; }
    .btn-reset { background: #3700b3; }
    .btn-update { background: #bb86fc; color: #000; margin-top: 15px; }
    .btn-update:active { opacity: 0.8; }
  </style>
</head>
<body>
  <h2>Motor Commander v2</h2>
  
  <div class="card">
    <div class="data-grid">
      <div><div class="label">DISTANCE</div><span id="d" class="val">--</span> <small>cm</small></div>
      <div><div class="label">ERROR</div><span id="e" class="val">--</span> <small>cm</small></div>
    </div>
    <div class="data-grid sec-row">
      <div><div class="label">PWM OUT</div><span id="p" class="val">--</span></div>
      <div><div class="label">INTEGRAL</div><span id="i" class="val">--</span></div>
    </div>
    
    <div class="bar-bg"><div id="bar" class="bar-fill"></div></div>
    
    <div class="btn-row">
        <button class="btn-start" onclick="send('start')">START RUN</button>
        <button class="btn-stop" onclick="send('stop')">STOP & RESET</button>
    </div>
    <button class="btn-reset" onclick="send('reset')">ZERO ENCODER</button>
  </div>

  <div class="card">
    <h3>Calibration & Logic</h3>
    <div class="input-group"><label>Calib (cm/cnt):</label> <input type="number" id="cal" step="0.000001"></div>
    <div class="input-group"><label>Target (cm):</label> <input type="number" id="target"></div>
    <div class="input-group"><label>Time (s):</label> <input type="number" id="tt" step="0.1" placeholder="0=Fast"></div>
    <div class="input-group"><label>Deadband (cm):</label> <input type="number" id="db" step="0.1"></div>
    
    <h3 style="margin-top:20px; border-top: 1px solid #444; padding-top:10px;">PID & Speed</h3>
    <div class="input-group"><label>Kp (Prop):</label> <input type="number" id="kp" step="0.1"></div>
    <div class="input-group"><label>Ki (Integ):</label> <input type="number" id="ki" step="0.1"></div>
    <div class="input-group"><label>Kd (Deriv):</label> <input type="number" id="kd" step="0.1"></div>
    
    <div class="input-group"><label>Max PWM:</label> <input type="number" id="maxS"></div>
    <div class="input-group"><label>Appr PWM:</label> <input type="number" id="appS"></div>
    <div class="input-group"><label>Min PWM:</label> <input type="number" id="minS"></div>
    <div class="input-group"><label>Slow Dist:</label> <input type="number" id="slowD"></div>
    
    <button class="btn-update" onclick="updateParams()">APPLY SETTINGS</button>
  </div>

<script>
  window.onload = function() {
    fetch('/settings?v=' + Date.now()).then(r => r.json()).then(d => {
      document.getElementById('cal').value = d.c;
      document.getElementById('target').value = d.t;
      document.getElementById('tt').value = d.tt;
      document.getElementById('db').value = d.db;
      document.getElementById('kp').value = d.kp;
      document.getElementById('ki').value = d.ki;
      document.getElementById('kd').value = d.kd;
      document.getElementById('maxS').value = d.im;
      document.getElementById('appS').value = d.ap;
      document.getElementById('minS').value = d.mn;
      document.getElementById('slowD').value = d.sd;
    }).catch(err => console.log("Error loading settings:", err));

    setInterval(getData, 200); 
  };

  function getData() {
    fetch('/status').then(r => r.json()).then(d => {
      document.getElementById('d').innerText = d.d.toFixed(1);
      document.getElementById('e').innerText = d.e.toFixed(1);
      document.getElementById('p').innerText = d.p.toFixed(0);
      document.getElementById('i').innerText = d.i.toFixed(1);
      
      let t = parseFloat(document.getElementById('target').value) || 100;
      if (t === 0) t = 1;
      let pct = (d.d / t) * 100;
      if(pct>100) pct=100; if(pct<0) pct=0;
      document.getElementById('bar').style.width = pct + "%";
    }).catch(e => { });
  }

  function send(cmd) {
    fetch('/cmd?act=' + cmd);
  }

  function updateParams() {
    let q = "/set?";
    q += "t=" + document.getElementById('target').value;
    q += "&c=" + document.getElementById('cal').value;
    q += "&tt=" + document.getElementById('tt').value;
    q += "&db=" + document.getElementById('db').value;
    q += "&kp=" + document.getElementById('kp').value;
    q += "&ki=" + document.getElementById('ki').value;
    q += "&kd=" + document.getElementById('kd').value;
    q += "&im=" + document.getElementById('maxS').value;
    q += "&ap=" + document.getElementById('appS').value;
    q += "&mn=" + document.getElementById('minS').value;
    q += "&sd=" + document.getElementById('slowD').value;
    
    fetch(q).then(() => {
        let btn = document.activeElement;
        btn.innerText = "SAVED!";
        btn.style.background = "#03dac6";
        setTimeout(() => { 
            btn.innerText = "APPLY SETTINGS"; 
            btn.style.background = "#bb86fc"; 
        }, 1000);
    });
  }
</script>
</body>
</html>
)rawliteral";

// -------------------- FUNCTION DECLARATIONS --------------------
void readEncoder();
void setMotor(float pwm);
void brakeMotor();
void hardStopMotor();
float getDistanceCm();
void handleWiFi();
void runPID();
void startRun();
void stopRun();
void resetZero();

// =================================================================
// -------------------- SETUP --------------------------------------
// =================================================================
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);

  pinMode(RESET_SWITCH, INPUT_PULLUP);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi Module Failed!");
    while (true);
  }

  Serial.print("Creating AP: "); Serial.println(ssid);
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("AP Creation Failed");
    while (true);
  }
  
  server.begin();
  Serial.print("IP Address: http://"); Serial.println(WiFi.localIP());
}

// =================================================================
// -------------------- MAIN LOOP ----------------------------------
// =================================================================
void loop() {
  
  // Handle WiFi
  handleWiFi();

  // Physical button handling
  if (digitalRead(RESET_SWITCH) == LOW) {
    delay(50); 
    if (digitalRead(RESET_SWITCH) == LOW) {
      if (isRunning) stopCommand = true; 
      else runCommand = true;  
      while(digitalRead(RESET_SWITCH) == LOW); 
    }
  }

  if (runCommand) {
    // PROTECTIVE LOGIC: Only start if NOT already running.
    if (!isRunning) {
      startRun();
    }
    runCommand = false;
  }

  if (stopCommand) {
    stopRun();
    stopCommand = false;
  }

  if (resetCommand) {
    resetZero();
    resetCommand = false;
    Serial.println("CMD: Zero Reset");
  }

  if (isRunning) {
    runPID();
  }
}

// =================================================================
// -------------------- MOTOR LOGIC --------------------------------
// =================================================================

void startRun() {
  encoderCount = 0; 
  integral = 0;
  lastError = 0;
  lastPidTime = millis();
  runStartTime = millis(); // Record start time for time-based profile
  stableStartTime = 0;
  isRunning = true;
  Serial.println("CMD: Start (Zeroed)");
}

void stopRun() {
  isRunning = false;
  hardStopMotor();
  encoderCount = 0; 
  Serial.println("CMD: Stop (Zeroed)");
}

void resetZero() {
  stopRun();
  encoderCount = 0;
  integral = 0;
  lastError = 0;
}

void runPID() {
  unsigned long now = millis();
  
  if (now - lastPidTime < 5) return; 

  float dt = (now - lastPidTime) / 1000.0;
  if (dt > 0.05) dt = 0.05; 
  lastPidTime = now;

  float currentDist = getDistanceCm();
  
  // -----------------------------------------------------------
  // TIME-BASED RAMP LOGIC
  // -----------------------------------------------------------
  float effectiveSetPoint = targetDistanceCm;
  
  // Only apply ramp if targetTime is set (> 0)
  if (targetTimeSeconds > 0.01) {
    float elapsedSec = (now - runStartTime) / 1000.0;
    
    if (elapsedSec < targetTimeSeconds) {
      // Linear ramp: (ElapsedTime / TargetTime) * TotalDistance
      effectiveSetPoint = (elapsedSec / targetTimeSeconds) * targetDistanceCm;
    }
  }

  float error = effectiveSetPoint - currentDist;

  // Integral Windup Protection
  if ((lastError > 0 && error < 0) || (lastError < 0 && error > 0)) {
    integral = 0; 
  }
  integral += error * dt;
  integral = constrain(integral, -integralLimit, integralLimit);
  
  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  int currentSpeedLimit = initialMaxPWM;
  if (fabs(error) < slowDistanceCm) {
    currentSpeedLimit = approachMaxPWM;
  }
  output = constrain(output, -currentSpeedLimit, currentSpeedLimit);

  setMotor(output);

  // -----------------------------------------------------------
  // STOP CONDITION (Modified for Time)
  // -----------------------------------------------------------
  // Check if we are physically close to target AND (if time is set) time has passed
  bool timeConditionMet = (targetTimeSeconds <= 0.0) || ((now - runStartTime) >= (targetTimeSeconds * 1000));
  
  if (fabs(targetDistanceCm - currentDist) <= deadbandCm && timeConditionMet) { 
    if (stableStartTime == 0) stableStartTime = millis();
    else if (millis() - stableStartTime > 500) { 
      brakeMotor();
      delay(100); 
      hardStopMotor();
      isRunning = false;
      encoderCount = 0; 
      Serial.println("Target Reached. Resetting.");
    }
  } else {
    stableStartTime = 0;
  }
}

// =================================================================
// -------------------- WIFI HANDLER -------------------------------
// =================================================================
void handleWiFi() {
  WiFiClient client = server.available();
  if (!client) return;

  unsigned long startTime = millis();
  String currentLine = "";
  String request = "";
  
  while (client.connected()) {
    
    // Keep PID running
    if (isRunning) runPID(); 
    
    // Safety Timeout: Increased to 2000ms to allow full HTML page load
    if (millis() - startTime > 2000) { client.stop(); return; }

    if (client.available()) {
      char c = client.read();
      request += c;
      if (c == '\n') {
        if (currentLine.length() == 0) {
          
          // -----------------------------------------------------
          // COMMAND HANDLING (Optimized)
          // -----------------------------------------------------
          
          if (request.indexOf("GET /cmd?act=start") >= 0) {
            runCommand = true; 
            client.println("HTTP/1.1 200 OK");
            if(isRunning) runPID(); // Keep Motor Running
            client.println("Connection: close");
            client.println();
            break; 
          }
          else if (request.indexOf("GET /cmd?act=stop") >= 0) {
            stopCommand = true;
            client.println("HTTP/1.1 200 OK");
            client.println("Connection: close");
            client.println();
            break;
          }
          else if (request.indexOf("GET /cmd?act=reset") >= 0) {
            resetCommand = true;
            client.println("HTTP/1.1 200 OK");
            client.println("Connection: close");
            client.println();
            break;
          }
          else if (request.indexOf("GET /set?") >= 0) {
            // Apply Settings
            int idx;
            if((idx = request.indexOf("t=")) > 0) targetDistanceCm = request.substring(idx+2).toFloat();
            if((idx = request.indexOf("c=")) > 0) cmPerCount = request.substring(idx+2).toFloat();
            if((idx = request.indexOf("tt=")) > 0) targetTimeSeconds = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("db=")) > 0) deadbandCm = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("kp=")) > 0) Kp = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("ki=")) > 0) Ki = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("kd=")) > 0) Kd = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("im=")) > 0) initialMaxPWM = request.substring(idx+3).toInt();
            if((idx = request.indexOf("ap=")) > 0) approachMaxPWM = request.substring(idx+3).toInt();
            if((idx = request.indexOf("mn=")) > 0) minStallPWM = request.substring(idx+3).toInt();
            if((idx = request.indexOf("sd=")) > 0) slowDistanceCm = request.substring(idx+3).toFloat();
            
            client.println("HTTP/1.1 200 OK");
            if(isRunning) runPID(); // Keep Motor Running
            client.println("Connection: close");
            client.println();
            break;
          }
          else if (request.indexOf("GET /status") >= 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println("Connection: close");
            client.println();
            
            // --- CRITICAL FIX: Interleave PID inside the send block ---
            if(isRunning) runPID(); 

            client.print("{\"d\":"); client.print(getDistanceCm(), 2);
            client.print(",\"e\":"); client.print(targetDistanceCm - getDistanceCm(), 2);
            
            if(isRunning) runPID(); 

            client.print(",\"p\":"); client.print(currentPwmOutput);
            client.print(",\"i\":"); client.print(integral, 2);
            client.print("}");
            break; 
          }
          else if (request.indexOf("GET /settings") >= 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println("Connection: close");
            client.println();
            
            if(isRunning) runPID(); 

            client.print("{\"c\":"); client.print(cmPerCount, 6);
            client.print(",\"t\":"); client.print(targetDistanceCm);
            client.print(",\"tt\":"); client.print(targetTimeSeconds);
            client.print(",\"db\":"); client.print(deadbandCm);
            
            if(isRunning) runPID();

            client.print(",\"kp\":"); client.print(Kp);
            client.print(",\"ki\":"); client.print(Ki);
            client.print(",\"kd\":"); client.print(Kd);
            
            if(isRunning) runPID();

            client.print(",\"im\":"); client.print(initialMaxPWM);
            client.print(",\"ap\":"); client.print(approachMaxPWM);
            client.print(",\"mn\":"); client.print(minStallPWM);
            client.print(",\"sd\":"); client.print(slowDistanceCm);
            client.print("}");
            break;
          }
          else {
            // Serve HTML
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");
            client.println("Cache-Control: no-store, no-cache, must-revalidate");
            client.println();
            
            int len = strlen_P(index_html);
            for (int k = 0; k < len; k += 100) {
               if(isRunning) runPID(); 
               int remaining = len - k;
               int chunk = (remaining < 100) ? remaining : 100;
               char buffer[101];
               memcpy_P(buffer, index_html + k, chunk);
               buffer[chunk] = 0;
               client.print(buffer);
            }
            break;
          }
        } else {
          currentLine = "";
        }
      } else if (c != '\r') {
        currentLine += c;
      }
    }
  }
  delay(1); 
  client.stop();
}

// =================================================================
// -------------------- HARDWARE DRIVERS ---------------------------
// =================================================================
void readEncoder() {
  int A = digitalRead(ENC_A);
  int B = digitalRead(ENC_B);
  if (A == B) encoderCount++; else encoderCount--;
}

float getDistanceCm() {
  long counts;
  noInterrupts();
  counts = encoderCount;
  interrupts();
  return counts * cmPerCount;
}

void setMotor(float pwm) {
  // Clamp
  if (pwm > 0 && pwm < minStallPWM) pwm = minStallPWM;
  if (pwm < 0 && pwm > -minStallPWM) pwm = -minStallPWM;
  pwm = constrain(pwm, -255, 255);

  currentPwmOutput = pwm;

  // REVERSED FORWARD LOGIC
  if (pwm > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, (int)pwm);
  } else if (pwm < 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, (int)-pwm);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  }
}

void brakeMotor() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); analogWrite(ENA, 255);
  currentPwmOutput = 0;
}

void hardStopMotor() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  currentPwmOutput = 0;
}