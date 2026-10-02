#include <Arduino.h>
#include <WiFiS3.h>

// =================================================================
// -------------------- PIN DEFINITIONS (UNO R4) -------------------
// =================================================================
const int ENA = 9;   // Motor PWM
const int IN1 = 7;   // Motor Direction 1
const int IN2 = 8;   // Motor Direction 2

const int ENC_A = 2; // Encoder Phase A (Interrupt)
const int ENC_B = 3; // Encoder Phase B

const int START_SWITCH = 4; // #2 Pencil actuation switch (INPUT_PULLUP)

// =================================================================
// -------------------- COMPETITION SETTINGS -----------------------
// =================================================================

// --- CALIBRATION ---
float cmPerCount = 0.0712; 

// --- EVENT SUPERVISOR VARIABLES ---
float targetDistanceCm     = 700.0; // Distance to Target Point
float bottleLineDistanceCm = 100.0; // Distance from Target Point to Bottle Line
float targetTimeSeconds    = 14.0;  // Exact target time for the run

// --- STRATEGY VARIABLES ---
float estTravelTimeSeconds = 4.5;   // Calibration: How long the actual physical movement takes
float pushClearanceCm      = 15.0;  // Extra distance to ensure bottle is fully past the line
float deadbandCm           = 0.5;   // Acceptable error before considering state settled

// --- PID TUNING ---
float Kp = 35.0;   
float Ki = 2.0;    
float Kd = 8.0;    

int initialMaxPWM  = 255;  
int approachMaxPWM = 180; 
int minStallPWM    = 100;  

// --- WIFI SETTINGS ---
char ssid[] = "Yash EV";
char pass[] = "";        
int status = WL_IDLE_STATUS;
WiFiServer server(80);

// =================================================================
// -------------------- SYSTEM STATE -------------------------------
// =================================================================
volatile long encoderCount = 0;

enum RunState {
  STATE_IDLE,
  STATE_DELAY,
  STATE_FORWARD,
  STATE_REVERSE,
  STATE_STOPPED
};

RunState currentState = STATE_IDLE;
bool runCommand = false;

// PID Variables
float integral = 0;
float lastError = 0;
float currentPwmOutput = 0; 
unsigned long lastPidTime = 0;
const float integralLimit = 300.0; 

// Timing Variables
unsigned long runStartTime = 0; 
unsigned long stateStartTime = 0;
unsigned long settleStartTime = 0;
float calculatedDelaySeconds = 0;

// =================================================================
// -------------------- HTML INTERFACE -----------------------------
// =================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>SciOly EV Commander</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #121212; color: #e0e0e0; text-align: center; padding: 10px; margin: 0; }
    h2 { color: #03dac6; margin-bottom: 5px; }
    .card { background: #1e1e1e; padding: 15px; border-radius: 12px; margin-bottom: 15px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
    
    .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; font-size: 1.1rem; margin-bottom: 10px; }
    .val { color: #bb86fc; font-weight: bold; font-size: 1.3rem; }
    .label { color: #888; font-size: 0.8rem; text-transform: uppercase; }
    
    .status-box { background: #333; color: #fff; padding: 10px; border-radius: 8px; font-weight: bold; font-size: 1.2rem; letter-spacing: 2px; margin-bottom: 10px; }

    .input-group { display: flex; justify-content: space-between; align-items: center; margin: 8px 0; border-bottom: 1px solid #333; padding-bottom: 5px; }
    label { font-size: 0.95rem; color: #aaa; text-align: left; flex: 1; }
    input[type=number] { width: 90px; padding: 8px; background: #2c2c2c; color: white; border: 1px solid #444; border-radius: 6px; font-size: 1rem; font-weight: bold; text-align: center; }

    button { width: 100%; padding: 15px; font-size: 1.1rem; border: none; border-radius: 8px; color: white; cursor: pointer; margin-top: 5px; font-weight: bold; }
    .btn-row { display: flex; gap: 10px; margin-top: 10px; }
    .btn-start { background: #03dac6; color: #000; }
    .btn-stop { background: #cf6679; color: #000; }
    .btn-update { background: #bb86fc; color: #000; margin-top: 15px; }
    .btn-update:active { opacity: 0.8; }
    
    h3 { margin-top:20px; border-top: 1px solid #444; padding-top:10px; color: #03dac6; font-size: 1.1rem; }
  </style>
</head>
<body>
  <h2>SciOly EV Commander</h2>
  
  <div class="card">
    <div id="status" class="status-box">IDLE</div>
    <div class="data-grid">
      <div><div class="label">DISTANCE</div><span id="d" class="val">--</span> <small>cm</small></div>
      <div><div class="label">ERROR</div><span id="e" class="val">--</span> <small>cm</small></div>
      <div><div class="label">TIME</div><span id="time" class="val">--</span> <small>s</small></div>
      <div><div class="label">PWM</div><span id="p" class="val">--</span></div>
    </div>
    
    <div class="btn-row">
        <button class="btn-start" onclick="send('start')">START RUN</button>
        <button class="btn-stop" onclick="send('stop')">ABORT</button>
    </div>
  </div>

  <div class="card">
    <h3>Competition Setup</h3>
    <div class="input-group"><label>Target Dist (cm):</label> <input type="number" id="td"></div>
    <div class="input-group"><label>Bottle Line Dist (cm):</label> <input type="number" id="bld"></div>
    <div class="input-group"><label>Target Time (s):</label> <input type="number" id="tt" step="0.1"></div>
    
    <h3>Strategy Tuning</h3>
    <div class="input-group"><label>Est Travel Time (s):</label> <input type="number" id="ett" step="0.1"></div>
    <div class="input-group"><label>Push Clearance (cm):</label> <input type="number" id="pc"></div>
    <div class="input-group"><label>Calib (cm/cnt):</label> <input type="number" id="cal" step="0.000001"></div>
    <div class="input-group"><label>Deadband (cm):</label> <input type="number" id="db" step="0.1"></div>
    
    <h3>PID Setup</h3>
    <div class="input-group"><label>Kp (Prop):</label> <input type="number" id="kp" step="0.1"></div>
    <div class="input-group"><label>Ki (Integ):</label> <input type="number" id="ki" step="0.1"></div>
    <div class="input-group"><label>Kd (Deriv):</label> <input type="number" id="kd" step="0.1"></div>
    <div class="input-group"><label>Max PWM:</label> <input type="number" id="maxS"></div>
    <div class="input-group"><label>Min PWM:</label> <input type="number" id="minS"></div>
    
    <button class="btn-update" onclick="updateParams()">APPLY & SAVE</button>
  </div>

<script>
  const states = ["IDLE", "DELAYING...", "FORWARD PUSH", "REVERSING", "FINISHED"];

  window.onload = function() {
    fetch('/settings?v=' + Date.now()).then(r => r.json()).then(d => {
      document.getElementById('td').value = d.td;
      document.getElementById('bld').value = d.bld;
      document.getElementById('tt').value = d.tt;
      document.getElementById('ett').value = d.ett;
      document.getElementById('pc').value = d.pc;
      document.getElementById('cal').value = d.c;
      document.getElementById('db').value = d.db;
      document.getElementById('kp').value = d.kp;
      document.getElementById('ki').value = d.ki;
      document.getElementById('kd').value = d.kd;
      document.getElementById('maxS').value = d.im;
      document.getElementById('minS').value = d.mn;
    });
    setInterval(getData, 200); 
  };

  function getData() {
    fetch('/status').then(r => r.json()).then(d => {
      document.getElementById('d').innerText = d.d.toFixed(1);
      document.getElementById('e').innerText = d.e.toFixed(1);
      document.getElementById('p').innerText = d.p.toFixed(0);
      document.getElementById('time').innerText = d.t.toFixed(1);
      document.getElementById('status').innerText = states[d.s];
    }).catch(e => { });
  }

  function send(cmd) { fetch('/cmd?act=' + cmd); }

  function updateParams() {
    let q = "/set?";
    q += "td=" + document.getElementById('td').value;
    q += "&bld=" + document.getElementById('bld').value;
    q += "&tt=" + document.getElementById('tt').value;
    q += "&ett=" + document.getElementById('ett').value;
    q += "&pc=" + document.getElementById('pc').value;
    q += "&c=" + document.getElementById('cal').value;
    q += "&db=" + document.getElementById('db').value;
    q += "&kp=" + document.getElementById('kp').value;
    q += "&ki=" + document.getElementById('ki').value;
    q += "&kd=" + document.getElementById('kd').value;
    q += "&im=" + document.getElementById('maxS').value;
    q += "&mn=" + document.getElementById('minS').value;
    
    fetch(q).then(() => {
        let btn = document.activeElement;
        btn.innerText = "SAVED!";
        btn.style.background = "#03dac6";
        setTimeout(() => { btn.innerText = "APPLY & SAVE"; btn.style.background = "#bb86fc"; }, 1000);
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
void runStateMachine();
void startRun();
void stopRun();

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

  pinMode(START_SWITCH, INPUT_PULLUP);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi Module Failed!");
    while (true);
  }

  Serial.print("Creating AP: "); Serial.println(ssid);
  status = WiFi.beginAP(ssid, pass);
  
  server.begin();
  Serial.print("SciOly EV Ready! IP: http://"); Serial.println(WiFi.localIP());
}

// =================================================================
// -------------------- MAIN LOOP ----------------------------------
// =================================================================
void loop() {
  handleWiFi();

  // Actuation via pencil
  if (digitalRead(START_SWITCH) == LOW) {
    delay(50); // Debounce
    if (digitalRead(START_SWITCH) == LOW) {
      if (currentState == STATE_IDLE || currentState == STATE_STOPPED) {
        runCommand = true;
      } else {
        stopRun(); // Abort if pressed during run
      }
      while(digitalRead(START_SWITCH) == LOW); 
    }
  }

  if (runCommand) {
    startRun();
    runCommand = false;
  }

  runStateMachine();
}

// =================================================================
// -------------------- STATE MACHINE & ALGORITHMS -----------------
// =================================================================

void startRun() {
  encoderCount = 0; 
  integral = 0;
  lastError = 0;
  
  // Calculate the strategic delay needed to hit exactly the target time
  calculatedDelaySeconds = targetTimeSeconds - estTravelTimeSeconds;
  if (calculatedDelaySeconds < 0) calculatedDelaySeconds = 0;
  
  runStartTime = millis();
  stateStartTime = millis();
  currentState = STATE_DELAY;
  
  Serial.print("Run Started! Waiting "); 
  Serial.print(calculatedDelaySeconds); 
  Serial.println(" seconds.");
}

void stopRun() {
  currentState = STATE_STOPPED;
  brakeMotor();
  Serial.println("Run Aborted/Stopped.");
}

void runStateMachine() {
  if (currentState == STATE_IDLE || currentState == STATE_STOPPED) return;

  unsigned long now = millis();
  float currentDist = getDistanceCm();
  float setpoint = 0;

  // --- STATE: DELAY (Time Algorithm) ---
  if (currentState == STATE_DELAY) {
    if (now - stateStartTime >= (calculatedDelaySeconds * 1000.0)) {
      currentState = STATE_FORWARD;
      stateStartTime = now;
      settleStartTime = 0;
      Serial.println("Delay Complete. FORWARD PUSH initiated.");
    }
    return; // Wait completely still
  }

  // --- DETERMINE SETPOINT ---
  if (currentState == STATE_FORWARD) {
    setpoint = targetDistanceCm + bottleLineDistanceCm + pushClearanceCm;
  } else if (currentState == STATE_REVERSE) {
    setpoint = targetDistanceCm;
  }

  // --- PID CALCULATION ---
  if (now - lastPidTime < 5) return; 
  float dt = (now - lastPidTime) / 1000.0;
  if (dt > 0.05) dt = 0.05; 
  lastPidTime = now;

  float error = setpoint - currentDist;

  // Anti-windup
  if ((lastError > 0 && error < 0) || (lastError < 0 && error > 0)) { integral = 0; }
  integral += error * dt;
  integral = constrain(integral, -integralLimit, integralLimit);
  
  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // Speed ramping / approach limits
  int currentSpeedLimit = initialMaxPWM;
  if (fabs(error) < 40.0) { currentSpeedLimit = approachMaxPWM; }
  output = constrain(output, -currentSpeedLimit, currentSpeedLimit);

  setMotor(output);

  // --- STATE TRANSITIONS ---
  if (fabs(error) <= deadbandCm) {
    if (settleStartTime == 0) settleStartTime = now;
    else if (now - settleStartTime > 300) { // Stable for 300ms
      
      if (currentState == STATE_FORWARD) {
        brakeMotor();
        delay(100);
        currentState = STATE_REVERSE;
        stateStartTime = now;
        settleStartTime = 0;
        integral = 0; // Reset PID for reverse
        Serial.println("Forward Target Reached. REVERSING initiated.");
      } 
      else if (currentState == STATE_REVERSE) {
        brakeMotor();
        currentState = STATE_STOPPED;
        Serial.println("Final Target Reached. COMPLETE STOP.");
      }
    }
  } else {
    settleStartTime = 0;
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
    runStateMachine(); 
    if (millis() - startTime > 1000) { client.stop(); return; } // Reduced timeout for faster loop

    if (client.available()) {
      char c = client.read();
      request += c;
      if (c == '\n') {
        if (currentLine.length() == 0) {
          
          if (request.indexOf("GET /cmd?act=start") >= 0) {
            runCommand = true; 
            client.println("HTTP/1.1 200 OK\nConnection: close\n"); break; 
          }
          else if (request.indexOf("GET /cmd?act=stop") >= 0) {
            stopRun();
            client.println("HTTP/1.1 200 OK\nConnection: close\n"); break;
          }
          else if (request.indexOf("GET /set?") >= 0) {
            int idx;
            if((idx = request.indexOf("td=")) > 0) targetDistanceCm = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("bld=")) > 0) bottleLineDistanceCm = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("tt=")) > 0) targetTimeSeconds = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("ett=")) > 0) estTravelTimeSeconds = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("pc=")) > 0) pushClearanceCm = request.substring(idx+3).toFloat();
            
            if((idx = request.indexOf("c=")) > 0) cmPerCount = request.substring(idx+2).toFloat();
            if((idx = request.indexOf("db=")) > 0) deadbandCm = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("kp=")) > 0) Kp = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("ki=")) > 0) Ki = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("kd=")) > 0) Kd = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("im=")) > 0) initialMaxPWM = request.substring(idx+3).toInt();
            if((idx = request.indexOf("mn=")) > 0) minStallPWM = request.substring(idx+3).toInt();
            
            client.println("HTTP/1.1 200 OK\nConnection: close\n"); break;
          }
          else if (request.indexOf("GET /status") >= 0) {
            client.println("HTTP/1.1 200 OK\nContent-Type: application/json\nConnection: close\n");
            
            float err = 0;
            if (currentState == STATE_FORWARD) err = (targetDistanceCm + bottleLineDistanceCm + pushClearanceCm) - getDistanceCm();
            else if (currentState == STATE_REVERSE || currentState == STATE_STOPPED) err = targetDistanceCm - getDistanceCm();
            
            float t = 0;
            if (currentState != STATE_IDLE) t = (millis() - runStartTime) / 1000.0;

            client.print("{\"d\":"); client.print(getDistanceCm(), 2);
            client.print(",\"e\":"); client.print(err, 2);
            client.print(",\"p\":"); client.print(currentPwmOutput);
            client.print(",\"t\":"); client.print(t, 1);
            client.print(",\"s\":"); client.print(currentState);
            client.print("}");
            break; 
          }
          else if (request.indexOf("GET /settings") >= 0) {
            client.println("HTTP/1.1 200 OK\nContent-Type: application/json\nConnection: close\n");
            client.print("{\"td\":"); client.print(targetDistanceCm);
            client.print(",\"bld\":"); client.print(bottleLineDistanceCm);
            client.print(",\"tt\":"); client.print(targetTimeSeconds);
            client.print(",\"ett\":"); client.print(estTravelTimeSeconds);
            client.print(",\"pc\":"); client.print(pushClearanceCm);
            client.print(",\"c\":"); client.print(cmPerCount, 6);
            client.print(",\"db\":"); client.print(deadbandCm);
            client.print(",\"kp\":"); client.print(Kp);
            client.print(",\"ki\":"); client.print(Ki);
            client.print(",\"kd\":"); client.print(Kd);
            client.print(",\"im\":"); client.print(initialMaxPWM);
            client.print(",\"mn\":"); client.print(minStallPWM);
            client.print("}");
            break;
          }
          else {
            client.println("HTTP/1.1 200 OK\nContent-Type: text/html\nConnection: close\nCache-Control: no-store\n");
            int len = strlen_P(index_html);
            for (int k = 0; k < len; k += 100) {
               runStateMachine(); 
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
  if (currentState == STATE_IDLE || currentState == STATE_STOPPED || currentState == STATE_DELAY) {
    hardStopMotor(); return;
  }

  // Overcome stall/friction
  if (pwm > 0 && pwm < minStallPWM) pwm = minStallPWM;
  if (pwm < 0 && pwm > -minStallPWM) pwm = -minStallPWM;
  pwm = constrain(pwm, -255, 255);

  currentPwmOutput = pwm;

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
