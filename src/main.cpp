#include <Arduino.h>
#include <WiFiS3.h>
#include <EEPROM.h>

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
// -------------------- PERSISTENT SETTINGS ------------------------
// =================================================================
#define EEPROM_MAGIC 0xEE0002 // Change this to force a factory reset

struct Settings {
  uint32_t magic;
  
  // Competition
  float targetDistanceCm;
  float bottleLineDistanceCm;
  float targetTimeSeconds;
  
  // Tuning & Strategy
  float estTravelTimeSeconds;
  float pushClearanceCm;
  float deadbandCm;
  float cmPerCount;
  int settleTimeMs;
  
  // PID & Motor
  float Kp;
  float Ki;
  float Kd;
  int maxPWM;
  int minPWM;
  float maxAccelRamp; // Traction control: max PWM change per PID cycle
} cfg;

void loadDefaultSettings() {
  cfg.magic = EEPROM_MAGIC;
  cfg.targetDistanceCm = 700.0;
  cfg.bottleLineDistanceCm = 100.0;
  cfg.targetTimeSeconds = 14.0;
  cfg.estTravelTimeSeconds = 4.5;
  cfg.pushClearanceCm = 15.0;
  cfg.deadbandCm = 0.5;
  cfg.cmPerCount = 0.0712;
  cfg.settleTimeMs = 300;
  cfg.Kp = 35.0;
  cfg.Ki = 2.0;
  cfg.Kd = 8.0;
  cfg.maxPWM = 255;
  cfg.minPWM = 100;
  cfg.maxAccelRamp = 20.0; // Prevents wheel spin
}

void loadSettings() {
  EEPROM.get(0, cfg);
  if (cfg.magic != EEPROM_MAGIC) {
    Serial.println("No valid EEPROM found. Loading defaults.");
    loadDefaultSettings();
    EEPROM.put(0, cfg);
  } else {
    Serial.println("Settings loaded from EEPROM.");
  }
}

void saveSettings() {
  EEPROM.put(0, cfg);
  Serial.println("Settings saved to EEPROM.");
}

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

// PID & Motion Variables
float integral = 0;
float lastError = 0;
float currentPwmOutput = 0; 
float setpoint = 0;
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
  <title>SciOly EV Commander Pro</title>
  <style>
    :root { --bg: #121212; --card: #1e1e1e; --primary: #03dac6; --secondary: #bb86fc; --danger: #cf6679; --text: #e0e0e0; }
    body { font-family: -apple-system, sans-serif; background: var(--bg); color: var(--text); padding: 10px; margin: 0; }
    h2 { color: var(--primary); margin: 0 0 15px 0; text-align: center; }
    
    /* Tabs */
    .tab-bar { display: flex; margin-bottom: 10px; background: #333; border-radius: 8px; overflow: hidden; }
    .tab { flex: 1; padding: 12px 0; text-align: center; cursor: pointer; font-weight: bold; color: #888; transition: 0.2s; }
    .tab.active { background: var(--secondary); color: #000; }
    
    .tab-content { display: none; }
    .tab-content.active { display: block; }

    /* Cards & Layout */
    .card { background: var(--card); padding: 15px; border-radius: 12px; margin-bottom: 15px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
    .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; text-align: center; margin-bottom: 15px; }
    .val { color: var(--secondary); font-weight: bold; font-size: 1.4rem; }
    .label { color: #888; font-size: 0.75rem; text-transform: uppercase; letter-spacing: 1px; }
    
    .status-box { background: #333; color: #fff; padding: 12px; border-radius: 8px; font-weight: bold; font-size: 1.2rem; letter-spacing: 2px; text-align: center; margin-bottom: 15px; }
    
    /* Graph */
    #graph-container { width: 100%; height: 150px; background: #2c2c2c; border-radius: 8px; margin-bottom: 15px; position: relative; }
    canvas { width: 100%; height: 100%; }

    /* Forms */
    .input-group { display: flex; justify-content: space-between; align-items: center; margin: 8px 0; border-bottom: 1px solid #333; padding-bottom: 6px; }
    label { font-size: 0.95rem; color: #aaa; }
    input[type=number] { width: 80px; padding: 6px; background: #2c2c2c; color: white; border: 1px solid #444; border-radius: 6px; font-weight: bold; text-align: center; }

    /* Buttons */
    button { width: 100%; padding: 14px; font-size: 1rem; border: none; border-radius: 8px; color: white; cursor: pointer; font-weight: bold; transition: 0.1s; }
    button:active { transform: scale(0.98); }
    .btn-row { display: flex; gap: 10px; }
    .btn-start { background: var(--primary); color: #000; }
    .btn-stop { background: var(--danger); color: #000; }
    .btn-save { background: var(--secondary); color: #000; margin-top: 10px; }
    .btn-eeprom { background: #3700b3; margin-top: 10px; }
    
    h3 { color: var(--primary); font-size: 1.1rem; border-bottom: 1px solid #444; padding-bottom: 5px; margin-top: 5px; }
  </style>
</head>
<body>
  <h2>EV Commander Pro</h2>
  
  <div class="tab-bar">
    <div class="tab active" onclick="setTab('dash')">Dash</div>
    <div class="tab" onclick="setTab('comp')">Comp</div>
    <div class="tab" onclick="setTab('tune')">Tuning</div>
  </div>

  <!-- DASHBOARD TAB -->
  <div id="dash" class="tab-content active">
    <div class="card">
      <div id="status" class="status-box">IDLE</div>
      
      <div id="graph-container"><canvas id="graph"></canvas></div>

      <div class="data-grid">
        <div><div class="label">DISTANCE</div><span id="d" class="val">0.0</span><small>cm</small></div>
        <div><div class="label">ERROR</div><span id="e" class="val">0.0</span><small>cm</small></div>
        <div><div class="label">RUN TIME</div><span id="time" class="val">0.0</span><small>s</small></div>
        <div><div class="label">PWM</div><span id="p" class="val">0</span></div>
      </div>
      
      <div class="btn-row">
          <button class="btn-start" onclick="send('start')">START</button>
          <button class="btn-stop" onclick="send('stop')">ABORT</button>
      </div>
    </div>
  </div>

  <!-- COMPETITION TAB -->
  <div id="comp" class="tab-content">
    <div class="card">
      <h3>Event Setup</h3>
      <div class="input-group"><label>Target Dist (cm):</label> <input type="number" id="td"></div>
      <div class="input-group"><label>Bottle Line Dist (cm):</label> <input type="number" id="bld"></div>
      <div class="input-group"><label>Target Time (s):</label> <input type="number" id="tt" step="0.1"></div>
      
      <h3>Strategy</h3>
      <div class="input-group"><label>Est Travel Time (s):</label> <input type="number" id="ett" step="0.1"></div>
      <div class="input-group"><label>Push Clearance (cm):</label> <input type="number" id="pc"></div>
      
      <button class="btn-save" onclick="updateParams()">APPLY CHANGES</button>
      <button class="btn-eeprom" onclick="send('save')">SAVE TO MEMORY (EEPROM)</button>
    </div>
  </div>

  <!-- TUNING TAB -->
  <div id="tune" class="tab-content">
    <div class="card">
      <h3>PID & Traction</h3>
      <div class="input-group"><label>Kp (Prop):</label> <input type="number" id="kp" step="0.1"></div>
      <div class="input-group"><label>Ki (Integ):</label> <input type="number" id="ki" step="0.1"></div>
      <div class="input-group"><label>Kd (Deriv):</label> <input type="number" id="kd" step="0.1"></div>
      <div class="input-group"><label>Traction (PWM Ramp):</label> <input type="number" id="ramp" step="1"></div>
      
      <h3>Motor & Hardware</h3>
      <div class="input-group"><label>Max PWM:</label> <input type="number" id="maxS"></div>
      <div class="input-group"><label>Min PWM:</label> <input type="number" id="minS"></div>
      <div class="input-group"><label>Calib (cm/cnt):</label> <input type="number" id="cal" step="0.000001"></div>
      
      <h3>Settling</h3>
      <div class="input-group"><label>Deadband (cm):</label> <input type="number" id="db" step="0.1"></div>
      <div class="input-group"><label>Settle Time (ms):</label> <input type="number" id="stm"></div>
      
      <button class="btn-save" onclick="updateParams()">APPLY CHANGES</button>
      <button class="btn-eeprom" onclick="send('save')">SAVE TO MEMORY (EEPROM)</button>
    </div>
  </div>

<script>
  const states = ["IDLE", "DELAYING...", "FORWARD PUSH", "REVERSING", "FINISHED"];
  
  // Graphing Engine
  const canvas = document.getElementById('graph');
  const ctx = canvas.getContext('2d');
  let history = [];
  const MAX_PTS = 50;

  function setTab(id) {
    document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
    document.querySelectorAll('.tab').forEach(el => el.classList.remove('active'));
    document.getElementById(id).classList.add('active');
    event.currentTarget.classList.add('active');
  }

  window.onload = function() {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = canvas.parentElement.clientHeight;
    
    fetch('/settings?v=' + Date.now()).then(r => r.json()).then(d => {
      document.getElementById('td').value = d.td; document.getElementById('bld').value = d.bld;
      document.getElementById('tt').value = d.tt; document.getElementById('ett').value = d.ett;
      document.getElementById('pc').value = d.pc; document.getElementById('cal').value = d.c;
      document.getElementById('db').value = d.db; document.getElementById('kp').value = d.kp;
      document.getElementById('ki').value = d.ki; document.getElementById('kd').value = d.kd;
      document.getElementById('maxS').value = d.im; document.getElementById('minS').value = d.mn;
      document.getElementById('ramp').value = d.rmp; document.getElementById('stm').value = d.st;
    });
    setInterval(getData, 200); 
  };

  function drawGraph() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if(history.length < 2) return;
    
    let maxD = Math.max(800, ...history.map(h => Math.max(h.d, h.sp)));
    let minD = 0;
    
    // Draw Setpoint
    ctx.beginPath();
    ctx.strokeStyle = '#cf6679'; ctx.lineWidth = 2; ctx.setLineDash([5, 5]);
    history.forEach((h, i) => {
      let x = (i / (MAX_PTS-1)) * canvas.width;
      let y = canvas.height - ((h.sp - minD) / (maxD - minD)) * canvas.height;
      if(i===0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // Draw Actual Distance
    ctx.beginPath();
    ctx.strokeStyle = '#03dac6'; ctx.lineWidth = 3; ctx.setLineDash([]);
    history.forEach((h, i) => {
      let x = (i / (MAX_PTS-1)) * canvas.width;
      let y = canvas.height - ((h.d - minD) / (maxD - minD)) * canvas.height;
      if(i===0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  function getData() {
    fetch('/status').then(r => r.json()).then(d => {
      document.getElementById('d').innerText = d.d.toFixed(1);
      document.getElementById('e').innerText = d.e.toFixed(1);
      document.getElementById('p').innerText = d.p.toFixed(0);
      document.getElementById('time').innerText = d.t.toFixed(1);
      document.getElementById('status').innerText = states[d.s];
      
      history.push({d: d.d, sp: d.sp});
      if(history.length > MAX_PTS) history.shift();
      drawGraph();
    }).catch(e => { });
  }

  function send(cmd) { 
    fetch('/cmd?act=' + cmd).then(() => {
        if(cmd==='save') alert("Saved to EEPROM!");
    }); 
  }

  function updateParams() {
    let q = "/set?&td=" + document.getElementById('td').value +
            "&bld=" + document.getElementById('bld').value +
            "&tt=" + document.getElementById('tt').value +
            "&ett=" + document.getElementById('ett').value +
            "&pc=" + document.getElementById('pc').value +
            "&c=" + document.getElementById('cal').value +
            "&db=" + document.getElementById('db').value +
            "&kp=" + document.getElementById('kp').value +
            "&ki=" + document.getElementById('ki').value +
            "&kd=" + document.getElementById('kd').value +
            "&im=" + document.getElementById('maxS').value +
            "&mn=" + document.getElementById('minS').value +
            "&rmp=" + document.getElementById('ramp').value +
            "&st=" + document.getElementById('stm').value;
    
    fetch(q).then(() => {
        let btns = document.querySelectorAll('.btn-save');
        btns.forEach(b => { b.innerText = "UPDATED!"; b.style.background = "#03dac6"; });
        setTimeout(() => { btns.forEach(b => { b.innerText = "APPLY CHANGES"; b.style.background = "#bb86fc"; }); }, 1000);
    });
  }
</script>
</body>
</html>
)rawliteral";

// -------------------- FUNCTION DECLARATIONS --------------------
void readEncoder();
void setMotor(float targetPwm);
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

  // 1. Give the ESP32-S3 coprocessor time to boot reliably on cold power-up
  delay(1500); 

  // Load persistence
  loadSettings();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);

  pinMode(START_SWITCH, INPUT_PULLUP);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi Module Failed! Running offline.");
  } else {
    // 2. Clear out any hung state from a previous brown-out or reset
    WiFi.disconnect();
    delay(500);

    // 3. Explicitly configure the AP's IP to fix Android/iOS "Connection Failure" / DHCP issues
    IPAddress local_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(local_ip, gateway, subnet);

    Serial.print("Creating AP: "); Serial.println(ssid);
    
    // 4. Retry loop (sometimes the ESP32 needs a second attempt)
    for (int i = 0; i < 3; i++) {
      if (strlen(pass) == 0) {
        status = WiFi.beginAP(ssid); // Correct way to make an open network
      } else {
        status = WiFi.beginAP(ssid, pass); // WPA2 requires 8+ characters
      }
      if (status == WL_AP_LISTENING) break;
      Serial.println("AP Creation Failed, retrying...");
      delay(1000);
    }

    if (status == WL_AP_LISTENING) {
      server.begin();
      Serial.print("SciOly EV Ready! IP: http://"); Serial.println(WiFi.localIP());
    } else {
      Serial.println("Fatal: Could not create AP. Running offline mode.");
    }
  }
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
      if (currentState == STATE_IDLE || currentState == STATE_STOPPED) runCommand = true;
      else stopRun(); // Abort
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
  currentPwmOutput = 0;
  setpoint = 0;
  
  // Calculate strategic delay
  calculatedDelaySeconds = cfg.targetTimeSeconds - cfg.estTravelTimeSeconds;
  if (calculatedDelaySeconds < 0) calculatedDelaySeconds = 0;
  
  runStartTime = millis();
  stateStartTime = millis();
  currentState = STATE_DELAY;
  
  Serial.print("Run Started! Waiting "); Serial.print(calculatedDelaySeconds); Serial.println("s.");
}

void stopRun() {
  currentState = STATE_STOPPED;
  setpoint = getDistanceCm(); // Reset setpoint to current so graph looks correct
  brakeMotor();
  Serial.println("Run Aborted/Stopped.");
}

void runStateMachine() {
  if (currentState == STATE_IDLE || currentState == STATE_STOPPED) return;

  unsigned long now = millis();
  float currentDist = getDistanceCm();

  // --- STATE: DELAY (Time Algorithm) ---
  if (currentState == STATE_DELAY) {
    if (now - stateStartTime >= (calculatedDelaySeconds * 1000.0)) {
      currentState = STATE_FORWARD;
      stateStartTime = now;
      settleStartTime = 0;
      Serial.println("Delay Complete. FORWARD PUSH initiated.");
    }
    return; 
  }

  // --- DETERMINE SETPOINT ---
  if (currentState == STATE_FORWARD) {
    setpoint = cfg.targetDistanceCm + cfg.bottleLineDistanceCm + cfg.pushClearanceCm;
  } else if (currentState == STATE_REVERSE) {
    setpoint = cfg.targetDistanceCm;
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

  float output = cfg.Kp * error + cfg.Ki * integral + cfg.Kd * derivative;
  output = constrain(output, -cfg.maxPWM, cfg.maxPWM);

  setMotor(output);

  // --- STATE TRANSITIONS ---
  if (fabs(error) <= cfg.deadbandCm) {
    if (settleStartTime == 0) settleStartTime = now;
    else if (now - settleStartTime > cfg.settleTimeMs) { // Settled
      
      if (currentState == STATE_FORWARD) {
        brakeMotor();
        delay(100);
        currentState = STATE_REVERSE;
        stateStartTime = now;
        settleStartTime = 0;
        integral = 0; 
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
    if (millis() - startTime > 2000) { client.stop(); return; } // Increased timeout

    if (client.available()) {
      char c = client.read();
      request += c;
      if (c == '\n') {
        if (currentLine.length() == 0) {
          
          if (request.indexOf("GET /cmd?act=start") >= 0) {
            runCommand = true; 
            client.println("HTTP/1.1 200 OK");
            client.println("Connection: close");
            client.println();
            break; 
          }
          else if (request.indexOf("GET /cmd?act=stop") >= 0) {
            stopRun();
            client.println("HTTP/1.1 200 OK");
            client.println("Connection: close");
            client.println();
            break;
          }
          else if (request.indexOf("GET /cmd?act=save") >= 0) {
            saveSettings();
            client.println("HTTP/1.1 200 OK");
            client.println("Connection: close");
            client.println();
            break;
          }
          else if (request.indexOf("GET /set?") >= 0) {
            int idx;
            if((idx = request.indexOf("&td=")) > 0) cfg.targetDistanceCm = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("&bld=")) > 0) cfg.bottleLineDistanceCm = request.substring(idx+5).toFloat();
            if((idx = request.indexOf("&tt=")) > 0) cfg.targetTimeSeconds = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("&ett=")) > 0) cfg.estTravelTimeSeconds = request.substring(idx+5).toFloat();
            if((idx = request.indexOf("&pc=")) > 0) cfg.pushClearanceCm = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("&c=")) > 0) cfg.cmPerCount = request.substring(idx+3).toFloat();
            if((idx = request.indexOf("&db=")) > 0) cfg.deadbandCm = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("&kp=")) > 0) cfg.Kp = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("&ki=")) > 0) cfg.Ki = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("&kd=")) > 0) cfg.Kd = request.substring(idx+4).toFloat();
            if((idx = request.indexOf("&im=")) > 0) cfg.maxPWM = request.substring(idx+4).toInt();
            if((idx = request.indexOf("&mn=")) > 0) cfg.minPWM = request.substring(idx+4).toInt();
            if((idx = request.indexOf("&rmp=")) > 0) cfg.maxAccelRamp = request.substring(idx+5).toFloat();
            if((idx = request.indexOf("&st=")) > 0) cfg.settleTimeMs = request.substring(idx+4).toInt();
            
            client.println("HTTP/1.1 200 OK");
            client.println("Connection: close");
            client.println();
            break;
          }
          else if (request.indexOf("GET /status") >= 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println("Connection: close");
            client.println();
            
            runStateMachine(); 

            float err = setpoint - getDistanceCm();
            float t = (currentState != STATE_IDLE) ? (millis() - runStartTime) / 1000.0 : 0;

            client.print("{\"d\":"); client.print(getDistanceCm(), 2);
            client.print(",\"e\":"); client.print(err, 2);
            
            runStateMachine(); 
            
            client.print(",\"sp\":"); client.print(setpoint, 2);
            client.print(",\"p\":"); client.print(currentPwmOutput);
            client.print(",\"t\":"); client.print(t, 1);
            client.print(",\"s\":"); client.print(currentState);
            client.print("}");
            break; 
          }
          else if (request.indexOf("GET /settings") >= 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println("Connection: close");
            client.println();
            
            runStateMachine(); 

            client.print("{\"td\":"); client.print(cfg.targetDistanceCm);
            client.print(",\"bld\":"); client.print(cfg.bottleLineDistanceCm);
            client.print(",\"tt\":"); client.print(cfg.targetTimeSeconds);
            client.print(",\"ett\":"); client.print(cfg.estTravelTimeSeconds);
            client.print(",\"pc\":"); client.print(cfg.pushClearanceCm);
            client.print(",\"c\":"); client.print(cfg.cmPerCount, 6);
            client.print(",\"db\":"); client.print(cfg.deadbandCm);
            
            runStateMachine();

            client.print(",\"kp\":"); client.print(cfg.Kp);
            client.print(",\"ki\":"); client.print(cfg.Ki);
            client.print(",\"kd\":"); client.print(cfg.Kd);
            client.print(",\"im\":"); client.print(cfg.maxPWM);
            client.print(",\"mn\":"); client.print(cfg.minPWM);
            client.print(",\"rmp\":"); client.print(cfg.maxAccelRamp);
            client.print(",\"st\":"); client.print(cfg.settleTimeMs);
            client.print("}");
            break;
          }
          else {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");
            client.println("Cache-Control: no-store, no-cache, must-revalidate");
            client.println();
            
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
  return counts * cfg.cmPerCount;
}

void setMotor(float targetPwm) {
  if (currentState == STATE_IDLE || currentState == STATE_STOPPED || currentState == STATE_DELAY) {
    hardStopMotor(); return;
  }

  // Traction Control (PWM Ramp Limiter)
  if (targetPwm > currentPwmOutput + cfg.maxAccelRamp) {
    currentPwmOutput += cfg.maxAccelRamp;
  } else if (targetPwm < currentPwmOutput - cfg.maxAccelRamp) {
    currentPwmOutput -= cfg.maxAccelRamp;
  } else {
    currentPwmOutput = targetPwm;
  }

  float pwm = currentPwmOutput;

  // Overcome stall friction
  if (pwm > 0 && pwm < cfg.minPWM) pwm = cfg.minPWM;
  if (pwm < 0 && pwm > -cfg.minPWM) pwm = -cfg.minPWM;
  pwm = constrain(pwm, -255, 255);

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
