#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <driver/pcnt.h>
#include <ArduinoJson.h>

const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

#define ENC_A_PIN       32
#define ENC_B_PIN       33
#define PWM_PIN         25
#define DIR_PIN         26
#define LIMIT_SW1_PIN   34
#define LIMIT_SW2_PIN   35
#define NTC_PIN          4
#define VDIV_PIN        36

#define PWM_CHANNEL     0
#define PWM_FREQ_HZ     20000   // 20 kHz — above audible range
#define PWM_RESOLUTION  8       // 8 bit is 0–255 duty

#define PCNT_UNIT       PCNT_UNIT_0
#define PCNT_H_LIM      32000
#define PCNT_L_LIM     -32000

Adafruit_INA219 ina219;   // default I2C address 0x40

float Kp = 1.2f;
float Ki = 0.08f;
float Kd = 0.05f;

#define NTC_B_COEFF     3950.0f
#define NTC_R_NOMINAL   10000.0f   // 10kΩ at 25°C
#define NTC_R_SERIES    10000.0f   // R14 bias resistor
#define NTC_T_NOMINAL   298.15f    // 25°C in Kelvin

#define VDIV_RATIO      (10000.0f / 110000.0f)
#define ADC_REF_V       3.3f
#define ADC_MAX_COUNT   4095.0f

volatile int32_t  encoderCount      = 0;   // absolute position (pulse accumulator)
volatile int16_t  pcntRaw           = 0;   // last PCNT read
volatile int32_t  pcntOverflow      = 0;   // overflow counter
volatile bool     limitSw1Active    = false;
volatile bool     limitSw2Active    = false;

volatile float    motorCurrentA     = 0.0f;
volatile float    busVoltageV       = 0.0f;
volatile float    railVoltageV      = 0.0f;
volatile float    temperatureC      = 0.0f;

volatile float    targetVelocityRPM = 0.0f;
volatile int32_t  targetPositionPulses = 0;
volatile bool     motorEnabled      = false;
volatile bool     controlModeVelocity = true;  // true=velocity, false=position
volatile int      manualDutyCycle   = 0;       // -255 to +255, used when PID off

float             pidIntegral       = 0.0f;
float             pidPrevError      = 0.0f;
unsigned long     pidLastTimeMs     = 0;

// Velocity estimation
int32_t           prevEncoderCount  = 0;
unsigned long     prevVelTimeMs     = 0;
volatile float    actualVelocityRPM = 0.0f;
// GoBilda 5303 encoder: 28 CPR * gear ratio. Adjust ENCODER_CPR to match your motor.
#define ENCODER_CPR     5264.0f   // counts per revolution (28 CPR * 188:1 gear)

// Manual intervention detection
int32_t           expectedPosition  = 0;
#define MANUAL_INTERVENTION_THRESHOLD  50   // pulses

// ─── Web Server ───────────────────────────────────────────────────────────────
WebServer server(80);

// ─── PCNT ISR — fires on counter overflow / underflow ────────────────────────
static void IRAM_ATTR pcntOverflowISR(void* arg) {
  uint32_t status = 0;
  pcnt_get_event_status(PCNT_UNIT, &status);
  if (status & PCNT_EVT_H_LIM) pcntOverflow++;
  if (status & PCNT_EVT_L_LIM) pcntOverflow--;
}

// ─── Limit switch ISRs ───────────────────────────────────────────────────────
static void IRAM_ATTR limitSw1ISR() {
  limitSw1Active = (digitalRead(LIMIT_SW1_PIN) == LOW);
}
static void IRAM_ATTR limitSw2ISR() {
  limitSw2Active = (digitalRead(LIMIT_SW2_PIN) == LOW);
}

// ─── Helper: get absolute encoder position ───────────────────────────────────
int32_t getEncoderCount() {
  int16_t raw;
  pcnt_get_counter_value(PCNT_UNIT, &raw);
  return (int32_t)pcntOverflow * PCNT_H_LIM + (int32_t)raw;
}

// ─── Helper: apply motor output (-255 to +255) ───────────────────────────────
void setMotorOutput(int output) {
  output = constrain(output, -255, 255);
  if (output == 0) {
    ledcWrite(PWM_CHANNEL, 0);
    return;
  }
  if (output > 0) {
    digitalWrite(DIR_PIN, HIGH);
    ledcWrite(PWM_CHANNEL, (uint32_t)output);
  } else {
    digitalWrite(DIR_PIN, LOW);
    ledcWrite(PWM_CHANNEL, (uint32_t)(-output));
  }
}

// ─── Helper: read NTC temperature (°C) ──────────────────────────────────────
float readTemperature() {
  int adcRaw = analogRead(NTC_PIN);
  float voltage = (adcRaw / ADC_MAX_COUNT) * ADC_REF_V;
  // NTC is on the bottom (GND side), R14 on top (3V3 side)
  float rNTC = NTC_R_SERIES * voltage / (ADC_REF_V - voltage);
  // Steinhart-Hart simplified (B parameter equation)
  float steinhart = rNTC / NTC_R_NOMINAL;           // R/R0
  steinhart = log(steinhart);                        // ln(R/R0)
  steinhart /= NTC_B_COEFF;                          // 1/B * ln(R/R0)
  steinhart += 1.0f / NTC_T_NOMINAL;                 // + 1/T0
  steinhart = 1.0f / steinhart;                      // T in Kelvin
  return steinhart - 273.15f;                        // convert to °C
}

// ─── Helper: read 24V rail via voltage divider ───────────────────────────────
float readRailVoltage() {
  int adcRaw = analogRead(VDIV_PIN);
  float vADC = (adcRaw / ADC_MAX_COUNT) * ADC_REF_V;
  return vADC / VDIV_RATIO;
}

// ─── PID velocity control loop ───────────────────────────────────────────────
void runPIDVelocity() {
  unsigned long now = millis();
  float dtMs = (float)(now - pidLastTimeMs);
  if (dtMs < 10.0f) return;   // run at ~100 Hz
  pidLastTimeMs = now;
  float dt = dtMs / 1000.0f;  // seconds

  float error = targetVelocityRPM - actualVelocityRPM;
  pidIntegral  += error * dt;
  pidIntegral   = constrain(pidIntegral, -200.0f, 200.0f);  // anti-windup
  float derivative = (error - pidPrevError) / dt;
  pidPrevError = error;

  float output = Kp * error + Ki * pidIntegral + Kd * derivative;
  output = constrain(output, -255.0f, 255.0f);
  setMotorOutput((int)output);
}

// ─── PID position control loop ───────────────────────────────────────────────
void runPIDPosition() {
  unsigned long now = millis();
  float dtMs = (float)(now - pidLastTimeMs);
  if (dtMs < 10.0f) return;
  pidLastTimeMs = now;
  float dt = dtMs / 1000.0f;

  int32_t currentPos = getEncoderCount();
  float error = (float)(targetPositionPulses - currentPos);
  pidIntegral  += error * dt;
  pidIntegral   = constrain(pidIntegral, -5000.0f, 5000.0f);
  float derivative = (error - pidPrevError) / dt;
  pidPrevError = error;

  float output = Kp * error + Ki * pidIntegral + Kd * derivative;
  output = constrain(output, -255.0f, 255.0f);
  setMotorOutput((int)output);
  expectedPosition = currentPos;
}

// ─── Velocity estimator (called every 50 ms) ─────────────────────────────────
void updateVelocity() {
  unsigned long now = millis();
  float dtMs = (float)(now - prevVelTimeMs);
  if (dtMs < 50.0f) return;
  prevVelTimeMs = now;

  int32_t current = getEncoderCount();
  float deltaPulses = (float)(current - prevEncoderCount);
  prevEncoderCount = current;
  float dt = dtMs / 1000.0f;
  // RPM = (pulses/dt) / CPR * 60
  actualVelocityRPM = (deltaPulses / dt) / ENCODER_CPR * 60.0f;
}

// ─── Manual intervention detection ───────────────────────────────────────────
bool detectManualIntervention() {
  if (!motorEnabled) {
    int32_t currentPos = getEncoderCount();
    if (abs(currentPos - expectedPosition) > MANUAL_INTERVENTION_THRESHOLD) {
      expectedPosition = currentPos;
      return true;
    }
  }
  return false;
}

// ─── Homing routine: drives to limit switch 1, zeroes encoder ────────────────
void homeMotor() {
  motorEnabled = false;
  // Drive slowly toward limit switch 1
  setMotorOutput(-60);
  unsigned long start = millis();
  while (!limitSw1Active && (millis() - start < 10000)) {
    delay(10);
  }
  setMotorOutput(0);
  // Zero the encoder by clearing PCNT
  pcnt_counter_clear(PCNT_UNIT);
  pcntOverflow     = 0;
  encoderCount     = 0;
  prevEncoderCount = 0;
  expectedPosition = 0;
}

// ─── Web server: serve the UI HTML (stored in PROGMEM) ───────────────────────
// The UI HTML is served from a separate file loaded via SPIFFS or inline.
// Here it is returned inline as a redirect to the UI running on a PC,
// or you can embed the full HTML below. For simplicity, the ESP32 serves
// a minimal data-only API and the full HTML/CSS/JS UI is loaded from the
// same ESP32 IP as "/ui" (see handleUI()).

// Full HTML is defined as a raw string literal at the bottom.
extern const char UI_HTML[] PROGMEM;

void handleUI() {
  server.send_P(200, "text/html", UI_HTML);
}

// GET /api/status — returns all sensor + motor data as JSON
void handleStatus() {
  StaticJsonDocument<512> doc;
  doc["current_A"]        = motorCurrentA;
  doc["bus_voltage_V"]    = busVoltageV;
  doc["rail_voltage_V"]   = railVoltageV;
  doc["temperature_C"]    = temperatureC;
  doc["position_pulses"]  = getEncoderCount();
  doc["velocity_rpm"]     = actualVelocityRPM;
  doc["limit_sw1"]        = limitSw1Active;
  doc["limit_sw2"]        = limitSw2Active;
  doc["motor_enabled"]    = motorEnabled;
  doc["target_rpm"]       = targetVelocityRPM;
  doc["target_pos"]       = targetPositionPulses;
  doc["mode_velocity"]    = controlModeVelocity;
  doc["manual_detected"]  = detectManualIntervention();

  String json;
  serializeJson(doc, json);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// POST /api/control — accepts JSON body with control commands
void handleControl() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}");
    return;
  }
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }

  if (doc.containsKey("enabled"))         motorEnabled         = doc["enabled"].as<bool>();
  if (doc.containsKey("target_rpm"))      targetVelocityRPM    = doc["target_rpm"].as<float>();
  if (doc.containsKey("target_pos"))      targetPositionPulses = doc["target_pos"].as<int32_t>();
  if (doc.containsKey("mode_velocity"))   controlModeVelocity  = doc["mode_velocity"].as<bool>();
  if (doc.containsKey("manual_duty"))     manualDutyCycle      = doc["manual_duty"].as<int>();
  if (doc.containsKey("kp"))             Kp                   = doc["kp"].as<float>();
  if (doc.containsKey("ki"))             Ki                   = doc["ki"].as<float>();
  if (doc.containsKey("kd"))             Kd                   = doc["kd"].as<float>();
  if (doc.containsKey("home") && doc["home"].as<bool>()) homeMotor();

  if (!motorEnabled) setMotorOutput(0);

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleOptions() {
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

// ─── Core 0 task: sensors + PID ──────────────────────────────────────────────
void sensorAndControlTask(void* param) {
  unsigned long lastSensorMs = 0;

  for (;;) {
    unsigned long now = millis();

    // ── Sensor reads every 100 ms ──
    if (now - lastSensorMs >= 100) {
      lastSensorMs = now;
      motorCurrentA  = ina219.getCurrent_mA() / 1000.0f;
      busVoltageV    = ina219.getBusVoltage_V();
      railVoltageV   = readRailVoltage();
      temperatureC   = readTemperature();
    }

    // ── Velocity estimate ──
    updateVelocity();

    // ── Motor control ──
    if (motorEnabled) {
      // Safety: stop if either limit switch is active
      if (limitSw1Active && targetVelocityRPM < 0) { setMotorOutput(0); }
      else if (limitSw2Active && targetVelocityRPM > 0) { setMotorOutput(0); }
      else if (controlModeVelocity) { runPIDVelocity(); }
      else { runPIDPosition(); }
    } else {
      // Manual jog (duty cycle set directly from UI)
      if (manualDutyCycle != 0) {
        if (limitSw1Active && manualDutyCycle < 0) setMotorOutput(0);
        else if (limitSw2Active && manualDutyCycle > 0) setMotorOutput(0);
        else setMotorOutput(manualDutyCycle);
      } else {
        setMotorOutput(0);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));  // 5 ms loop
  }
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("[BOOT] Motor Controller starting...");

  // ── GPIO ──
  pinMode(DIR_PIN,       OUTPUT);
  pinMode(LIMIT_SW1_PIN, INPUT);   // external 10k pull-up on PCB
  pinMode(LIMIT_SW2_PIN, INPUT);

  // ── LEDC PWM ──
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  // ── PCNT (hardware quadrature decoder) ──
  pcnt_config_t pcntCfg = {};
  pcntCfg.pulse_gpio_num  = ENC_A_PIN;
  pcntCfg.ctrl_gpio_num   = ENC_B_PIN;
  pcntCfg.unit            = PCNT_UNIT;
  pcntCfg.channel         = PCNT_CHANNEL_0;
  pcntCfg.pos_mode        = PCNT_COUNT_INC;
  pcntCfg.neg_mode        = PCNT_COUNT_DEC;
  pcntCfg.lctrl_mode      = PCNT_MODE_REVERSE;
  pcntCfg.hctrl_mode      = PCNT_MODE_KEEP;
  pcntCfg.counter_h_lim   = PCNT_H_LIM;
  pcntCfg.counter_l_lim   = PCNT_L_LIM;
  pcnt_unit_config(&pcntCfg);

  // Second channel for full quadrature (B triggers on A level)
  pcntCfg.pulse_gpio_num  = ENC_B_PIN;
  pcntCfg.ctrl_gpio_num   = ENC_A_PIN;
  pcntCfg.channel         = PCNT_CHANNEL_1;
  pcntCfg.pos_mode        = PCNT_COUNT_DEC;
  pcntCfg.neg_mode        = PCNT_COUNT_INC;
  pcnt_unit_config(&pcntCfg);

  pcnt_filter_enable(PCNT_UNIT);
  pcnt_filter_value_set(PCNT_UNIT, 100);   // glitch filter ~1µs @ 80 MHz APB
  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT, pcntOverflowISR, NULL);
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

  // ── Limit switch interrupts ──
  attachInterrupt(LIMIT_SW1_PIN, limitSw1ISR, CHANGE);
  attachInterrupt(LIMIT_SW2_PIN, limitSw2ISR, CHANGE);

  // ── I2C + INA219 ──
  Wire.begin(21, 22);
  if (!ina219.begin()) {
    Serial.println("[ERROR] INA219 not found! Check wiring.");
  } else {
    ina219.setCalibration_32V_2A();
    Serial.println("[OK] INA219 initialised");
  }

  // ── ADC attenuation for GPIO4 and GPIO36 ──
  analogSetAttenuation(ADC_11db);   // 0–3.3V range

  // ── Wi-Fi ──
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WIFI] Connecting");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WIFI] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WIFI] Connection failed — running in AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("MotorController", "12345678");
    Serial.printf("[AP] SSID: MotorController  IP: %s\n",
                  WiFi.softAPIP().toString().c_str());
  }

  // ── HTTP routes ──
  server.on("/",          HTTP_GET,    handleUI);
  server.on("/api/status",HTTP_GET,    handleStatus);
  server.on("/api/control",HTTP_POST,  handleControl);
  server.on("/api/control",HTTP_OPTIONS, handleOptions);
  server.begin();
  Serial.println("[HTTP] Server started");

  // ── Start sensor + control task on Core 0 ──
  xTaskCreatePinnedToCore(
    sensorAndControlTask,
    "SensorCtrl",
    8192,
    NULL,
    2,      // priority
    NULL,
    0       // Core 0
  );

  pidLastTimeMs  = millis();
  prevVelTimeMs  = millis();
}

// ─── Loop (Core 1) — handles HTTP requests ───────────────────────────────────
void loop() {
  server.handleClient();
  delay(1);
}

// ─── Embedded UI HTML ─────────────────────────────────────────────────────────
// The ESP32 serves this HTML at "/" so you can open it from any browser
// on the same network. The JS polls /api/status every 500 ms and sends
// commands via POST /api/control.

const char UI_HTML[] PROGMEM = R"rawhtml(

<!DOCTYPE html>
<html>
<head>
    <title>Motor Control UI</title>
</head>

<body>

<h2>Motor Control Panel</h2>

<h3>Monitoring Data</h3>

<b>Sensors Data</b><br>
Voltage: <span id="voltage">0</span><br>
Current: <span id="current">0</span><br>
Temperature: <span id="temp">0</span><br>
Limit Switch 1: <span id="ls1">0</span><br>
Limit Switch 2: <span id="ls2">0</span><br><br>

<b>Motor Data</b><br>
Position: <span id="pos">0</span><br>
Velocity: <span id="vel">0</span><br>
Status: <span id="status">OFF</span><br><br>

<button onclick="getStatus()">Refresh</button>

<hr>

<h3>Motor Jogging Control</h3>

<button onclick="forward()">Forward</button>
<button onclick="reverse()">Reverse</button>

<br><br>
Target Velocity:
<input type="number" id="targetVel"><br><br>

Target Position:
<input type="number" id="targetPos"><br><br>

<button onclick="sendTargets()">Set Targets</button>

<hr>

<h3>Digital Twin</h3>

<div style="width:300px; height:50px; border:1px solid black; position:relative;">
    <div id="motor" style="width:20px; height:50px; background:black; position:absolute; left:0;"></div>
</div>

<hr>

<button onclick="enableMotor()">Enable</button>
<button onclick="disableMotor()">Disable</button>

<script>
function getStatus() {
    fetch('/api/status')
    .then(res => res.json())
    .then(data => {

        document.getElementById("voltage").innerText = data.rail_voltage_V;
        document.getElementById("current").innerText = data.current_A;
        document.getElementById("temp").innerText = data.temperature_C;
        document.getElementById("ls1").innerText = data.limit_sw1;
        document.getElementById("ls2").innerText = data.limit_sw2;

        document.getElementById("pos").innerText = data.position_pulses;
        document.getElementById("vel").innerText = data.velocity_rpm;
        document.getElementById("status").innerText = data.motor_enabled ? "ON" : "OFF";

        // Digital twin
        let pos = data.position_pulses;
        document.getElementById("motor").style.left = (pos % 300) + "px";
    });
}

function forward() {
    fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ manual_duty: 150 })
    });
}

function reverse() {
    fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ manual_duty: -150 })
    });
}

function sendTargets() {
    let vel = document.getElementById("targetVel").value;
    let pos = document.getElementById("targetPos").value;

    fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            target_rpm: vel,
            target_pos: pos,
            mode_velocity: true
        })
    });
}

function enableMotor() {
    fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ enabled: true })
    });
}

function disableMotor() {
    fetch('/api/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ enabled: false })
    });
}
</script>

</body>
</html>
)rawhtml";
