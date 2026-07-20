#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>

// Onboard RGB-LED (WS2812). Standard bei ESP32-S3-DevKitC-1 ist GPIO48,
// manche Clones nutzen GPIO38 - falls die LED nicht reagiert, hier umstellen.
#define RGB_LED_PIN 48
Adafruit_NeoPixel rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// ================= WIFI AP CONFIG =================
// ESP32-S3 spannt sein eigenes WLAN auf. Verbinde dich mit diesem Netzwerk
// und öffne dann http://192.168.4.1 im Browser.
const char* AP_SSID = "ArmControl";
const char* AP_PASS = "steuerung123";   // min. 8 Zeichen

WebServer server(80);

// ================= MOTOR CONFIG =================
// !!! GPIO19/20 sind auf dem ESP32-S3 die nativen USB D-/D+ Pins -> NICHT benutzen !!!
// !!! GPIO0, 45, 46 sind Strapping-Pins -> möglichst vermeiden !!!
// !!! GPIO26-32 ggf. für Flash/PSRAM belegt (je nach Modul) -> vermeiden !!!
// Diese Pins hier sind auf den meisten S3-DevBoards frei, bitte gegen euer
// konkretes Board-Pinout gegenchecken.

#define STEP_PIN_1 4
#define DIR_PIN_1  5
#define STEP_PIN_2 6
#define DIR_PIN_2  7
#define STEP_PIN_3 15
#define DIR_PIN_3  16
#define EN_PIN     17

constexpr bool MOTOR1DIRECTION = true;
constexpr bool MOTOR2DIRECTION = true;
constexpr bool MOTOR3DIRECTION = true;

constexpr float RPS = 0.5;              // Umdrehungen/Sekunde (normal)
constexpr float FINE_RPS = 0.05;        // Umdrehungen/Sekunde (Fine-Step)
#define STEPS_PER_REV 1600

volatile bool fineStepMode = false;

// Berechnet die aktuelle Puls-Halbperiode in Mikrosekunden, abhaengig vom Fine-Step-Modus
uint32_t currentStepPeriodUs() {
  float rps = fineStepMode ? FINE_RPS : RPS;
  return (uint32_t)(1000000.0 / (rps * STEPS_PER_REV * 2));
}

// ================= MOTOR STATE (Arrays statt struct) =================
// Index 0 wird nicht benutzt, Motoren sind 1,2,3 - vermeidet Off-by-one-Verwirrung.
#define NUM_MOTORS 4

uint8_t stepPins[NUM_MOTORS]   = {0, STEP_PIN_1, STEP_PIN_2, STEP_PIN_3};
uint8_t dirPins[NUM_MOTORS]    = {0, DIR_PIN_1,  DIR_PIN_2,  DIR_PIN_3};
bool    motorBaseDir[NUM_MOTORS] = {true, MOTOR1DIRECTION, MOTOR2DIRECTION, MOTOR3DIRECTION};

volatile bool     motorEnabled[NUM_MOTORS]    = {false, false, false, false};
volatile bool     motorDir[NUM_MOTORS]        = {true, true, true, true};
bool              motorStepState[NUM_MOTORS]  = {false, false, false, false};
uint32_t          motorLastStepUs[NUM_MOTORS] = {0, 0, 0, 0};
volatile uint32_t motorStepCount[NUM_MOTORS]  = {0, 0, 0, 0};

// ================= NON-BLOCKING STEP GENERATION =================
// Wird bei jedem loop()-Durchlauf aufgerufen. Kein Hardware-Timer nötig,
// robust unabhängig von der installierten ESP32-Core-Version.
void serviceMotor(int i) {
  if (!motorEnabled[i]) return;
  uint32_t now = micros();
  if (now - motorLastStepUs[i] >= currentStepPeriodUs()) {
    motorLastStepUs[i] = now;
    motorStepState[i] = !motorStepState[i];
    digitalWrite(stepPins[i], motorStepState[i]);
    if (motorStepState[i]) motorStepCount[i]++;  // einen Zaehler pro voller Puls-Flanke
  }
}

void setMotorDir(int i, bool logicalUp) {
  motorDir[i] = logicalUp ? motorBaseDir[i] : !motorBaseDir[i];
  digitalWrite(dirPins[i], motorDir[i]);
}

// ================= RGB LED STATUS =================
// Gelenk 1 = Rot, Gelenk 2 = Gruen, Gelenk 3 = Blau. Nichts aktiv = LED aus.
// Bei mehreren gleichzeitig gehaltenen Buttons gewinnt die niedrigste Nummer.
void updateStatusLED() {
  if (motorEnabled[1]) {
    rgbLed.setPixelColor(0, rgbLed.Color(255, 0, 0));
  } else if (motorEnabled[2]) {
    rgbLed.setPixelColor(0, rgbLed.Color(0, 255, 0));
  } else if (motorEnabled[3]) {
    rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 255));
  } else {
    rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));
  }
  rgbLed.show();
}

// ================= WEB HANDLERS =================
// Aufruf-Schema: /ctrl?m=1&d=up&s=start   bzw. s=stop
void handleCtrl() {
  if (!server.hasArg("m") || !server.hasArg("d") || !server.hasArg("s")) {
    server.send(400, "text/plain", "missing args");
    return;
  }
  int m = server.arg("m").toInt();
  String d = server.arg("d");
  String s = server.arg("s");
  bool up = (d == "up");
  bool start = (s == "start");

  if (m < 1 || m > 3) {
    server.send(400, "text/plain", "invalid motor");
    return;
  }

  if (start) {
    setMotorDir(m, up);
    motorEnabled[m] = true;
    Serial.printf("[BUTTON] Motor %d: START Richtung=%s (dirPin=%s)\n",
                   m, up ? "hoch" : "runter", motorDir[m] ? "HIGH" : "LOW");
  } else {
    motorEnabled[m] = false;
    Serial.printf("[BUTTON] Motor %d: STOP (gesamt %lu Pulse seit Boot)\n",
                   m, (unsigned long)motorStepCount[m]);
  }
  updateStatusLED();
  server.send(200, "text/plain", "ok");
}

// Toggelt den Fine-Step-Modus (langsame Feinjustierung) an/aus
void handleFineToggle() {
  fineStepMode = !fineStepMode;
  Serial.printf("[FINE] Fine-Step-Modus: %s\n", fineStepMode ? "AN" : "AUS");
  server.send(200, "text/plain", fineStepMode ? "on" : "off");
}

void handleRoot() {
  String html = R"HTML(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:sans-serif;text-align:center;background:#111;color:#eee;
       -webkit-user-select:none;user-select:none;-webkit-touch-callout:none;}
  .row{margin:20px;}
  button{width:120px;height:70px;font-size:18px;margin:8px;border-radius:10px;border:none;
         background:#2a7;color:white;touch-action:none;
         -webkit-user-select:none;user-select:none;-webkit-touch-callout:none;
         -webkit-tap-highlight-color:transparent;}
  button:active{background:#194;}
  h3{color:#9cf;}
</style></head><body>
<h2>Arm Control (3 Achsen)</h2>

<h3>Gelenk 1</h3>
<div class="row">
  <button onmousedown="go(1,'up')" onmouseup="stop(1)" ontouchstart="go(1,'up')" ontouchend="stop(1)">🔼 Hoch</button>
  <button onmousedown="go(1,'down')" onmouseup="stop(1)" ontouchstart="go(1,'down')" ontouchend="stop(1)">🔽 Runter</button>
</div>

<h3>Gelenk 2</h3>
<div class="row">
  <button onmousedown="go(2,'up')" onmouseup="stop(2)" ontouchstart="go(2,'up')" ontouchend="stop(2)">🔼 Hoch</button>
  <button onmousedown="go(2,'down')" onmouseup="stop(2)" ontouchstart="go(2,'down')" ontouchend="stop(2)">🔽 Runter</button>
</div>

<h3>Gelenk 3</h3>
<div class="row">
  <button onmousedown="go(3,'up')" onmouseup="stop(3)" ontouchstart="go(3,'up')" ontouchend="stop(3)">🔼 Hoch</button>
  <button onmousedown="go(3,'down')" onmouseup="stop(3)" ontouchstart="go(3,'down')" ontouchend="stop(3)">🔽 Runter</button>
</div>

<hr style="border-color:#333;margin:24px auto;width:80%;">
<div class="row">
  <button id="fineBtn" onclick="toggleFine()" style="width:200px;background:#555;">🚀 Fine-Step: AUS</button>
</div>

<script>
function go(m,d){ fetch('/ctrl?m='+m+'&d='+d+'&s=start'); }
function stop(m){ fetch('/ctrl?m='+m+'&d=up&s=stop'); }
// Sicherheitsnetz: falls Finger/Maus den Button verlässt ohne "up"-Event
document.addEventListener('mouseleave', function(){
  for (let m=1;m<=3;m++) stop(m);
});

function toggleFine(){
  fetch('/fine').then(r => r.text()).then(state => {
    const btn = document.getElementById('fineBtn');
    if (state === 'on') {
      btn.textContent = '🐢 Fine-Step: AN';
      btn.style.background = '#c73';
    } else {
      btn.textContent = '🚀 Fine-Step: AUS';
      btn.style.background = '#555';
    }
  });
}
</script>
</body></html>
)HTML";
  server.send(200, "text/html; charset=utf-8", html);
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN_1, OUTPUT); pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT); pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_3, OUTPUT); pinMode(DIR_PIN_3, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Treiber aktivieren (TB6600: LOW = enabled)

  rgbLed.begin();
  rgbLed.setBrightness(50); // 0-255, nicht blenden
  updateStatusLED();

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP gestartet. IP-Adresse: ");
  Serial.println(WiFi.softAPIP()); // sollte 192.168.4.1 sein

  server.on("/", handleRoot);
  server.on("/ctrl", handleCtrl);
  server.on("/fine", handleFineToggle);
  server.begin();
  Serial.println("Webserver läuft.");
}

void loop() {
  server.handleClient();
  serviceMotor(1);
  serviceMotor(2);
  serviceMotor(3);

  static uint32_t lastReport = 0;
  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    Serial.printf("[STATUS] M1: %s (%lu Pulse)  M2: %s (%lu Pulse)  M3: %s (%lu Pulse)\n",
                  motorEnabled[1] ? "AN" : "aus", (unsigned long)motorStepCount[1],
                  motorEnabled[2] ? "AN" : "aus", (unsigned long)motorStepCount[2],
                  motorEnabled[3] ? "AN" : "aus", (unsigned long)motorStepCount[3]);
  }
}
