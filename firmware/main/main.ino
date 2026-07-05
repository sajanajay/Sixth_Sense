// Importing Necessary Libraries (Header Files)
#include <Arduino.h>

// Specififc Library designed for acceses to Watchdog Timer in ESP32

#include <esp_task_wdt.h>  

//Purpose of the WDT is reset the system if the firmware becomes unresponsive.


// GPIO Pin Allocation for Radar Sensor 

/* !!!Notice!!!
    Radar TX ----> Defined ESP 32 RX GPIO Pin
    Radar RX ----> Defined ESP 32 TX GPIO Pin

    Above crossing is the fundamental behavior of serial communication

*/

#define RADAR_RX 16  // JO16 
#define RADAR_TX 17  // JO17

// GPIO Pins Allocation for Motor Control (MOSFET Controlled DC Motors)
#define MOTOR_RIGHT_PIN 26 // IO26 (Right Haptic - MOTOR_1)
#define MOTOR_LEFT_PIN 27  // IO27 (Left Haptic - MOTOR_2)

// Pin Allocation for Indication LEDs & Analog Battery Voltage Sensing Input
#define FIRMWARE_LED_PIN 4   // JO4 (Onboard D6)
#define INDICATION_LED_PIN 5 // JO5 (External Header J4)
#define BATTERY_ADC_PIN 34   // JO34

// PWM Settings used to Control the Intensity of the ERM Haptic Motors.
#define PWM_FREQ 1000 // 1000 pulses per seconds, T-period = 1ms     
#define PWM_RES 8
  // level count = 256 value range 0-255
  // value 0 (0% Power) --> always off    value 255 (100% Power) --> always on
  
#define PWM_DUTY_50 127   // 50% power (Used in Initialization)
  
  // Higher the threat higher the intensity of the vibration
#define PWM_DUTY_40 102   // 40% power (FAST Vehicles: >= 10 km/h)
#define PWM_DUTY_20 51    // 20% power (SLOW Vehicles: 5 to 9 km/h)
#define PWM_DUTY_0 0      // 0% power (Off)

// Battery Sensing Configuration
/*
    V = ADC * (Vref / 4096) * ((R1+R2)/R2)
      = ADC * (3.3/4096) * ((20K+10K)/10K) = ADC * 0.002417
*/
const float VOLTAGE_MULTIPLIER = 0.002417;
const float LOW_BATT_THRESHOLD = 6.2;

//#define DEBUG - Used during the build to print the parameters(speed, distacne , etc...)
// in serial montor

// Initialize a dediacted UART2 Serial Interface to Communicate with Radar
HardwareSerial RadarSerial(2);

// Assignin Radar Protocol Constants to an Array
const byte HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const byte FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};

// Parser Variables
enum ParserState { SYNC_HEADER, READ_LENGTH, READ_PAYLOAD, SYNC_FOOTER };
ParserState currentState = SYNC_HEADER;
int headerIndex = 0;
int footerIndex = 0;
uint16_t payloadLength = 0;
int payloadIndex = 0;
byte payload[128]; // FIX: Increased from 64 — LD2451 multi-target frames can exceed 64 bytes
bool payloadValid = false; // FIX: Track whether current payload is fresh and complete

// Directional Speed-Based State Machine
enum ThreatZone { CLEAR = 0, SLOW = 1, FAST = 2 };
ThreatZone previousLeftZone  = CLEAR;
ThreatZone previousRightZone = CLEAR;

// FIX: Independent confidence counters per zone — a left-side burst no longer gates the right motor
int leftConfidenceCounter  = 0;
int rightConfidenceCounter = 0;
const int REQUIRED_HITS    = 2;   // Target must persist for 2 frames before triggering
const int CONFIDENCE_CAP   = 5;   // Maximum counter value
const int MIN_THREAT_SPEED = 5;   // Ignore micro-vibrations under 5 km/h

// Independent Non-Blocking Motor Timer Variables (250ms "Single Tap")
const unsigned long MOTOR_DURATION   = 250;
unsigned long leftMotorStartTime     = 0;
bool leftMotorActive                 = false;
unsigned long rightMotorStartTime    = 0;
bool rightMotorActive                = false;

// Sustained-threat re-pulse: re-alert rider if threat persists at same level
const unsigned long REPULSE_INTERVAL = 1000; // Re-fire every 1s for sustained threats
unsigned long lastLeftPulseTime      = 0;
unsigned long lastRightPulseTime     = 0;

// Non-Blocking Firmware LED Pulse Variables
const unsigned long FW_LED_DURATION = 100; // milliseconds
unsigned long fwLedStartTime        = 0;
bool fwLedActive                    = false;

// Non-Blocking Indication LED Blink Variables
unsigned long lastBlinkTime         = 0;
const unsigned long BLINK_INTERVAL  = 500; // 500ms low-voltage toggle
bool indicationLedState             = false;

// Battery Polling Timer
unsigned long lastBatteryCheck           = 0;
const unsigned long BATT_CHECK_INTERVAL  = 1000; // Sample battery every 1s
float currentVoltage                     = 8.4;

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(921600);
  RadarSerial.begin(115200, SERIAL_8N1, RADAR_RX, RADAR_TX);

  // Watchdog timer — 5 second timeout (IDF 5.x / Arduino Core 3.x API)
  // Resets the chip if loop() ever stalls, critical for a safety wearable
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms    = 5000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);

  // Initialize ESP32 V3 Hardware PWM for Motors
  ledcAttach(MOTOR_RIGHT_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_LEFT_PIN, PWM_FREQ, PWM_RES);

  // Initialize LED Pins
  pinMode(FIRMWARE_LED_PIN, OUTPUT);
  pinMode(INDICATION_LED_PIN, OUTPUT);
  digitalWrite(FIRMWARE_LED_PIN, LOW);
  digitalWrite(INDICATION_LED_PIN, HIGH); // Solid ON when powered

  analogReadResolution(12); // Setup ADC range (0-4095)

  Serial.println("System Booting...");

  // ---------------------------------------------------------
  // HARDWARE HAPTIC SELF-TEST (250ms Buzz on Power Up)
  // ---------------------------------------------------------
  Serial.println("Executing Haptic Self-Test...");
  ledcWrite(MOTOR_LEFT_PIN, PWM_DUTY_50);
  ledcWrite(MOTOR_RIGHT_PIN, PWM_DUTY_50);
  delay(250);
  ledcWrite(MOTOR_LEFT_PIN, PWM_DUTY_0);
  ledcWrite(MOTOR_RIGHT_PIN, PWM_DUTY_0);
  Serial.println("Self-Test Complete.");

  // Flush any radar data that accumulated in the buffer during the 250ms delay
  while (RadarSerial.available()) {
    RadarSerial.read();
  }

  Serial.println("System Initialized: Directional Velocity Logic Active (250ms Tap).");
}

// ---------------------------------------------------------
// BATTERY & INDICATION LED
// ---------------------------------------------------------
void checkBatteryAndIndication() {
  // FIX: Snapshot millis() once to avoid non-deterministic double-call
  unsigned long now = millis();

  if (now - lastBatteryCheck >= BATT_CHECK_INTERVAL) {
    int rawADC = analogRead(BATTERY_ADC_PIN);
    currentVoltage = rawADC * VOLTAGE_MULTIPLIER;
    lastBatteryCheck = now;
  }

  if (currentVoltage < LOW_BATT_THRESHOLD) {
    if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
      indicationLedState = !indicationLedState;
      digitalWrite(INDICATION_LED_PIN, indicationLedState);
      lastBlinkTime = millis();
    }
  } else {
    digitalWrite(INDICATION_LED_PIN, HIGH);
    indicationLedState = true;
  }
}

// ---------------------------------------------------------
// CORE THREAT EVALUATION
// ---------------------------------------------------------
void evaluateTargetThreat() {

 
  if (!payloadValid || payloadLength == 0) {
    
    if (leftConfidenceCounter  > 0) leftConfidenceCounter--;
    if (rightConfidenceCounter > 0) rightConfidenceCounter--;
    return;
  }
  payloadValid = false; 
  byte targetCount = payload[0];
  ThreatZone currentLeftZone  = CLEAR;
  ThreatZone currentRightZone = CLEAR;
  bool validLeftThreat  = false;
  bool validRightThreat = false;

  for (int i = 0; i < targetCount; i++) {
    int baseIdx = 2 + (i * 5);
    if (baseIdx + 4 >= (int)payloadLength) break;

    
    int angle      = (int)payload[baseIdx] - 128;
    byte direction = payload[baseIdx + 2];
    byte speed_kmh = payload[baseIdx + 3];

    if (direction == 0x01 && speed_kmh >= MIN_THREAT_SPEED) {
      ThreatZone targetThreatLevel = (speed_kmh >= 10) ? FAST : SLOW;

      
      bool isLeft   = (angle > 7.5);
      bool isRight  = (angle < -7.5);
      bool isCentre = (angle >= -7.5 && angle <= 7.5);

      if (isLeft || isCentre) {
        if (targetThreatLevel > currentLeftZone) currentLeftZone = targetThreatLevel;
        validLeftThreat = true;
      }
      if (isRight || isCentre) {
        if (targetThreatLevel > currentRightZone) currentRightZone = targetThreatLevel;
        validRightThreat = true;
      }
    }
  }

  // ---------------------------------------------------------
  // FIX: INDEPENDENT "LEAKY BUCKET" COUNTERS PER ZONE
  // A cluster of left-side targets no longer pre-gates the right motor
  // ---------------------------------------------------------
  if (validLeftThreat) {
    leftConfidenceCounter++;
    if (leftConfidenceCounter > CONFIDENCE_CAP) leftConfidenceCounter = CONFIDENCE_CAP;
  } else {
    if (leftConfidenceCounter > 0) leftConfidenceCounter--;
    if (leftConfidenceCounter < REQUIRED_HITS) currentLeftZone = CLEAR;
  }

  if (validRightThreat) {
    rightConfidenceCounter++;
    if (rightConfidenceCounter > CONFIDENCE_CAP) rightConfidenceCounter = CONFIDENCE_CAP;
  } else {
    if (rightConfidenceCounter > 0) rightConfidenceCounter--;
    if (rightConfidenceCounter < REQUIRED_HITS) currentRightZone = CLEAR;
  }

  // ---------------------------------------------------------
  // EXECUTE 250ms "SINGLE TAP" HAPTICS (Only on Escalation)
  // ---------------------------------------------------------
  bool triggeredNewEvent = false;

  // FIX: Only update previousZone when confidence is met — sub-threshold detections
  // no longer silently advance the baseline and eat the next real escalation
  if (leftConfidenceCounter >= REQUIRED_HITS) {
    bool escalated  = (currentLeftZone > previousLeftZone);
    bool repulse    = (currentLeftZone != CLEAR && currentLeftZone == previousLeftZone &&
                       millis() - lastLeftPulseTime >= REPULSE_INTERVAL);
    if (escalated || repulse) {
      int dutyCycle = (currentLeftZone == FAST) ? PWM_DUTY_40 : PWM_DUTY_20;
      ledcWrite(MOTOR_LEFT_PIN, dutyCycle);
      leftMotorStartTime = millis();
      leftMotorActive    = true;
      lastLeftPulseTime  = millis();
      triggeredNewEvent  = true;
#ifdef DEBUG
      Serial.printf(">>> LEFT TAP  | Speed Tier: %d | PWM: %d | %s <<<\n",
                    currentLeftZone, dutyCycle, escalated ? "ESCALATION" : "SUSTAINED");
#endif
    }
    previousLeftZone = currentLeftZone;
  }

  if (rightConfidenceCounter >= REQUIRED_HITS) {
    bool escalated  = (currentRightZone > previousRightZone);
    bool repulse    = (currentRightZone != CLEAR && currentRightZone == previousRightZone &&
                       millis() - lastRightPulseTime >= REPULSE_INTERVAL);
    if (escalated || repulse) {
      int dutyCycle = (currentRightZone == FAST) ? PWM_DUTY_40 : PWM_DUTY_20;
      ledcWrite(MOTOR_RIGHT_PIN, dutyCycle);
      rightMotorStartTime = millis();
      rightMotorActive    = true;
      lastRightPulseTime  = millis();
      triggeredNewEvent   = true;
#ifdef DEBUG
      Serial.printf(">>> RIGHT TAP | Speed Tier: %d | PWM: %d | %s <<<\n",
                    currentRightZone, dutyCycle, escalated ? "ESCALATION" : "SUSTAINED");
#endif
    }
    previousRightZone = currentRightZone;
  }

  if (triggeredNewEvent) {
    digitalWrite(FIRMWARE_LED_PIN, HIGH);
    fwLedStartTime = millis();
    fwLedActive    = true;
  }
}

// ---------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------
void loop() {
  // Feed the watchdog — if we ever fail to reach here within 5s, the WDT resets the chip
  esp_task_wdt_reset();

  // 1. Run Battery Diagnostic
  checkBatteryAndIndication();

  // 2. Manage Independent 250ms Motor Duration Timeouts
  if (leftMotorActive) {
    if (millis() - leftMotorStartTime >= MOTOR_DURATION) {
      ledcWrite(MOTOR_LEFT_PIN, PWM_DUTY_0);
      leftMotorActive = false;
    }
  }

  if (rightMotorActive) {
    if (millis() - rightMotorStartTime >= MOTOR_DURATION) {
      ledcWrite(MOTOR_RIGHT_PIN, PWM_DUTY_0);
      rightMotorActive = false;
    }
  }

  // 3. Manage Firmware Status LED Pulse
  if (fwLedActive) {
    if (millis() - fwLedStartTime >= FW_LED_DURATION) {
      digitalWrite(FIRMWARE_LED_PIN, LOW);
      fwLedActive = false;
    }
  }

  // 4. PREVENT FIFO BUFFER LOGJAM
  // FIX: Don't return after flush — reset state and let the parser loop continue.
  //      Also reset footerIndex which was previously missed.
  if (RadarSerial.available() > 512) {
    while (RadarSerial.available()) {
      RadarSerial.read();
    }
    currentState = SYNC_HEADER;
    headerIndex  = 0;
    footerIndex  = 0; // FIX: was missing, caused footer sync ghost matches after flush
    // No return — fall through to the parser which will now find nothing to read
  }

  // 5. Standard UART Stream Parsing State Machine
  while (RadarSerial.available()) {
    byte b = RadarSerial.read();

    switch (currentState) {
      case SYNC_HEADER:
        if (b == HEADER[headerIndex]) {
          headerIndex++;
          if (headerIndex == 4) {
            currentState = READ_LENGTH;
            headerIndex  = 0;
          }
        } else {
          headerIndex = (b == HEADER[0]) ? 1 : 0;
        }
        break;

      case READ_LENGTH:
        if (headerIndex == 0) {
          payloadLength = b; // Low byte
          headerIndex++;
        } else {
          payloadLength |= ((uint16_t)b << 8); // High byte
          headerIndex    = 0;
          if (payloadLength >= sizeof(payload)) {
            // FIX: Was '>' — this missed the exact boundary case.
            // >= correctly rejects frames that would overflow the buffer.
            currentState = SYNC_HEADER;
          } else if (payloadLength == 0) {
            // FIX: Mark payload as invalid before jumping to footer
            // so evaluateTargetThreat() doesn't process last frame's stale data
            payloadValid = false;
            currentState = SYNC_FOOTER;
          } else {
            payloadIndex = 0;
            currentState = READ_PAYLOAD;
          }
        }
        break;

      case READ_PAYLOAD:
        payload[payloadIndex++] = b;
        if (payloadIndex >= payloadLength) {
          payloadValid = true; // FIX: Mark this frame as fresh and complete
          currentState = SYNC_FOOTER;
        }
        break;

      case SYNC_FOOTER:
        if (b == FOOTER[footerIndex]) {
          footerIndex++;
          if (footerIndex == 4) {
            evaluateTargetThreat();
            currentState = SYNC_HEADER;
            footerIndex  = 0;
          }
        } else {
          currentState = SYNC_HEADER;
          footerIndex  = 0;
        }
        break;
    }
  }
}