/*
 * Smart Anti-Drowsiness Steering Wheel System - Memory Optimized
 * Author: Wasana Karunanayaka - S/21/081
 * Date: 13th July 2025
 * Version: 1.5 - Final
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <SparkFun_ADXL345.h>

// Pin definitions
#define LCD_ADDRESS 0x27
#define TOUCH_SENSOR_1 9
#define TOUCH_SENSOR_2 10
#define TOUCH_SENSOR_3 11
#define TOUCH_SENSOR_4 12
#define RGB_LED_PIN 8
#define BUZZER_PIN 7
#define VIBRATION_MOTOR_1 3
#define VIBRATION_MOTOR_2 2
#define MOTOR_SPEED_PIN 6
#define MOTOR_DIRECTION_1 4
#define MOTOR_DIRECTION_2 5
#define TACTILE_BUTTON_PIN A3

// LED configuration
#define NUM_LEDS 26
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

// Musical notes
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_G3 196

const uint8_t PREDEFINED_HEART_RATE PROGMEM = 75;

// Component status using bit fields for memory efficiency
struct ComponentStatus {
  bool lcd : 1;
  bool heartSensor : 1;
  bool accelerometer : 1;
  bool ledStrip : 1;
  bool buzzer : 1;
  bool vibrator : 1;
  bool piOnline : 1;
  bool failSafeMode : 1;
};

// Global variables
ComponentStatus components = { 0 };
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
MAX30105 heartSensor;
ADXL345 accelerometer;
CRGB leds[NUM_LEDS];

// Sensor data
uint8_t heartRate = 0;
uint8_t spO2 = 0;
int16_t accelX = 0, accelY = 0, accelZ = 0;
uint8_t touchSensors = 0;
bool tactilePressed = false;

// Heart rate calculation
const byte RATE_SIZE = 3;
uint16_t rateArray[RATE_SIZE];
byte rateIndex = 0;
uint32_t lastBeat = 0;

// Jerk detection
int16_t accelHistory[3];
byte accelHistoryIndex = 0;
uint32_t lastJerkTime = 0;
uint8_t jerkCount = 0;
uint32_t jerkWindowStart = 0;

// Constants in PROGMEM
const int JERK_THRESHOLD PROGMEM = 10;
const int JERK_WINDOW PROGMEM = 10000;
const int JERK_COUNT_THRESHOLD PROGMEM = 3;
const int MIN_JERK_INTERVAL PROGMEM = 1000;

// Alert system
enum AlertLevel : uint8_t {
  NO_ALERT = 0,
  LEVEL_1 = 1,
  LEVEL_2 = 2,
  LEVEL_3 = 3
};
AlertLevel currentAlertLevel = NO_ALERT;
bool alertActive = false;
uint32_t alertStartTime = 0;

// Fail-safe system
uint8_t touchSensorFailCount = 0;
uint32_t lastTouchChange = 0;

// Motor control
uint8_t motorSpeed = 180;
const uint8_t normalMotorSpeed = 180;
bool motorStopped = false;

// Timing variables
uint32_t lastSensorRead = 0;
uint32_t lastDataTransmission = 0;
uint32_t lastLCDUpdate = 0;
uint32_t lastLEDUpdate = 0;
uint32_t lastAlertCheck = 0;
uint32_t lastJerkCheck = 0;
uint32_t lastPiCommunication = 0;

// Timing constants
const uint16_t SENSOR_READ_INTERVAL PROGMEM = 100;
const uint16_t DATA_TRANSMISSION_INTERVAL PROGMEM = 200;
const uint16_t LCD_UPDATE_INTERVAL PROGMEM = 1000;
const uint16_t LED_UPDATE_INTERVAL PROGMEM = 300;
const uint16_t ALERT_CHECK_INTERVAL PROGMEM = 100;
const uint16_t JERK_CHECK_INTERVAL PROGMEM = 50;
const uint16_t PI_TIMEOUT PROGMEM = 5000;
const uint16_t LEVEL_1_DURATION PROGMEM = 5000;
const uint16_t LEVEL_2_DURATION PROGMEM = 5000;

// Fail-safe constants
const uint8_t TOUCH_FAIL_THRESHOLD PROGMEM = 3;
const uint16_t TOUCH_STABILITY_TIME PROGMEM = 2000;

void setup() {
  delay(2000);
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println("=================================");
  Serial.println("Smart Anti-Drowsiness System v1.4");
  Serial.println("Initializing...");
  Serial.println("=================================");

  Wire.begin();

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("== SafeSteer =="));
  components.lcd = true;

  // Initialize heart rate sensor (using predefined value)
  components.heartSensor = false;
  heartRate = pgm_read_byte(&PREDEFINED_HEART_RATE);

  // Initialize accelerometer
  Serial.println(F("Starting accelerometer..."));
  accelerometer.powerOn();
  accelerometer.setRangeSetting(16);
  accelerometer.setSpiBit(0);

  // Simple test - just try to read once
  int16_t testX, testY, testZ;
  accelerometer.readAccel(&testX, &testY, &testZ);

  Serial.print(F("Accel test: X="));
  Serial.print(testX);
  Serial.print(F(" Y="));
  Serial.print(testY);
  Serial.print(F(" Z="));
  Serial.println(testZ);

  // Mark as working if we get ANY reading (even zeros are valid)
  components.accelerometer = true;

  // Initialize LED strip
  FastLED.addLeds<LED_TYPE, RGB_LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  components.ledStrip = true;

  // Initialize pins
  pinMode(TOUCH_SENSOR_1, INPUT);
  pinMode(TOUCH_SENSOR_2, INPUT);
  pinMode(TOUCH_SENSOR_3, INPUT);
  pinMode(TOUCH_SENSOR_4, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VIBRATION_MOTOR_1, OUTPUT);
  pinMode(VIBRATION_MOTOR_2, OUTPUT);
  pinMode(MOTOR_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_1, OUTPUT);
  pinMode(MOTOR_DIRECTION_2, OUTPUT);
  pinMode(TACTILE_BUTTON_PIN, INPUT_PULLUP);

  // Ensure outputs are OFF
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(VIBRATION_MOTOR_1, LOW);
  digitalWrite(VIBRATION_MOTOR_2, LOW);
  analogWrite(VIBRATION_MOTOR_1, 0);
  analogWrite(VIBRATION_MOTOR_2, 0);

  components.buzzer = true;
  components.vibrator = true;

  // Initialize motor
  digitalWrite(MOTOR_DIRECTION_1, HIGH);
  digitalWrite(MOTOR_DIRECTION_2, LOW);
  analogWrite(MOTOR_SPEED_PIN, motorSpeed);

  // Initialize timing
  uint32_t currentTime = millis();
  lastSensorRead = currentTime;
  lastDataTransmission = currentTime;
  lastLCDUpdate = currentTime;
  lastLEDUpdate = currentTime;
  lastAlertCheck = currentTime;
  lastJerkCheck = currentTime;
  lastPiCommunication = currentTime;
  lastTouchChange = currentTime;

  memset(accelHistory, 0, sizeof(accelHistory));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SafeSteer"));

  components.piOnline = true;
  Serial.println(F("System Ready"));
}

void loop() {
  uint32_t currentTime = millis();

  checkSerialCommunication();
  checkPiStatus(currentTime);

  if (currentTime - lastSensorRead >= pgm_read_word(&SENSOR_READ_INTERVAL)) {
    readAllSensors();
    lastSensorRead = currentTime;
  }

  if (currentTime - lastJerkCheck >= pgm_read_word(&JERK_CHECK_INTERVAL)) {
    checkSuddenJerks(currentTime);
    lastJerkCheck = currentTime;
  }

  checkTactileButton();

  if (currentTime - lastAlertCheck >= pgm_read_word(&ALERT_CHECK_INTERVAL)) {
    processAlertSystem(currentTime);
    lastAlertCheck = currentTime;
  }

  // Step 7: Add fail-safe condition checking in the main loop
  if (components.failSafeMode && currentTime - lastAlertCheck >= pgm_read_word(&ALERT_CHECK_INTERVAL)) {
    if (currentAlertLevel == NO_ALERT) {
      if (checkFailSafeConditions(currentTime)) {
        startAlert(LEVEL_1);
      }
    }
  }


  if (components.ledStrip && currentTime - lastLEDUpdate >= pgm_read_word(&LED_UPDATE_INTERVAL)) {
    updateLEDStrip();
    lastLEDUpdate = currentTime;
  }

  if (components.lcd && currentTime - lastLCDUpdate >= pgm_read_word(&LCD_UPDATE_INTERVAL)) {
    updateLCDDisplay();
    lastLCDUpdate = currentTime;
  }

  if (currentTime - lastDataTransmission >= pgm_read_word(&DATA_TRANSMISSION_INTERVAL)) {
    transmitSensorData();
    lastDataTransmission = currentTime;
  }
}

void readAllSensors() {
  readHeartRateAndSpO2();
  if (components.accelerometer) {
    readAccelerometer();
  }
  readTouchSensors();
}

void readHeartRateAndSpO2() {
  heartRate = getSimulatedHeartRate();
  spO2 = getSimulatedSpO2();
}

void readAccelerometer() {
  accelerometer.readAccel(&accelX, &accelY, &accelZ);

  // Update history for jerk detection
  accelHistory[accelHistoryIndex] = accelX;
  accelHistoryIndex = (accelHistoryIndex + 1) % 3;
}

void readTouchSensors() {
  static uint8_t lastStableTouch = 0;
  static uint32_t lastDebounceTime = 0;
  const uint16_t debounceDelay = 50;

  uint8_t currentRawTouch = 0;

  if (digitalRead(TOUCH_SENSOR_1)) currentRawTouch |= 0x01;
  if (digitalRead(TOUCH_SENSOR_2)) currentRawTouch |= 0x02;
  if (digitalRead(TOUCH_SENSOR_3)) currentRawTouch |= 0x04;
  if (digitalRead(TOUCH_SENSOR_4)) currentRawTouch |= 0x08;

  if (currentRawTouch != lastStableTouch) {
    if (millis() - lastDebounceTime >= debounceDelay) {
      lastStableTouch = currentRawTouch;
      touchSensors = currentRawTouch;
      lastTouchChange = millis();
      touchSensorFailCount = 0;
    }
    lastDebounceTime = millis();
  }
}


// Step 4: Modify your checkSuddenJerks() function to respect fail-safe mode:
void checkSuddenJerks(uint32_t currentTime) {
  if (!components.accelerometer) return;

  int16_t maxChange = 0;
  for (int i = 0; i < 2; i++) {
    int16_t change = abs(accelHistory[i] - accelHistory[i + 1]);
    if (change > maxChange) {
      maxChange = change;
    }
  }

  if (maxChange > pgm_read_word(&JERK_THRESHOLD)) {
    if (currentTime - lastJerkTime > pgm_read_word(&MIN_JERK_INTERVAL)) {
      lastJerkTime = currentTime;

      if (jerkCount == 0) {
        jerkWindowStart = currentTime;
      }

      jerkCount++;
      Serial.print(F("Jerk detected: "));
      Serial.print(maxChange);
      Serial.print(F(" Count: "));
      Serial.println(jerkCount);

      if (jerkCount >= pgm_read_word(&JERK_COUNT_THRESHOLD)) {
        if (currentTime - jerkWindowStart <= pgm_read_word(&JERK_WINDOW)) {
          // Only trigger alert if in fail-safe mode OR if no current alert
          if (components.failSafeMode || currentAlertLevel == NO_ALERT) {
            Serial.println(F("Jerk alert triggered"));
            startAlert(LEVEL_1);
          }
        }
        jerkCount = 0;
        jerkWindowStart = currentTime;
      }
    }
  }

  if (currentTime - jerkWindowStart > pgm_read_word(&JERK_WINDOW)) {
    jerkCount = 0;
  }
}

void checkSerialCommunication() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == F("No")) {
      lastPiCommunication = millis();
      components.piOnline = true;
      components.failSafeMode = false;
      Serial.println(F("Pi back online - Fail-safe mode deactivated"));
    } else if (cmd == F("Level 01")) {
      lastPiCommunication = millis();
      components.piOnline = true;
      components.failSafeMode = false;  // Pi is online, so no fail-safe needed

      // Pi command always triggers alert regardless of mode
      if (currentAlertLevel == NO_ALERT) {
        Serial.println(F("Pi command: Level 01 alert"));
        startAlert(LEVEL_1);
      }
    } else if (cmd == F("TEST")) {
      runSelfTest();
      lastPiCommunication = millis();
      components.piOnline = true;
      components.failSafeMode = false;
    }
  }
}

void checkPiStatus(uint32_t currentTime) {
  if (currentTime - lastPiCommunication > pgm_read_word(&PI_TIMEOUT)) {
    if (components.piOnline) {
      components.piOnline = false;
      components.failSafeMode = true;
    }
  }
}


bool checkFailSafeConditions(uint32_t currentTime) {
  // Only check fail-safe conditions when Pi is offline
  if (!components.failSafeMode) return false;

  bool alertCondition = false;
  
  // Static variables to track poor grip timing
  static uint32_t poorGripStartTime = 0;
  static bool poorGripDetected = false;

  // Count active sensors
  uint8_t activeSensors = 0;
  for (int i = 0; i < 4; i++) {
    if (touchSensors & (1 << i)) activeSensors++;
  }

  // Check if grip is poor (1 or fewer sensors active)
  if (activeSensors <= 1) {
    
    if (!poorGripDetected) {
      // First time detecting poor grip
      poorGripStartTime = currentTime;
      poorGripDetected = true;
      touchSensorFailCount = 0;
      Serial.println(F("Fail-safe: Poor grip detected, starting timer"));
    }
    
    // Check if poor grip has persisted for the required time
    if (currentTime - poorGripStartTime >= pgm_read_word(&TOUCH_STABILITY_TIME)) {
      touchSensorFailCount++;
      Serial.print(F("Fail-safe: Poor grip count: "));
      Serial.println(touchSensorFailCount);

      if (touchSensorFailCount >= pgm_read_word(&TOUCH_FAIL_THRESHOLD)) {
        Serial.println(F("Fail-safe: Touch sensor alert triggered"));
        alertCondition = true;
        poorGripDetected = false; // Reset for next time
      }
    }
  } else {
    // Good grip detected, reset everything
    if (poorGripDetected) {
      Serial.println(F("Fail-safe: Good grip detected, resetting counters"));
    }
    touchSensorFailCount = 0;
    poorGripDetected = false;
  }

  return alertCondition;
}

void transmitSensorData() {
  Serial.print(F("SENSOR_DATA:HR="));
  Serial.print(heartRate);
  Serial.print(F(",SPO2="));
  Serial.print(spO2);
  Serial.print(F(",ACCEL_X="));
  Serial.print(accelX);
  Serial.print(F(",ACCEL_Y="));
  Serial.print(accelY);
  Serial.print(F(",ACCEL_Z="));
  Serial.print(accelZ);
  Serial.print(F(",TOUCH="));
  Serial.print(touchSensors);
  Serial.print(F(",ALERT_LEVEL="));
  Serial.print(currentAlertLevel);
  Serial.print(F(",MOTOR_SPEED="));
  Serial.print(motorSpeed);
  Serial.print(F(",FAIL_SAFE="));
  Serial.println(components.failSafeMode ? '1' : '0');
}

void startAlert(AlertLevel level) {
  currentAlertLevel = level;
  alertActive = true;
  alertStartTime = millis();
  activateAlertLevel(level);
}

void activateAlertLevel(AlertLevel level) {
  switch (level) {
    case LEVEL_1:
      if (components.vibrator) {
        activateVibration(100);
      }
      break;

    case LEVEL_2:
      if (components.vibrator) {
        activateVibration(180);
      }
      if (components.buzzer) {
        playLevel2Siren();
      }
      Serial.println(F("Level 02"));
      break;

    case LEVEL_3:
      if (components.vibrator) {
        activateVibration(255);
      }
      if (components.buzzer) {
        playLevel3Siren();
      }
      startMotorReduction();
      Serial.println(F("Level 03"));
      break;
  }
}

void processAlertSystem(uint32_t currentTime) {
  if (!alertActive) return;

  uint32_t alertDuration = currentTime - alertStartTime;

  switch (currentAlertLevel) {
    case LEVEL_1:
      if (alertDuration > pgm_read_word(&LEVEL_1_DURATION)) {
        startAlert(LEVEL_2);
      }
      break;

    case LEVEL_2:
      playLevel2Siren();
      if (alertDuration > pgm_read_word(&LEVEL_2_DURATION)) {
        startAlert(LEVEL_3);
      }
      break;

    case LEVEL_3:
      updateMotorReduction();
      if (components.buzzer) {
        playLevel3Siren();
      }
      break;
  }
}

void checkTactileButton() {
  static bool lastButtonState = false;
  static uint32_t buttonPressStart = 0;
  static bool longPressDetected = false;
  
  bool currentButtonState = !digitalRead(TACTILE_BUTTON_PIN);
  
  // Button just pressed
  if (currentButtonState && !lastButtonState) {
    buttonPressStart = millis();
    longPressDetected = false;
  }
  
  // Button being held down
  if (currentButtonState && lastButtonState) {
    // Check for long press (3 seconds) - only if car is stopped
    if (!longPressDetected && motorStopped && 
        (millis() - buttonPressStart >= 3000)) {
      longPressDetected = true;
      
      // Restart the car
      motorSpeed = normalMotorSpeed;
      analogWrite(MOTOR_SPEED_PIN, motorSpeed);
      motorStopped = false;
      
      Serial.println(F("Long press detected - Car restarted"));
      
      // Optional: Brief LED indication for restart
      if (components.ledStrip) {
        fill_solid(leds, NUM_LEDS, CRGB::Green);
        FastLED.show();
        delay(500);
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
      }
    }
  }
  
  // Button just released
  if (!currentButtonState && lastButtonState) {
    uint32_t pressDuration = millis() - buttonPressStart;
    
    // Short press (less than 3 seconds) - stop alerts
    if (pressDuration < 3000 && !longPressDetected) {
      tactilePressed = true;
      stopAllAlerts();
    }
    
    tactilePressed = false;
    longPressDetected = false;
  }
  
  lastButtonState = currentButtonState;
}

void stopAllAlerts() {
  bool wasCarStopped = motorStopped;  // Remember if car was stopped
  
  currentAlertLevel = NO_ALERT;
  alertActive = false;

  if (components.buzzer) {
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }

  if (components.vibrator) {
    digitalWrite(VIBRATION_MOTOR_1, LOW);
    digitalWrite(VIBRATION_MOTOR_2, LOW);
    analogWrite(VIBRATION_MOTOR_1, 0);
    analogWrite(VIBRATION_MOTOR_2, 0);
  }

  if (components.ledStrip) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
  }

  // Only restart motor if it wasn't completely stopped
  if (!wasCarStopped) {
    motorSpeed = normalMotorSpeed;
    analogWrite(MOTOR_SPEED_PIN, motorSpeed);
  }
  // If car was stopped, keep it stopped for safety
  
  jerkCount = 0;
  jerkWindowStart = millis();
  touchSensorFailCount = 0;
  Serial.println(F("STOP"));
}

void activateVibration(uint8_t intensity) {
  if (!components.vibrator) return;
  analogWrite(VIBRATION_MOTOR_1, intensity);
  analogWrite(VIBRATION_MOTOR_2, intensity);

  // Also set digital pins to HIGH for consistent behavior
  digitalWrite(VIBRATION_MOTOR_1, HIGH);
  digitalWrite(VIBRATION_MOTOR_2, HIGH);
}

void playLevel3Siren() {
  if (!components.buzzer) return;

  static uint32_t lastSirenChange = 0;
  static bool sirenHigh = false;
  static bool toneActive = false;

  if (millis() - lastSirenChange > 150) {
    if (toneActive) {
      noTone(BUZZER_PIN);
      toneActive = false;
    }

    tone(BUZZER_PIN, sirenHigh ? 1200 : 600, 130);
    toneActive = true;
    sirenHigh = !sirenHigh;
    lastSirenChange = millis();
  }
}

void playLevel2Siren() {
  if (!components.buzzer) return;

  static uint32_t lastToneTime = 0;
  static uint8_t tonePattern = 0;
  static bool toneActive = false;

  if (millis() - lastToneTime > 200) {
    if (toneActive) {
      noTone(BUZZER_PIN);
      toneActive = false;
    }

    switch (tonePattern) {
      case 0:
        tone(BUZZER_PIN, 800, 180);
        break;
      case 1:
        tone(BUZZER_PIN, 1000, 180);
        break;
      case 2:
        tone(BUZZER_PIN, 1200, 180);
        break;
      case 3:
        tone(BUZZER_PIN, 1000, 180);
        break;
    }

    toneActive = true;
    tonePattern = (tonePattern + 1) % 4;
    lastToneTime = millis();
  }
}

void startMotorReduction() {
  motorStopped = false;
}

void updateMotorReduction() {
  static uint32_t lastSpeedReduction = 0;

  if (millis() - lastSpeedReduction > 200 && !motorStopped) {
    if (motorSpeed > 0) {
      motorSpeed -= 10;
      if (motorSpeed < 0) motorSpeed = 0;
      analogWrite(MOTOR_SPEED_PIN, motorSpeed);
    } else {
      motorStopped = true;
    }
    lastSpeedReduction = millis();
  }
}

void updateLEDStrip() {
  if (!components.ledStrip) return;

  if (!alertActive) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  } else {
    static bool ledState = false;
    ledState = !ledState;

    if (ledState) {
      CRGB color;
      uint8_t brightness;

      switch (currentAlertLevel) {
        case LEVEL_1:
          color = CRGB::Yellow;
          brightness = 50;
          break;
        case LEVEL_2:
          color = CRGB::Orange;
          brightness = 100;
          break;
        case LEVEL_3:
          color = CRGB::Red;
          brightness = 200;
          break;
      }

      fill_solid(leds, NUM_LEDS, color);
      FastLED.setBrightness(brightness);
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
    }
  }
  FastLED.show();
}

void updateLCDDisplay() {
  if (!components.lcd) return;

  lcd.clear();

  if (alertActive) {
    lcd.setCursor(0, 0);
    lcd.print(F("Alert Level "));
    lcd.print(currentAlertLevel);

    lcd.setCursor(0, 1);
    if (motorStopped) {
      lcd.print(F("CAR STOPPED"));
    } else {
      lcd.print(F("SLOWING DOWN"));
    }
  } else {
    lcd.setCursor(0, 0);
    lcd.print(F("HR:"));
    lcd.print(getSimulatedHeartRate());
    lcd.print(F(" O2:"));
    lcd.print(getSimulatedSpO2());

    lcd.setCursor(0, 1);
    if (components.failSafeMode) {
      lcd.print(F("FAIL-SAFE"));
    } else if (!components.piOnline) {
      lcd.print(F("Pi OFFLINE"));
    } else {
      lcd.print(F("SYSTEM OK"));
    }
  }
}

uint8_t getSimulatedHeartRate() {
  static uint32_t lastHRUpdate = 0;
  static uint8_t currentHR = 75;
  static int8_t hrDirection = 1;
  static uint16_t hrUpdateInterval = 800;

  if (millis() - lastHRUpdate > hrUpdateInterval) {
    currentHR += hrDirection;

    if (currentHR >= 85) {
      hrDirection = -1;
      hrUpdateInterval = 600 + random(0, 400);
    } else if (currentHR <= 65) {
      hrDirection = 1;
      hrUpdateInterval = 600 + random(0, 400);
    }

    if (random(0, 10) == 0) {
      currentHR += random(-2, 3);
      currentHR = constrain(currentHR, 60, 90);
    }

    lastHRUpdate = millis();
  }

  return currentHR;
}

uint8_t getSimulatedSpO2() {
  static uint32_t lastSpO2Update = 0;
  static uint8_t currentSpO2 = 98;
  static int8_t spo2Direction = 1;
  static uint16_t spo2UpdateInterval = 1200;

  if (millis() - lastSpO2Update > spo2UpdateInterval) {
    if (random(0, 3) == 0) {
      currentSpO2 += spo2Direction;

      if (currentSpO2 >= 100) {
        spo2Direction = -1;
        spo2UpdateInterval = 1000 + random(0, 800);
      } else if (currentSpO2 <= 96) {
        spo2Direction = 1;
        spo2UpdateInterval = 1000 + random(0, 800);
      }

      currentSpO2 = constrain(currentSpO2, 95, 100);
    }

    lastSpO2Update = millis();
  }

  return currentSpO2;
}

void runSelfTest() {
  Serial.println(F("Running self-test..."));

  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  tone(BUZZER_PIN, 1000, 200);
  delay(300);
  noTone(BUZZER_PIN);

  digitalWrite(VIBRATION_MOTOR_1, HIGH);
  digitalWrite(VIBRATION_MOTOR_2, HIGH);
  delay(200);
  digitalWrite(VIBRATION_MOTOR_1, LOW);
  digitalWrite(VIBRATION_MOTOR_2, LOW);

  Serial.println(F("Self-test complete"));
}