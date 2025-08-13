#include <DS18B20.h>
#include <BluetoothSerial.h>
#include <SPI.h>
#include "SD.h"
#include <time.h>
#include <Adafruit_MAX31865.h>
#include <L298N.h>
#include <PID_v1_bc.h>

// Variable declarations
const int msfanPin = 16;
int msfanPwmValue = 30;

#define NEW_HEATER_PIN 22
const int NEW_HEATER_PWM_CHANNEL = 1;
const int NEW_HEATER_PWM_FREQ = 25000;
const int NEW_HEATER_PWM_RESOLUTION = 8;
int MAX_PWM_LIMIT = 70;  // ~4% of 255, now variable
const float PWM_MIN_THRESHOLD = 0.5;  // DEBUG: Minimum pidOutput threshold to allow PWM

#define K_CS 13
const int heaterPin = 12;

float heaterThresholdLow = 20.0;
float heaterThresholdHigh = 22.0;
int heaterState = 0;
int heaterManualMode = 0;
int manualHeaterState = 0;
int heaterSensor = 3;
float heatertemp = 0;

int newHeaterState = 0;
int newHeaterManualMode = 0;
double newHeaterPwmValue = 0;
float newHeaterThresholdLow = 38.0;
float newHeaterThresholdHigh = 42.0;
int newHeaterSensor = 4;
float newHeaterTemp = 0;
float rampRate = 0;
float rampMaxTemp = 0;
unsigned long lastRampTime = 0;
double rampTargetTemp = 0;
double newHeaterTargetTemp = 0;

double pidInput = 0;
double pidOutput = 0;
double pidSetpoint = 0;
float Kp = 5;
float Ki = 0.0;
float Kd = 0.01;
PID newHeaterPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

float kTemp = 0;
float lastKTemp = 0;

unsigned long startMillis;

SPIClass hspi(HSPI);
#define HSPI_MISO 19
#define HSPI_MOSI 23
#define HSPI_SCK  14
#define HSPI_CS   17
#define THERMO_CS 21

const unsigned int IN1 = 27;
const unsigned int IN2 = 26;
const unsigned int EN = 25;

L298N motor(EN, IN1, IN2);

DS18B20 ds(4);

BluetoothSerial SerialBT;

const int fanPin = 18;
int dutyPercent = 20;
int pwmValue = 20;

float temperatureThreshold_low = 25.0;
float temperatureThreshold_high = 40.0;
int fan_pwm_min = 20;
int fan_pwm_max = 100;
int manualMode = 0;
int exp_ID = 0;

int mixspeed = 0;

unsigned long tempInterval = 2000;
unsigned long fanControlInterval = 100;
unsigned long dataSendInterval = 1000;

unsigned long lastTempTime = 0;
unsigned long lastFanControlTime = 0;
unsigned long lastDataSendTime = 0;

float curExpStartTime = 0;
float curMins = 0;
float curElapsedTime = 0;

float currentTemp1 = 0;
float currentTemp2 = 0;
float thermotemp = 0;
float TempDiff = 0;

uint8_t sensor1Address[8] = {0x28, 0x43, 0xB8, 0xBB, 0x00, 0x00, 0x00, 0x46};
uint8_t sensor2Address[8] = {0x28, 0x72, 0x69, 0x35, 0x00, 0x00, 0x00, 0xC3};

int num_dev = 0;
int tar_sensor = 1;
int compar_sensor = 3;
float currentTemp = 0;
int write_SD = 0;
int clear_SD = 0;

Adafruit_MAX31865 thermo = Adafruit_MAX31865(THERMO_CS, &hspi);
#define RREF      430.0
#define RNOMINAL  100.0


//float readKTypeTemperature() {
//  digitalWrite(HSPI_CS, HIGH);
//  digitalWrite(THERMO_CS, HIGH);
//  delay(250);
//  float tempSum = 0;
//  int validReadings = 0;
//  const int NUM_READINGS = 5;
//  hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  // ← обязательно!
//  for (int i = 0; i < NUM_READINGS; i++) {
//    digitalWrite(K_CS, LOW);
//    delayMicroseconds(10);
//    uint16_t data = 0;
//    data = hspi.transfer(0x00) << 8;
//    data |= hspi.transfer(0x00);
//    digitalWrite(K_CS, HIGH);
//    Serial.print("MAX6675 Reading ");
//    Serial.print(i + 1);
//    Serial.print(": 0x");
//    Serial.println(data, HEX);
//    if (!(data & 0x0004)) {
//      data >>= 3;
//      float temp = data * 0.25;
//      tempSum += temp;
//      validReadings++;
//    } else {
//      Serial.println("MAX6675 Error: Thermocouple disconnected");
//    }
//    delay(10);
//  }
//  hspi.endTransaction();  // ← обязательно!
//  if (validReadings > 0) {
//    float avgTemp = tempSum / validReadings;
//    if (lastKTemp == 0) lastKTemp = avgTemp;
//    avgTemp = 0.8 * lastKTemp + 0.2 * avgTemp;
//    lastKTemp = avgTemp;
//    Serial.print("MAX6675 Filtered Temp: ");
//    Serial.println(avgTemp);
//    return avgTemp;
//  } else {
//    Serial.println("MAX6675 Error: No valid readings");
//    return -1;
//  }
//}

float readKTypeTemperature() {
    digitalWrite(HSPI_CS, HIGH);
    digitalWrite(THERMO_CS, HIGH);
    hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  // ← обязательно!
    digitalWrite(K_CS, LOW);
    delayMicroseconds(10);
    uint16_t data = 0;
    data = hspi.transfer(0x00) << 8;
    data |= hspi.transfer(0x00);
    digitalWrite(K_CS, HIGH);
    hspi.endTransaction();  // ← обязательно!

    if (data & 0x0004) return -1; // ошибка термопары
    data >>= 3;
    return data * 0.25;
}

void setup() {
  pinMode(msfanPin, OUTPUT);
  digitalWrite(msfanPin, LOW);
  ledcSetup(0, 25000, 8);
  ledcAttachPin(msfanPin, 0);
  ledcWrite(0, msfanPwmValue);

  pinMode(NEW_HEATER_PIN, OUTPUT);
  ledcSetup(NEW_HEATER_PWM_CHANNEL, NEW_HEATER_PWM_FREQ, NEW_HEATER_PWM_RESOLUTION);
  ledcAttachPin(NEW_HEATER_PIN, NEW_HEATER_PWM_CHANNEL);
  ledcWrite(NEW_HEATER_PWM_CHANNEL, 0);

  pinMode(K_CS, OUTPUT);
  digitalWrite(K_CS, HIGH);

  newHeaterPID.SetMode(AUTOMATIC);
  newHeaterPID.SetOutputLimits(0, MAX_PWM_LIMIT);
  newHeaterPID.SetSampleTime(2000);

  SerialBT.setPin("1515");
  pinMode(fanPin, OUTPUT);
  Serial.begin(115200);
  SerialBT.begin("ESP32_Fan_Control_N");
  num_dev = ds.getNumberOfDevices();
  pinMode(HSPI_SCK, OUTPUT);
  pinMode(HSPI_MOSI, OUTPUT);
  pinMode(HSPI_MISO, INPUT);
  pinMode(HSPI_CS, OUTPUT);
  pinMode(THERMO_CS, OUTPUT);
  digitalWrite(HSPI_CS, HIGH);
  digitalWrite(THERMO_CS, HIGH);

  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, -1);
  if (!SD.begin(HSPI_CS, hspi)) {
    Serial.println("SD Card MOUNT FAIL");
  }
  if (!thermo.begin(MAX31865_2WIRE)) {
    Serial.println("Device MOUNT FAIL");
    while (1);
  }

  startMillis = millis();
  curExpStartTime = millis();
  motor.setSpeed(0);
  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  curMins = currentMillis / 60000;
  curElapsedTime = (currentMillis - curExpStartTime) / 60000;

  if (currentMillis - lastTempTime >= tempInterval) {
    lastTempTime = currentMillis;
    digitalWrite(HSPI_CS, HIGH);
    digitalWrite(THERMO_CS, LOW);
    thermotemp = thermo.temperature(RNOMINAL, RREF);
    digitalWrite(THERMO_CS, HIGH);

    kTemp = readKTypeTemperature();

    if (kTemp != -1) {
      ds.select(sensor1Address);
      if (num_dev == 2) {
        currentTemp1 = ds.getTempC();
        ds.select(sensor2Address);
        currentTemp2 = ds.getTempC();
      }

      if (tar_sensor == 1) currentTemp = currentTemp1;
      if (tar_sensor == 2) currentTemp = currentTemp2;
      if (tar_sensor == 3) currentTemp = thermotemp;
      if (tar_sensor == 4) currentTemp = kTemp;

      TempDiff = currentTemp - currentTemp1;

      if (heaterSensor == 1) heatertemp = currentTemp1;
      if (heaterSensor == 2) heatertemp = currentTemp2;
      if (heaterSensor == 3) heatertemp = thermotemp;
      if (heaterSensor == 4) heatertemp = kTemp;

      if (newHeaterSensor == 1) newHeaterTemp = currentTemp1;
      if (newHeaterSensor == 2) newHeaterTemp = currentTemp2;
      if (newHeaterSensor == 3) newHeaterTemp = thermotemp;
      if (newHeaterSensor == 4) newHeaterTemp = kTemp;

      if (heaterManualMode == 0) {
        if (heatertemp < heaterThresholdLow && heaterState == 0) {
          digitalWrite(heaterPin, HIGH);
          heaterState = 1;
        } else if (heatertemp > heaterThresholdHigh && heaterState == 1) {
          digitalWrite(heaterPin, LOW);
          heaterState = 0;
        }
      } else {
        digitalWrite(heaterPin, manualHeaterState);
        heaterState = manualHeaterState;
      }

      if (newHeaterState == 1) {
        if (newHeaterManualMode == 0) {
          if (rampRate > 0 && rampMaxTemp > 0) {
            if (currentMillis - lastRampTime >= 60000) {
              lastRampTime = currentMillis;
              rampTargetTemp += rampRate;
              if (rampTargetTemp > rampMaxTemp) rampTargetTemp = rampMaxTemp;
            }
            newHeaterTargetTemp = rampTargetTemp;
          } else {
            newHeaterTargetTemp = (newHeaterThresholdLow + newHeaterThresholdHigh) / 2.0;
          }
          pidInput = newHeaterTemp;
          pidSetpoint = newHeaterTargetTemp;
          if (newHeaterTemp > newHeaterThresholdHigh + 2.0) {
            newHeaterPwmValue = 0;
            Serial.println("Overheat protection: PWM set to 0");
          } else if (newHeaterTemp > newHeaterTargetTemp) {
            newHeaterPwmValue = 0;
            Serial.println("Temp above setpoint: PWM set to 0");
          } else {
            newHeaterPID.Compute();
            newHeaterPwmValue = pidOutput;
            if (newHeaterPwmValue < PWM_MIN_THRESHOLD) {
              newHeaterPwmValue = 0;
              Serial.println("pidOutput below threshold: PWM set to 0");
            }
            if (newHeaterPwmValue > MAX_PWM_LIMIT) newHeaterPwmValue = MAX_PWM_LIMIT;
          }
          Serial.print("PID Input: ");
          Serial.print(pidInput);
          Serial.print(", Setpoint: ");
          Serial.print(pidSetpoint);
          Serial.print(", Error: ");
          Serial.print(pidSetpoint - pidInput);
          Serial.print(", pidOutput: ");
          Serial.print(pidOutput);
          Serial.print(", PWM: ");
          Serial.println(newHeaterPwmValue);
          ledcWrite(NEW_HEATER_PWM_CHANNEL, newHeaterPwmValue);
        } else {
          ledcWrite(NEW_HEATER_PWM_CHANNEL, newHeaterPwmValue);
        }
      } else {
        newHeaterPwmValue = 0;
        ledcWrite(NEW_HEATER_PWM_CHANNEL, 0);
      }

      if (manualMode == 0) {
        if (currentTemp > temperatureThreshold_low) {
          pwmValue = map(currentTemp, temperatureThreshold_low, temperatureThreshold_high, fan_pwm_min, fan_pwm_max);
        } else {
          pwmValue = 0;
        }
      }

      if (write_SD == 1) {
        String payload = "{";
        payload += "\"temperature\":" + String(currentTemp) + ",";
        payload += "\"onetemp\":" + String(currentTemp1) + ",";
        payload += "\"twotemp\":" + String(currentTemp2) + ",";
        payload += "\"tempdiff\":" + String(TempDiff) + ",";
        payload += "\"dutyPercent\":" + String(dutyPercent) + ",";
        payload += "\"manualMode\":" + String(manualMode) + ",";
        payload += "\"pwmValue\":" + String(pwmValue) + ",";
        payload += "\"thresholdLow\":" + String(temperatureThreshold_low) + ",";
        payload += "\"thresholdHigh\":" + String(temperatureThreshold_high) + ",";
        payload += "\"fanPwmMin\":" + String(fan_pwm_min) + ",";
        payload += "\"fanPwmMax\":" + String(fan_pwm_max) + ",";
        payload += "\"tempInterval\":" + String(tempInterval) + ",";
        payload += "\"fanInterval\":" + String(fanControlInterval) + ",";
        payload += "\"dataSendInterval\":" + String(dataSendInterval) + ",";
        payload += "\"mins\":" + String(curMins) + ",";
        payload += "\"elapsedMins\":" + String(curElapsedTime) + ",";
        payload += "\"expID\":" + String(exp_ID) + ",";
        payload += "\"ishead\":" + String(0) + ",";
        payload += "\"writeon\":" + String(write_SD) + ",";
        payload += "\"tarSensor\":" + String(tar_sensor) + ",";
        payload += "\"thermotemp\":" + String(thermotemp) + ",";
        payload += "\"kTemp\":" + String(kTemp) + ",";
        payload += "\"heaterState\":" + String(heaterState) + ",";
        payload += "\"heaterThresholdLow\":" + String(heaterThresholdLow) + ",";
        payload += "\"heaterThresholdHigh\":" + String(heaterThresholdHigh) + ",";
        payload += "\"heaterSensor\":" + String(heaterSensor) + ",";
        payload += "\"newHeaterState\":" + String(newHeaterState) + ",";
        payload += "\"newHeaterPwmValue\":" + String(newHeaterPwmValue) + ",";
        payload += "\"newHeaterThresholdLow\":" + String(newHeaterThresholdLow) + ",";
        payload += "\"newHeaterThresholdHigh\":" + String(newHeaterThresholdHigh) + ",";
        payload += "\"newHeaterSensor\":" + String(newHeaterSensor) + ",";
        payload += "\"pidKp\":" + String(Kp) + ",";
        payload += "\"pidKi\":" + String(Ki) + ",";
        payload += "\"pidKd\":" + String(Kd) + ",";
        payload += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
        payload += "\"heaterManualMode\":" + String(heaterManualMode) + ",";
        payload += "\"newHeaterManualMode\":" + String(newHeaterManualMode) + ",";
        payload += "\"maxPwmLimit\":" + String(MAX_PWM_LIMIT);
        payload += "}";
        digitalWrite(THERMO_CS, HIGH);
        digitalWrite(HSPI_CS, LOW);
        digitalWrite(K_CS, HIGH);
        File file = SD.open("/templog.txt", FILE_APPEND);
        if (file) {
          file.println(payload);
          file.close();
        } else {
          Serial.println("Failed to append to /templog.txt");
        }
        digitalWrite(HSPI_CS, HIGH);
      }
    }
  }

  if (currentMillis - lastDataSendTime >= dataSendInterval) {
    lastDataSendTime = currentMillis;
    String payload = "{";
    payload += "\"temperature\":" + String(currentTemp) + ",";
    payload += "\"onetemp\":" + String(currentTemp1) + ",";
    payload += "\"twotemp\":" + String(currentTemp2) + ",";
    payload += "\"tempdiff\":" + String(TempDiff) + ",";
    payload += "\"dutyPercent\":" + String(dutyPercent) + ",";
    payload += "\"manualMode\":" + String(manualMode) + ",";
    payload += "\"pwmValue\":" + String(pwmValue) + ",";
    payload += "\"thresholdLow\":" + String(temperatureThreshold_low) + ",";
    payload += "\"thresholdHigh\":" + String(temperatureThreshold_high) + ",";
    payload += "\"fanPwmMin\":" + String(fan_pwm_min) + ",";
    payload += "\"fanPwmMax\":" + String(fan_pwm_max) + ",";
    payload += "\"tempInterval\":" + String(tempInterval) + ",";
    payload += "\"fanInterval\":" + String(fanControlInterval) + ",";
    payload += "\"dataSendInterval\":" + String(dataSendInterval) + ",";
    payload += "\"mins\":" + String(curMins) + ",";
    payload += "\"elapsedMins\":" + String(curElapsedTime) + ",";
    payload += "\"expID\":" + String(exp_ID) + ",";
    payload += "\"ishead\":" + String(0) + ",";
    payload += "\"writeon\":" + String(write_SD) + ",";
    payload += "\"tarSensor\":" + String(tar_sensor) + ",";
    payload += "\"thermotemp\":" + String(thermotemp) + ",";
    payload += "\"kTemp\":" + String(kTemp) + ",";
    payload += "\"heaterState\":" + String(heaterState) + ",";
    payload += "\"heaterThresholdLow\":" + String(heaterThresholdLow) + ",";
    payload += "\"heaterThresholdHigh\":" + String(heaterThresholdHigh) + ",";
    payload += "\"heaterSensor\":" + String(heaterSensor) + ",";
    payload += "\"newHeaterState\":" + String(newHeaterState) + ",";
    payload += "\"newHeaterPwmValue\":" + String(newHeaterPwmValue) + ",";
    payload += "\"newHeaterThresholdLow\":" + String(newHeaterThresholdLow) + ",";
    payload += "\"newHeaterThresholdHigh\":" + String(newHeaterThresholdHigh) + ",";
    payload += "\"newHeaterSensor\":" + String(newHeaterSensor) + ",";
    payload += "\"pidKp\":" + String(Kp) + ",";
    payload += "\"pidKi\":" + String(Ki) + ",";
    payload += "\"pidKd\":" + String(Kd) + ",";
    payload += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
    payload += "\"heaterManualMode\":" + String(heaterManualMode) + ",";
    payload += "\"newHeaterManualMode\":" + String(newHeaterManualMode) + ",";
    payload += "\"maxPwmLimit\":" + String(MAX_PWM_LIMIT);
    payload += "}";
    SerialBT.println(payload);
  }

  if (currentMillis - lastFanControlTime >= fanControlInterval) {
    lastFanControlTime = currentMillis;
    if (manualMode == 1) {
      pwmValue = dutyPercent;
    }
    unsigned long onTime = fanControlInterval * pwmValue / 100;
    unsigned long offTime = fanControlInterval - onTime;
    if (pwmValue > 0) {
      digitalWrite(fanPin, HIGH);
      delay(onTime);
    }
    if (pwmValue < 100) {
      digitalWrite(fanPin, LOW);
      delay(offTime);
    }
  }

  if (SerialBT.available()) {
    String command = SerialBT.readString();
    if (command.startsWith("msfan_pwm:")) {
      msfanPwmValue = command.substring(10).toInt();
      msfanPwmValue = constrain(msfanPwmValue, 0, 255);
      ledcWrite(0, msfanPwmValue);
    }
    if (command.startsWith("heater_low:")) {
      heaterThresholdLow = command.substring(11).toFloat();
    }
    if (command.startsWith("heater_high:")) {
      heaterThresholdHigh = command.substring(12).toFloat();
    }
    if (command.startsWith("heater_manual:")) {
      heaterManualMode = command.substring(14).toInt();
    }
    if (command.startsWith("heater_state:") && heaterManualMode) {
      manualHeaterState = command.substring(13).toInt();
    }
    if (command.startsWith("heater_sensor:")) {
      heaterSensor = command.substring(14).toInt();
    }
    if (command.startsWith("new_heater_state:")) {
      newHeaterState = command.substring(17).toInt();
    }
    if (command.startsWith("new_heater_manual:")) {
      newHeaterManualMode = command.substring(18).toInt();
      if (newHeaterManualMode == 0) newHeaterPwmValue = 0;
    }
    if (command.startsWith("new_heater_pwm:") && newHeaterManualMode) {
      newHeaterPwmValue = map(command.substring(15).toInt(), 0, 100, 0, MAX_PWM_LIMIT);
    }
    if (command.startsWith("new_heater_low:")) {
      newHeaterThresholdLow = command.substring(15).toFloat();
    }
    if (command.startsWith("new_heater_high:")) {
      newHeaterThresholdHigh = command.substring(16).toFloat();
    }
    if (command.startsWith("new_heater_sensor:")) {
      newHeaterSensor = command.substring(18).toInt();
    }
    if (command.startsWith("new_heater_ramp:")) {
      String params = command.substring(16);
      int commaIndex = params.indexOf(',');
      if (commaIndex != -1) {
        rampRate = params.substring(0, commaIndex).toFloat();
        rampMaxTemp = params.substring(commaIndex + 1).toFloat();
        rampTargetTemp = newHeaterTemp;
        lastRampTime = millis();
      }
    }
    if (command.startsWith("pid_kp:")) {
      Kp = command.substring(7).toFloat();
      newHeaterPID.SetTunings(Kp, Ki, Kd);
    }
    if (command.startsWith("pid_ki:")) {
      Ki = command.substring(7).toFloat();
      newHeaterPID.SetTunings(Kp, Ki, Kd);
    }
    if (command.startsWith("pid_kd:")) {
      Kd = command.substring(7).toFloat();
      newHeaterPID.SetTunings(Kp, Ki, Kd);
    }
    if (command.startsWith("max_pwm_limit:")) {
      MAX_PWM_LIMIT = command.substring(14).toInt();
      MAX_PWM_LIMIT = constrain(MAX_PWM_LIMIT, 0, 255);
      newHeaterPID.SetOutputLimits(0, MAX_PWM_LIMIT);
      Serial.print("New MAX_PWM_LIMIT: ");
      Serial.println(MAX_PWM_LIMIT);
    }
    if (command.startsWith("threshold_low:")) {
      temperatureThreshold_low = command.substring(14).toFloat();
    }
    if (command.startsWith("threshold_high:")) {
      temperatureThreshold_high = command.substring(15).toFloat();
    }
    if (command.startsWith("manual:")) {
      manualMode = command.substring(7).toInt();
    }
    if (command.startsWith("speed:") && manualMode) {
      dutyPercent = command.substring(6).toInt();
    }
    if (command.startsWith("fan_pwm_min:")) {
      fan_pwm_min = command.substring(12).toInt();
    }
    if (command.startsWith("fan_pwm_max:")) {
      fan_pwm_max = command.substring(12).toInt();
    }
    if (command.startsWith("compar_sensor:")) {
      compar_sensor = command.substring(14).toInt();
    }
    if (command.startsWith("set_mixspeed:")) {
      mixspeed = command.substring(13).toInt();
      if (mixspeed == 0) {
        motor.stop();
      } else {
        motor.setSpeed(mixspeed);
        motor.forward();
      }
    }
    if (command.startsWith("fan_int:")) {
      fanControlInterval = command.substring(8).toInt();
    }
    if (command.startsWith("temp_int:")) {
      tempInterval = command.substring(9).toInt();
      newHeaterPID.SetSampleTime(tempInterval);
    }
    if (command.startsWith("ds_int:")) {
      dataSendInterval = command.substring(7).toInt();
    }
    if (command.startsWith("tar_sensor:")) {
      tar_sensor = command.substring(11).toInt();
    }
    if (command.startsWith("setid:")) {
      exp_ID = command.substring(6).toInt();
      curExpStartTime = millis();
    }
    if (command.startsWith("write_SD:")) {
      write_SD = command.substring(9).toInt();
      if (write_SD == 1) {
        digitalWrite(THERMO_CS, HIGH);
        digitalWrite(HSPI_CS, LOW);
        digitalWrite(K_CS, HIGH);
        if (!SD.begin(HSPI_CS, hspi)) {
          SerialBT.println("SD Card MOUNT FAIL");
          write_SD = 3;
        }
        if (!SD.exists("/templog.txt")) {
          File file = SD.open("/templog.txt", FILE_WRITE);
          String payload_header = "{";
          payload_header += "\"temperature\":0,";
          payload_header += "\"onetemp\":0,";
          payload_header += "\"twotemp\":0,";
          payload_header += "\"tempdiff\":0,";
          payload_header += "\"dutyPercent\":0,";
          payload_header += "\"manualMode\":0,";
          payload_header += "\"pwmValue\":0,";
          payload_header += "\"thresholdLow\":0,";
          payload_header += "\"thresholdHigh\":0,";
          payload_header += "\"fanPwmMin\":0,";
          payload_header += "\"fanPwmMax\":0,";
          payload_header += "\"tempInterval\":0,";
          payload_header += "\"fanInterval\":0,";
          payload_header += "\"dataSendInterval\":0,";
          payload_header += "\"mins\":" + String(curMins) + ",";
          payload_header += "\"elapsedMins\":" + String(curElapsedTime) + ",";
          payload_header += "\"expID\":" + String(exp_ID) + ",";
          payload_header += "\"ishead\":1,";
          payload_header += "\"writeon\":" + String(write_SD) + ",";
          payload_header += "\"tarSensor\":" + String(tar_sensor) + ",";
          payload_header += "\"thermotemp\":0,";
          payload_header += "\"kTemp\":0,";
          payload_header += "\"heaterState\":" + String(heaterState) + ",";
          payload_header += "\"heaterThresholdLow\":0,";
          payload_header += "\"heaterThresholdHigh\":0,";
          payload_header += "\"heaterSensor\":" + String(heaterSensor) + ",";
          payload_header += "\"newHeaterState\":" + String(newHeaterState) + ",";
          payload_header += "\"newHeaterPwmValue\":" + String(newHeaterPwmValue) + ",";
          payload_header += "\"newHeaterThresholdLow\":0,";
          payload_header += "\"newHeaterThresholdHigh\":0,";
          payload_header += "\"newHeaterSensor\":" + String(newHeaterSensor) + ",";
          payload_header += "\"pidKp\":" + String(Kp) + ",";
          payload_header += "\"pidKi\":" + String(Ki) + ",";
          payload_header += "\"pidKd\":" + String(Kd) + ",";
          payload_header += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
          payload_header += "\"heaterManualMode\":" + String(heaterManualMode) + ",";
          payload_header += "\"newHeaterManualMode\":" + String(newHeaterManualMode) + ",";
          payload_header += "\"maxPwmLimit\":" + String(MAX_PWM_LIMIT);
          payload_header += "}";
          file.println(payload_header);
          file.close();
        }
        digitalWrite(HSPI_CS, HIGH);
      }
    }
    if (command.startsWith("clear_SD:")) {
      clear_SD = command.substring(9).toInt();
      if (clear_SD == 1) {
        digitalWrite(THERMO_CS, HIGH);
        digitalWrite(HSPI_CS, LOW);
        digitalWrite(K_CS, HIGH);
        SD.remove("/templog.txt");
        File file = SD.open("/templog.txt", FILE_WRITE);
        String payload_header = "{";
        payload_header += "\"temperature\":0,";
        payload_header += "\"onetemp\":0,";
        payload_header += "\"twotemp\":0,";
        payload_header += "\"tempdiff\":0,";
        payload_header += "\"dutyPercent\":0,";
        payload_header += "\"manualMode\":0,";
        payload_header += "\"pwmValue\":0,";
        payload_header += "\"thresholdLow\":0,";
        payload_header += "\"thresholdHigh\":0,";
        payload_header += "\"fanPwmMin\":0,";
        payload_header += "\"fanPwmMax\":0,";
        payload_header += "\"tempInterval\":0,";
        payload_header += "\"fanInterval\":0,";
        payload_header += "\"dataSendInterval\":0,";
        payload_header += "\"mins\":" + String(curMins) + ",";
        payload_header += "\"elapsedMins\":" + String(curElapsedTime) + ",";
        payload_header += "\"expID\":" + String(exp_ID) + ",";
        payload_header += "\"ishead\":1,";
        payload_header += "\"writeon\":" + String(write_SD) + ",";
        payload_header += "\"tarSensor\":" + String(tar_sensor) + ",";
        payload_header += "\"thermotemp\":0,";
        payload_header += "\"kTemp\":0,";
        payload_header += "\"heaterState\":" + String(heaterState) + ",";
        payload_header += "\"heaterThresholdLow\":0,";
        payload_header += "\"heaterThresholdHigh\":0,";
        payload_header += "\"heaterSensor\":" + String(heaterSensor) + ",";
        payload_header += "\"newHeaterState\":" + String(newHeaterState) + ",";
        payload_header += "\"newHeaterPwmValue\":" + String(newHeaterPwmValue) + ",";
        payload_header += "\"newHeaterThresholdLow\":0,";
        payload_header += "\"newHeaterThresholdHigh\":0,";
        payload_header += "\"newHeaterSensor\":" + String(newHeaterSensor) + ",";
        payload_header += "\"pidKp\":" + String(Kp) + ",";
        payload_header += "\"pidKi\":" + String(Ki) + ",";
        payload_header += "\"pidKd\":" + String(Kd) + ",";
        payload_header += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
        payload_header += "\"heaterManualMode\":" + String(heaterManualMode) + ",";
        payload_header += "\"newHeaterManualMode\":" + String(newHeaterManualMode) + ",";
        payload_header += "\"maxPwmLimit\":" + String(MAX_PWM_LIMIT);
        payload_header += "}";
        file.println(payload_header);
        file.close();
        clear_SD = 0;
        digitalWrite(HSPI_CS, HIGH);
      }
    }
  }
}
