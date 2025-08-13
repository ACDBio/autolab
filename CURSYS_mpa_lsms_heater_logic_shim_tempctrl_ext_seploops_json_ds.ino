#include <DS18B20.h>
#include <BluetoothSerial.h>  // Библиотека для Bluetooth
#include <SPI.h>
#include "SD.h"
#include <time.h>
#include <Adafruit_MAX31865.h>
#include <L298N.h>
// CHANGE: Added PID library for new heater control
#include <PID_v1_bc.h>

// --- Add to variable declarations (near fanPin declaration) ---
const int msfanPin = 16; // Pin for magnetic stirrer fan
int msfanPwmValue = 30; // PWM value for msfan (0-255)

// CHANGE: Added pin and PWM configuration for new heater (HW-532)
#define NEW_HEATER_PIN 22
const int NEW_HEATER_PWM_CHANNEL = 1;  // PWM channel for new heater
const int NEW_HEATER_PWM_FREQ = 25000;  // 25 kHz to avoid noise
const int NEW_HEATER_PWM_RESOLUTION = 8;  // 8-bit resolution

// CHANGE: Added pin for MAX6675
#define K_CS 13  // CS pin for MAX6675

// Пин для твердотельного реле
const int heaterPin = 12;

// Пороговые значения температуры для нагревателя
float heaterThresholdLow = 20.0;  // Температура включения нагревателя
float heaterThresholdHigh = 22.0; // Температура выключения нагревателя
int heaterState = 0;              // Состояние реле (0 - выкл, 1 - вкл)
int heaterManualMode = 0;         // 0 - автоматический режим, 1 - ручной
int manualHeaterState = 0;        // Состояние нагревателя в ручном режиме
int heaterSensor=3;
float heatertemp = 0;

// CHANGE: Added variables for new heater (HW-532)
int newHeaterState = 0;           // State (0 - off, 1 - on)
int newHeaterManualMode = 0;      // 0 - auto, 1 - manual
double newHeaterPwmValue = 0;     // PWM value (0-255, double for PID)
float newHeaterThresholdLow = 20.0;  // Lower temperature threshold
float newHeaterThresholdHigh = 30.0; // Upper temperature threshold
int newHeaterSensor = 4;          // Sensor for new heater (4 - MAX6675)
float newHeaterTemp = 0;          // Current temperature for new heater
float rampRate = 0;               // Ramp rate (°C/min)
float rampMaxTemp = 0;            // Target max temperature for ramp
unsigned long lastRampTime = 0;   // Last ramp update time
double rampTargetTemp = 0;        // Current ramp target temperature
double newHeaterTargetTemp = 0;   // Target temperature for PID

// CHANGE: Added PID variables and object
double pidInput = 0;    // PID input (temperature)
double pidOutput = 0;   // PID output (PWM)
double pidSetpoint = 0; // PID setpoint (target temperature)
float Kp = 10.0;        // Proportional gain
float Ki = 0.1;         // Integral gain
float Kd = 1.0;         // Derivative gain
PID newHeaterPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// CHANGE: Added variable for MAX6675 temperature
float kTemp = 0;  // Temperature from MAX6675

unsigned long startMillis;

// Define SPIClass objects for HSPI and VSPI
SPIClass hspi(HSPI); // HSPI is SPI2
#define HSPI_MISO 19 //withou SD - ok config, after soldering SD - ok
#define HSPI_MOSI 23
#define HSPI_SCK  14
#define HSPI_CS   17
#define THERMO_CS   21

// Pin definition for L298N
const unsigned int IN1 = 27;
const unsigned int IN2 = 26;
const unsigned int EN = 25;

// Create one motor instance
L298N motor(EN, IN1, IN2);

DS18B20 ds(4);  // пин

BluetoothSerial SerialBT;

const int fanPin = 18;
int dutyPercent = 20;   // начальная скважность для вентиляторного режима
int pwmValue = 20;       // начальная скважность

float temperatureThreshold_low = 25.0;  // Порог температуры низкий
float temperatureThreshold_high = 40.0;  // Порог температуры высокий
int fan_pwm_min = 20;  // минимальная скорость
int fan_pwm_max = 100;  // максимальная скорость
int manualMode = 0;  // Флаг ручного режима
int exp_ID = 0;

int mixspeed = 0;

unsigned long tempInterval = 500;  // интервал для обновления температуры (500 мс)
unsigned long fanControlInterval = 100;  // интервал для контроля вентилятора (100 мс)
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

uint8_t sensor1Address[8] = {0x28, 0x43, 0xB8, 0xBB, 0x00, 0x00, 0x00, 0x46}; //DS18B20 addressess (find with discover_onewire.h sketch)
uint8_t sensor2Address[8] = {0x28, 0x72, 0x69, 0x35, 0x00, 0x00, 0x00, 0xC3};

int num_dev=0;

int tar_sensor=1; //1 or 2 or 3 (thermocouple)

int compar_sensor=3;

float currentTemp=0;

int write_SD=0;
int clear_SD=0;

Adafruit_MAX31865 thermo = Adafruit_MAX31865(THERMO_CS, &hspi); // HSPI_CS, HSPI_MOSI, HSPI_MISO, HSPI_SCK);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

//// CHANGE: Added function to read MAX6675 temperature
//float readKTypeTemperature() {
//  digitalWrite(K_CS, LOW);
//  delayMicroseconds(10);
//  uint16_t data = 0;
//  data = hspi.transfer(0x00) << 8;
//  data |= hspi.transfer(0x00);
//  digitalWrite(K_CS, HIGH);
//  
//  if (data & 0x0004) {
//    return -1;  // Thermocouple error
//  }
//  
//  data >>= 3;
//  return data * 0.25;  // Temperature in °C
//}



float readKTypeTemperature() {
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
    digitalWrite(msfanPin, LOW); // Ensure pin is low initially
    ledcSetup(0, 25000, 8); // Setup PWM channel 1, 21kHz frequency, 8-bit resolution
    ledcAttachPin(msfanPin, 0); // Attach msfanPin to PWM channel 1
    ledcWrite(0, msfanPwmValue); // Set initial PWM value (0 = off)

    // CHANGE: Initialize new heater pin and PWM
    pinMode(NEW_HEATER_PIN, OUTPUT);
    ledcSetup(NEW_HEATER_PWM_CHANNEL, NEW_HEATER_PWM_FREQ, NEW_HEATER_PWM_RESOLUTION);
    ledcAttachPin(NEW_HEATER_PIN, NEW_HEATER_PWM_CHANNEL);
    ledcWrite(NEW_HEATER_PWM_CHANNEL, 0);

//    // CHANGE: Initialize MAX6675 CS pin
//    pinMode(K_CS, OUTPUT);
//    digitalWrite(K_CS, HIGH);

    // CHANGE: Initialize PID
    newHeaterPID.SetMode(AUTOMATIC);
    newHeaterPID.SetOutputLimits(0, 255);
    newHeaterPID.SetSampleTime(500);  // Sync with tempInterval

    SerialBT.setPin("1515");
    pinMode(fanPin, OUTPUT);
    Serial.begin(115200);
    SerialBT.begin("ESP32_Fan_Control_N");  // Инициализация Bluetooth с именем устройства
    num_dev=ds.getNumberOfDevices();  // первый запрос на измерение
    pinMode(HSPI_SCK, OUTPUT);
    pinMode(HSPI_MOSI, OUTPUT);
    pinMode(HSPI_MISO, INPUT);
    pinMode(HSPI_CS, OUTPUT);
    pinMode(THERMO_CS, OUTPUT);
    pinMode(K_CS, OUTPUT);
    digitalWrite(HSPI_CS, HIGH); // Ensure CS is high
    digitalWrite(THERMO_CS, HIGH); // Ensure CS is high
    digitalWrite(K_CS, HIGH); // Ensure CS is high

    // Initialize HSPI with custom pins
    hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, -1);
    if (!SD.begin(HSPI_CS, hspi)) 
    {
      Serial.println("SD Card MOUNT FAIL");
    } 
    if (!thermo.begin(MAX31865_2WIRE)) 
    {
        Serial.println("Device MOUNT FAIL");
        while (1);
    } 
    
    startMillis = millis();
    curExpStartTime=millis();
    motor.setSpeed(0);
    pinMode(heaterPin, OUTPUT);
    digitalWrite(heaterPin, LOW); // Изначально реле выключено
}

void loop() {
    unsigned long currentMillis = millis();
    curMins=currentMillis/60000;
    curElapsedTime=(currentMillis-curExpStartTime)/60000;

    // Чтение температуры каждые 500 мс
    if (currentMillis - lastTempTime >= tempInterval) {
        lastTempTime = currentMillis;
        digitalWrite(HSPI_CS, HIGH);  // отключить SD
        digitalWrite(THERMO_CS, LOW); // включить термопару

        thermotemp=thermo.temperature(RNOMINAL, RREF);

        digitalWrite(THERMO_CS, HIGH); // отключить термопару
        
        // CHANGE: Read MAX6675 temperature
        digitalWrite(K_CS, LOW);
        kTemp = readKTypeTemperature();
        Serial.println(kTemp);
        digitalWrite(K_CS, HIGH);

        if (1==1) {         // измерения готовы по таймеру
            if (1==1) {  // если чтение успешно
                ds.select(sensor1Address);
                if (num_dev==2){
                    currentTemp1=ds.getTempC();
                    ds.select(sensor2Address);
                    currentTemp2=ds.getTempC();
                }

                if (tar_sensor==1){
                    currentTemp=currentTemp1;
                } 
                if (tar_sensor==2) {
                    currentTemp=currentTemp2;
                } 
                if (tar_sensor==3) {
                    currentTemp=thermotemp;
                }
                // CHANGE: Added sensor option for MAX6675
                if (tar_sensor==4) {
                    currentTemp=kTemp;
                }

                TempDiff=currentTemp-currentTemp1;

                if (heaterSensor==1){
                    heatertemp=currentTemp1;
                } 
                if (heaterSensor==2) {
                    heatertemp=currentTemp2;
                } 
                if (heaterSensor==3) {
                    heatertemp=thermotemp;
                }
                // CHANGE: Added sensor option for MAX6675
                if (heaterSensor==4) {
                    heatertemp=kTemp;
                }

                // CHANGE: Select temperature for new heater
                if (newHeaterSensor==1) {
                    newHeaterTemp=currentTemp1;
                } 
                if (newHeaterSensor==2) {
                    newHeaterTemp=currentTemp2;
                } 
                if (newHeaterSensor==3) {
                    newHeaterTemp=thermotemp;
                }
                if (newHeaterSensor==4) {
                    newHeaterTemp=kTemp;
                }

                // Управление нагревателем (термостат)
                if (heaterManualMode == 0) { // Автоматический режим
                    if (heatertemp < heaterThresholdLow && heaterState == 0) {
                        digitalWrite(heaterPin, HIGH); // Включить нагреватель
                        heaterState = 1;
                    } else if (heatertemp > heaterThresholdHigh && heaterState == 1) {
                        digitalWrite(heaterPin, LOW);  // Выключить нагреватель
                        heaterState = 0;
                    }
                } else { // Ручной режим
                    digitalWrite(heaterPin, manualHeaterState);
                    heaterState = manualHeaterState;
                }

                // CHANGE: Added control for new heater (PID)
                if (newHeaterState == 1) {
                    if (newHeaterManualMode == 0) {
                        if (rampRate > 0 && rampMaxTemp > 0) {
                            // Ramp mode
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
                        newHeaterPID.Compute();
                        newHeaterPwmValue = pidOutput;
                    }
                    ledcWrite(NEW_HEATER_PWM_CHANNEL, newHeaterPwmValue);
                } else {
                    newHeaterPwmValue = 0;
                    ledcWrite(NEW_HEATER_PWM_CHANNEL, 0);
                }

                // Управление вентилятором в автоматическом режиме
                if (manualMode==0) {
                    if (currentTemp > temperatureThreshold_low) {
                        pwmValue = map(currentTemp, temperatureThreshold_low, temperatureThreshold_high, fan_pwm_min, fan_pwm_max);  // Плавное увеличение
                    } else {
                        pwmValue = 0;  // Вентилятор выключен, если температура ниже порога
                    }
                }
 
                // SEARCH/REPLACE: Modified SD payload to include kTemp and new heater parameters
                /*
                SEARCH:
                if (write_SD==1){
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
                  payload += "\"heaterState\":" + String(heaterState) + ",";
                  payload += "\"heaterThresholdLow\":" + String(heaterThresholdLow) + ",";
                  payload += "\"heaterThresholdHigh\":" + String(heaterThresholdHigh) + ",";
                  payload += "\"heaterSensor\":" + String(heaterSensor) + ",";
                  payload += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
                  payload += "\"heaterManualMode\":" + String(heaterManualMode);
                  payload += "}";        
                  digitalWrite(THERMO_CS, HIGH); // отключить термопару
                  digitalWrite(HSPI_CS, LOW);    // включить SD
                  File file = SD.open("/templog.txt", FILE_APPEND);
                  if (file) {
                    file.println(payload);
                    file.close();
                  } else {
                    Serial.println("Failed to append to /templog.txt");
                  }
                  digitalWrite(HSPI_CS, HIGH);   // отключить SD
                }

                REPLACE:
                */
                if (write_SD==1){
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
                  // CHANGE: Added kTemp and new heater parameters
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
                  payload += "\"newHeaterManualMode\":" + String(newHeaterManualMode);
                  payload += "}";        
                  digitalWrite(THERMO_CS, HIGH); // отключить термопару
                  digitalWrite(HSPI_CS, LOW);    // включить SD
                  // CHANGE: Ensure MAX6675 CS is high
                  digitalWrite(K_CS, HIGH);
                  File file = SD.open("/templog.txt", FILE_APPEND);
                  if (file) {
                    file.println(payload);
                    file.close();
                  } else {
                    Serial.println("Failed to append to /templog.txt");
                  }
                  digitalWrite(HSPI_CS, HIGH);   // отключить SD
                }
            } else {
                Serial.println("Error reading temperature");
            }
        }
    }

    // SEARCH/REPLACE: Modified Bluetooth payload to include kTemp and new heater parameters
    /*
    SEARCH:
    if (currentMillis - lastDataSendTime >= dataSendInterval) {
                lastDataSendTime = currentMillis;
                String tempStr = String(currentTemp);
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
                payload += "\"thermotemp\":" + String(thermotemp)+ ",";  
                payload += "\"heaterState\":" + String(heaterState) + ",";
                payload += "\"heaterThresholdLow\":" + String(heaterThresholdLow) + ",";
                payload += "\"heaterThresholdHigh\":" + String(heaterThresholdHigh) + ",";
                payload += "\"heaterSensor\":" + String(heaterSensor) + ",";
                payload += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
                payload += "\"heaterManualMode\":" + String(heaterManualMode);
                payload += "}";
                SerialBT.println(payload);
    }

    REPLACE:
    */
    if (currentMillis - lastDataSendTime >= dataSendInterval) {
                lastDataSendTime = currentMillis;
                String tempStr = String(currentTemp);
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
                // CHANGE: Added kTemp and new heater parameters
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
                payload += "\"newHeaterManualMode\":" + String(newHeaterManualMode);
                payload += "}";
                SerialBT.println(payload);
    }

    // Управление вентилятором каждые 100 мс
    if (currentMillis - lastFanControlTime >= fanControlInterval) {
        lastFanControlTime = currentMillis;

        if (manualMode==1) {
            pwmValue = dutyPercent;  // в ручном режиме pwmValue равен заданной скорости
        }

        unsigned long onTime = fanControlInterval * pwmValue / 100;  // время включения
        unsigned long offTime = fanControlInterval - onTime;         // время выключения

        if (pwmValue > 0) {
            digitalWrite(fanPin, HIGH);
            delay(onTime);
        }
        if (pwmValue < 100) {
            digitalWrite(fanPin, LOW);
            delay(offTime);
        }
    }

    // SEARCH/REPLACE: Modified Bluetooth command handling to include new heater controls
    /*
    SEARCH:
    if (SerialBT.available()) {
        String command = SerialBT.readString();
        if (command.startsWith("msfan_pwm:")) {
            msfanPwmValue = command.substring(10).toInt();
            msfanPwmValue = constrain(msfanPwmValue, 0, 255); // Ensure value is within 0-255
            ledcWrite(0, msfanPwmValue); // Update PWM
        }
        if (command.startsWith("heater_low:")) {
            heaterThresholdLow = command.substring(11).toFloat();  // Установить нижний порог
        }
        if (command.startsWith("heater_high:")) {
            heaterThresholdHigh = command.substring(12).toFloat(); // Установить верхний порог
        }
        if (command.startsWith("heater_manual:")) {
            heaterManualMode = command.substring(14).toInt(); // Включить/выключить ручной режим
        }
        if (command.startsWith("heater_state:") && heaterManualMode) {
            manualHeaterState = command.substring(13).toInt(); // Установить состояние в ручном режиме
        }
        if (command.startsWith("heater_sensor:")) {
            heaterSensor = command.substring(14).toInt(); // Установить состояние в ручном режиме
        }
        if (command.startsWith("threshold_low:")) {
            temperatureThreshold_low = command.substring(14).toFloat();  // Установка нового низкого порога
        }
        if (command.startsWith("threshold_high:")) {
            temperatureThreshold_high = command.substring(15).toFloat();  // Установка нового высокого порога
        }
        if (command.startsWith("manual:")) {
            manualMode = command.substring(7).toInt();  // Переключение в ручной режим
        }
        if (command.startsWith("speed:") && manualMode) {
            dutyPercent = command.substring(6).toInt();  // Установка скорости вручную
        }
        if (command.startsWith("fan_pwm_min:")) {
            fan_pwm_min = command.substring(12).toInt();  // Установка минимальной скорости
        }
        if (command.startsWith("fan_pwm_max:")) {
            fan_pwm_max = command.substring(12).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("compar_sensor:")) {
            compar_sensor = command.substring(14).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("set_mixspeed:")) {
            mixspeed = command.substring(13).toInt();  // Установка максимальной скорости
            if (mixspeed == 0){
              motor.stop();
            } else {
                motor.setSpeed(mixspeed);
                motor.forward();
            }
        }
        if (command.startsWith("fan_int:")) {
            fanControlInterval = command.substring(8).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("temp_int:")) {
            tempInterval = command.substring(9).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("ds_int:")) {
            dataSendInterval = command.substring(7).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("tar_sensor:")) { //1 or 2 or 3 (the sensor to control the fan) 3 is the thermocouple
            tar_sensor = command.substring(11).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("setid:")) { //1 or 2 (the sensor to control the fan)
            exp_ID = command.substring(6).toInt();  // Установка максимальной скорости
            curExpStartTime=millis();
        }
        if (command.startsWith("write_SD:")) {
            write_SD = command.substring(9).toInt();  
            if (write_SD==1){
              digitalWrite(THERMO_CS, HIGH); // отключить термопару
              digitalWrite(HSPI_CS, LOW);    // включить SD
              if (!SD.begin(HSPI_CS, hspi)) 
              {
                  SerialBT.println("SD Card MOUNT FAIL");
                  write_SD=3;
              }
              if (!SD.exists("/templog.txt")) {
                File file = SD.open("/templog.txt", FILE_WRITE);
                String payload_header = "{";
                  payload_header += "\"temperature\":" + String(0) + ",";
                  payload_header += "\"onetemp\":" + String(0) + ",";
                  payload_header += "\"twotemp\":" + String(0) + ",";
                  payload_header += "\"tempdiff\":" + String(0) + ",";
                  payload_header += "\"dutyPercent\":" + String(0) + ",";
                  payload_header += "\"manualMode\":" + String(0) + ",";
                  payload_header += "\"pwmValue\":" + String(0) + ",";
                  payload_header += "\"thresholdLow\":" + String(0) + ",";
                  payload_header += "\"thresholdHigh\":" + String(0) + ",";
                  payload_header += "\"fanPwmMin\":" + String(0) + ",";
                  payload_header += "\"fanPwmMax\":" + String(0) + ",";
                  payload_header += "\"tempInterval\":" + String(0) + ",";
                  payload_header += "\"fanInterval\":" + String(0) + ",";
                  payload_header += "\"dataSendInterval\":" + String(0) + ",";
                  payload_header += "\"mins\":" + String(curMins) + ",";
                  payload_header += "\"elapsedMins\":" + String(curElapsedTime) + ",";
                  payload_header += "\"expID\":" + String(exp_ID) + ",";
                  payload_header += "\"ishead\":" + String(1) + ",";
                  payload_header += "\"writeon\":" + String(write_SD) + ",";
                  payload_header += "\"tarSensor\":" + String(tar_sensor)+ ",";
                  payload_header += "\"thermotemp\":" + String(0)+ ",";  
                  payload_header += "\"heaterState\":" + String(heaterState) + ",";
                  payload_header += "\"heaterThresholdLow\":" + String(0) + ",";
                  payload_header += "\"heaterThresholdHigh\":" + String(0) + ",";
                  payload_header += "\"heaterSensor\":" + String(heaterSensor) + ",";
                  payload_header += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
                  payload_header += "\"heaterManualMode\":" + String(heaterManualMode);
                  payload_header += "}";
                file.println(payload_header);
                file.close();
              }
              digitalWrite(HSPI_CS, HIGH);   // отключить SD
            }
        }
        if (command.startsWith("clear_SD:")){
          clear_SD = command.substring(9).toInt();
          if (clear_SD==1){
            digitalWrite(THERMO_CS, HIGH); // отключить термопару
            digitalWrite(HSPI_CS, LOW);    // включить SD
            SD.remove("/templog.txt");
            File file = SD.open("/templog.txt", FILE_WRITE);
            String payload_header = "{";
                payload_header += "\"temperature\":" + String(0) + ",";
                payload_header += "\"onetemp\":" + String(0) + ",";
                payload_header += "\"twotemp\":" + String(0) + ",";
                payload_header += "\"tempdiff\":" + String(0) + ",";
                payload_header += "\"dutyPercent\":" + String(0) + ",";
                payload_header += "\"manualMode\":" + String(0) + ",";
                payload_header += "\"pwmValue\":" + String(0) + ",";
                payload_header += "\"thresholdLow\":" + String(0) + ",";
                payload_header += "\"thresholdHigh\":" + String(0) + ",";
                payload_header += "\"fanPwmMin\":" + String(0) + ",";
                payload_header += "\"fanPwmMax\":" + String(0) + ",";
                payload_header += "\"tempInterval\":" + String(0) + ",";
                payload_header += "\"fanInterval\":" + String(0) + ",";
                payload_header += "\"dataSendInterval\":" + String(0) + ",";
                payload_header += "\"mins\":" + String(curMins) + ",";
                payload_header += "\"elapsedMins\":" + String(curElapsedTime) + ",";
                payload_header += "\"expID\":" + String(exp_ID) + ",";
                payload_header += "\"ishead\":" + String(1) + ",";
                payload_header += "\"writeon\":" + String(write_SD) + ",";
                payload_header += "\"tarSensor\":" + String(tar_sensor)+ ",";
                payload_header += "\"thermotemp\":" + String(0)+ ",";  
                payload_header += "\"heaterState\":" + String(heaterState) + ",";
                payload_header += "\"heaterThresholdLow\":" + String(0) + ",";
                payload_header += "\"heaterThresholdHigh\":" + String(0) + ",";
                payload_header += "\"heaterSensor\":" + String(heaterSensor) + ",";
                payload_header += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
                payload_header += "\"heaterManualMode\":" + String(heaterManualMode);
                payload_header += "}";
            file.println(payload_header);
            file.close();
            clear_SD=0;
            digitalWrite(HSPI_CS, HIGH);   // отключить SD
          }
        }
    }

    REPLACE:
    */
    if (SerialBT.available()) {
        String command = SerialBT.readString();
        if (command.startsWith("msfan_pwm:")) {
            msfanPwmValue = command.substring(10).toInt();
            msfanPwmValue = constrain(msfanPwmValue, 0, 255); // Ensure value is within 0-255
            ledcWrite(0, msfanPwmValue); // Update PWM
        }
        if (command.startsWith("heater_low:")) {
            heaterThresholdLow = command.substring(11).toFloat();  // Установить нижний порог
        }
        if (command.startsWith("heater_high:")) {
            heaterThresholdHigh = command.substring(12).toFloat(); // Установить верхний порог
        }
        if (command.startsWith("heater_manual:")) {
            heaterManualMode = command.substring(14).toInt(); // Включить/выключить ручной режим
        }
        if (command.startsWith("heater_state:") && heaterManualMode) {
            manualHeaterState = command.substring(13).toInt(); // Установить состояние в ручном режиме
        }
        if (command.startsWith("heater_sensor:")) {
            heaterSensor = command.substring(14).toInt(); // Установить состояние в ручном режиме
        }
        // CHANGE: Added commands for new heater
        if (command.startsWith("new_heater_state:")) {
            newHeaterState = command.substring(17).toInt();
        }
        if (command.startsWith("new_heater_manual:")) {
            newHeaterManualMode = command.substring(18).toInt();
            if (newHeaterManualMode == 0) newHeaterPwmValue = 0; // Reset PWM in auto mode
        }
        if (command.startsWith("new_heater_pwm:") && newHeaterManualMode) {
            newHeaterPwmValue = map(command.substring(15).toInt(), 0, 100, 0, 255);
            newHeaterPwmValue = constrain(newHeaterPwmValue, 0, 255);
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
                rampTargetTemp = newHeaterTemp; // Start from current temperature
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
        if (command.startsWith("threshold_low:")) {
            temperatureThreshold_low = command.substring(14).toFloat();  // Установка нового низкого порога
        }
        if (command.startsWith("threshold_high:")) {
            temperatureThreshold_high = command.substring(15).toFloat();  // Установка нового высокого порога
        }
        if (command.startsWith("manual:")) {
            manualMode = command.substring(7).toInt();  // Переключение в ручной режим
        }
        if (command.startsWith("speed:") && manualMode) {
            dutyPercent = command.substring(6).toInt();  // Установка скорости вручную
        }
        if (command.startsWith("fan_pwm_min:")) {
            fan_pwm_min = command.substring(12).toInt();  // Установка минимальной скорости
        }
        if (command.startsWith("fan_pwm_max:")) {
            fan_pwm_max = command.substring(12).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("compar_sensor:")) {
            compar_sensor = command.substring(14).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("set_mixspeed:")) {
            mixspeed = command.substring(13).toInt();  // Установка максимальной скорости
            if (mixspeed == 0){
              motor.stop();
            } else {
                motor.setSpeed(mixspeed);
                motor.forward();
            }
        }
        if (command.startsWith("fan_int:")) {
            fanControlInterval = command.substring(8).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("temp_int:")) {
            tempInterval = command.substring(9).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("ds_int:")) {
            dataSendInterval = command.substring(7).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("tar_sensor:")) { //1 or 2 or 3 (the sensor to control the fan) 3 is the thermocouple
            tar_sensor = command.substring(11).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("setid:")) { //1 or 2 (the sensor to control the fan)
            exp_ID = command.substring(6).toInt();  // Установка максимальной скорости
            curExpStartTime=millis();
        }
        // SEARCH/REPLACE: Modified write_SD to include kTemp and new heater parameters
        /*
        SEARCH:
        if (command.startsWith("write_SD:")) {
            write_SD = command.substring(9).toInt();  
            if (write_SD==1){
              digitalWrite(THERMO_CS, HIGH); // отключить термопару
              digitalWrite(HSPI_CS, LOW);    // включить SD
              if (!SD.begin(HSPI_CS, hspi)) 
              {
                  SerialBT.println("SD Card MOUNT FAIL");
                  write_SD=3;
              }
              if (!SD.exists("/templog.txt")) {
                File file = SD.open("/templog.txt", FILE_WRITE);
                String payload_header = "{";
                  payload_header += "\"temperature\":" + String(0) + ",";
                  payload_header += "\"onetemp\":" + String(0) + ",";
                  payload_header += "\"twotemp\":" + String(0) + ",";
                  payload_header += "\"tempdiff\":" + String(0) + ",";
                  payload_header += "\"dutyPercent\":" + String(0) + ",";
                  payload_header += "\"manualMode\":" + String(0) + ",";
                  payload_header += "\"pwmValue\":" + String(0) + ",";
                  payload_header += "\"thresholdLow\":" + String(0) + ",";
                  payload_header += "\"thresholdHigh\":" + String(0) + ",";
                  payload_header += "\"fanPwmMin\":" + String(0) + ",";
                  payload_header += "\"fanPwmMax\":" + String(0) + ",";
                  payload_header += "\"tempInterval\":" + String(0) + ",";
                  payload_header += "\"fanInterval\":" + String(0) + ",";
                  payload_header += "\"dataSendInterval\":" + String(0) + ",";
                  payload_header += "\"mins\":" + String(curMins) + ",";
                  payload_header += "\"elapsedMins\":" + String(curElapsedTime) + ",";
                  payload_header += "\"expID\":" + String(exp_ID) + ",";
                  payload_header += "\"ishead\":" + String(1) + ",";
                  payload_header += "\"writeon\":" + String(write_SD) + ",";
                  payload_header += "\"tarSensor\":" + String(tar_sensor)+ ",";
                  payload_header += "\"thermotemp\":" + String(0)+ ",";  
                  payload_header += "\"heaterState\":" + String(heaterState) + ",";
                  payload_header += "\"heaterThresholdLow\":" + String(0) + ",";
                  payload_header += "\"heaterThresholdHigh\":" + String(0) + ",";
                  payload_header += "\"heaterSensor\":" + String(heaterSensor) + ",";
                  payload_header += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
                  payload_header += "\"heaterManualMode\":" + String(heaterManualMode);
                  payload_header += "}";
                file.println(payload_header);
                file.close();
              }
              digitalWrite(HSPI_CS, HIGH);   // отключить SD
            }
        }

        REPLACE:
        */
        if (command.startsWith("write_SD:")) {
            write_SD = command.substring(9).toInt();  
            if (write_SD==1){
              digitalWrite(THERMO_CS, HIGH); // отключить термопару
              digitalWrite(HSPI_CS, LOW);    // включить SD
              // CHANGE: Ensure MAX6675 CS is high
              digitalWrite(K_CS, HIGH);
              if (!SD.begin(HSPI_CS, hspi)) 
              {
                  SerialBT.println("SD Card MOUNT FAIL");
                  write_SD=3;
              }
              if (!SD.exists("/templog.txt")) {
                File file = SD.open("/templog.txt", FILE_WRITE);
                String payload_header = "{";
                  payload_header += "\"temperature\":" + String(0) + ",";
                  payload_header += "\"onetemp\":" + String(0) + ",";
                  payload_header += "\"twotemp\":" + String(0) + ",";
                  payload_header += "\"tempdiff\":" + String(0) + ",";
                  payload_header += "\"dutyPercent\":" + String(0) + ",";
                  payload_header += "\"manualMode\":" + String(0) + ",";
                  payload_header += "\"pwmValue\":" + String(0) + ",";
                  payload_header += "\"thresholdLow\":" + String(0) + ",";
                  payload_header += "\"thresholdHigh\":" + String(0) + ",";
                  payload_header += "\"fanPwmMin\":" + String(0) + ",";
                  payload_header += "\"fanPwmMax\":" + String(0) + ",";
                  payload_header += "\"tempInterval\":" + String(0) + ",";
                  payload_header += "\"fanInterval\":" + String(0) + ",";
                  payload_header += "\"dataSendInterval\":" + String(0) + ",";
                  payload_header += "\"mins\":" + String(curMins) + ",";
                  payload_header += "\"elapsedMins\":" + String(curElapsedTime) + ",";
                  payload_header += "\"expID\":" + String(exp_ID) + ",";
                  payload_header += "\"ishead\":" + String(1) + ",";
                  payload_header += "\"writeon\":" + String(write_SD) + ",";
                  payload_header += "\"tarSensor\":" + String(tar_sensor) + ",";
                  payload_header += "\"thermotemp\":" + String(0) + ",";
                  // CHANGE: Added kTemp and new heater parameters
                  payload_header += "\"kTemp\":" + String(0) + ",";
                  payload_header += "\"heaterState\":" + String(heaterState) + ",";
                  payload_header += "\"heaterThresholdLow\":" + String(0) + ",";
                  payload_header += "\"heaterThresholdHigh\":" + String(0) + ",";
                  payload_header += "\"heaterSensor\":" + String(heaterSensor) + ",";
                  payload_header += "\"newHeaterState\":" + String(newHeaterState) + ",";
                  payload_header += "\"newHeaterPwmValue\":" + String(newHeaterPwmValue) + ",";
                  payload_header += "\"newHeaterThresholdLow\":" + String(0) + ",";
                  payload_header += "\"newHeaterThresholdHigh\":" + String(0) + ",";
                  payload_header += "\"newHeaterSensor\":" + String(newHeaterSensor) + ",";
                  payload_header += "\"pidKp\":" + String(Kp) + ",";
                  payload_header += "\"pidKi\":" + String(Ki) + ",";
                  payload_header += "\"pidKd\":" + String(Kd) + ",";
                  payload_header += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
                  payload_header += "\"heaterManualMode\":" + String(heaterManualMode) + ",";
                  payload_header += "\"newHeaterManualMode\":" + String(newHeaterManualMode);
                  payload_header += "}";
                file.println(payload_header);
                file.close();
              }
              digitalWrite(HSPI_CS, HIGH);   // отключить SD
            }
        }
        // SEARCH/REPLACE: Modified clear_SD to include kTemp and new heater parameters
        /*
        SEARCH:
        if (command.startsWith("clear_SD:")){
          clear_SD = command.substring(9).toInt();
          if (clear_SD==1){
            digitalWrite(THERMO_CS, HIGH); // отключить термопару
            digitalWrite(HSPI_CS, LOW);    // включить SD
            SD.remove("/templog.txt");
            File file = SD.open("/templog.txt", FILE_WRITE);
            String payload_header = "{";
                payload_header += "\"temperature\":" + String(0) + ",";
                payload_header += "\"onetemp\":" + String(0) + ",";
                payload_header += "\"twotemp\":" + String(0) + ",";
                payload_header += "\"tempdiff\":" + String(0) + ",";
                payload_header += "\"dutyPercent\":" + String(0) + ",";
                payload_header += "\"manualMode\":" + String(0) + ",";
                payload_header += "\"pwmValue\":" + String(0) + ",";
                payload_header += "\"thresholdLow\":" + String(0) + ",";
                payload_header += "\"thresholdHigh\":" + String(0) + ",";
                payload_header += "\"fanPwmMin\":" + String(0) + ",";
                payload_header += "\"fanPwmMax\":" + String(0) + ",";
                payload_header += "\"tempInterval\":" + String(0) + ",";
                payload_header += "\"fanInterval\":" + String(0) + ",";
                payload_header += "\"dataSendInterval\":" + String(0) + ",";
                payload_header += "\"mins\":" + String(curMins) + ",";
                payload_header += "\"elapsedMins\":" + String(curElapsedTime) + ",";
                payload_header += "\"expID\":" + String(exp_ID) + ",";
                payload_header += "\"ishead\":" + String(1) + ",";
                payload_header += "\"writeon\":" + String(write_SD) + ",";
                payload_header += "\"tarSensor\":" + String(tar_sensor)+ ",";
                payload_header += "\"thermotemp\":" + String(0)+ ",";  
                payload_header += "\"heaterState\":" + String(heaterState) + ",";
                payload_header += "\"heaterThresholdLow\":" + String(0) + ",";
                payload_header += "\"heaterThresholdHigh\":" + String(0) + ",";
                payload_header += "\"heaterSensor\":" + String(heaterSensor) + ",";
                payload_header += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
                payload_header += "\"heaterManualMode\":" + String(heaterManualMode);
                payload_header += "}";
            file.println(payload_header);
            file.close();
            clear_SD=0;
            digitalWrite(HSPI_CS, HIGH);   // отключить SD
          }
        }

        REPLACE:
        */
        if (command.startsWith("clear_SD:")){
          clear_SD = command.substring(9).toInt();
          if (clear_SD==1){
            digitalWrite(THERMO_CS, HIGH); // отключить термопару
            digitalWrite(HSPI_CS, LOW);    // включить SD
            // CHANGE: Ensure MAX6675 CS is high
            digitalWrite(K_CS, HIGH);
            SD.remove("/templog.txt");
            File file = SD.open("/templog.txt", FILE_WRITE);
            String payload_header = "{";
                payload_header += "\"temperature\":" + String(0) + ",";
                payload_header += "\"onetemp\":" + String(0) + ",";
                payload_header += "\"twotemp\":" + String(0) + ",";
                payload_header += "\"tempdiff\":" + String(0) + ",";
                payload_header += "\"dutyPercent\":" + String(0) + ",";
                payload_header += "\"manualMode\":" + String(0) + ",";
                payload_header += "\"pwmValue\":" + String(0) + ",";
                payload_header += "\"thresholdLow\":" + String(0) + ",";
                payload_header += "\"thresholdHigh\":" + String(0) + ",";
                payload_header += "\"fanPwmMin\":" + String(0) + ",";
                payload_header += "\"fanPwmMax\":" + String(0) + ",";
                payload_header += "\"tempInterval\":" + String(0) + ",";
                payload_header += "\"fanInterval\":" + String(0) + ",";
                payload_header += "\"dataSendInterval\":" + String(0) + ",";
                payload_header += "\"mins\":" + String(curMins) + ",";
                payload_header += "\"elapsedMins\":" + String(curElapsedTime) + ",";
                payload_header += "\"expID\":" + String(exp_ID) + ",";
                payload_header += "\"ishead\":" + String(1) + ",";
                payload_header += "\"writeon\":" + String(write_SD) + ",";
                payload_header += "\"tarSensor\":" + String(tar_sensor) + ",";
                payload_header += "\"thermotemp\":" + String(0) + ",";
                // CHANGE: Added kTemp and new heater parameters
                payload_header += "\"kTemp\":" + String(0) + ",";
                payload_header += "\"heaterState\":" + String(heaterState) + ",";
                payload_header += "\"heaterThresholdLow\":" + String(0) + ",";
                payload_header += "\"heaterThresholdHigh\":" + String(0) + ",";
                payload_header += "\"heaterSensor\":" + String(heaterSensor) + ",";
                payload_header += "\"newHeaterState\":" + String(newHeaterState) + ",";
                payload_header += "\"newHeaterPwmValue\":" + String(newHeaterPwmValue) + ",";
                payload_header += "\"newHeaterThresholdLow\":" + String(0) + ",";
                payload_header += "\"newHeaterThresholdHigh\":" + String(0) + ",";
                payload_header += "\"newHeaterSensor\":" + String(newHeaterSensor) + ",";
                payload_header += "\"pidKp\":" + String(Kp) + ",";
                payload_header += "\"pidKi\":" + String(Ki) + ",";
                payload_header += "\"pidKd\":" + String(Kd) + ",";
                payload_header += "\"msfanPwmValue\":" + String(msfanPwmValue) + ",";
                payload_header += "\"heaterManualMode\":" + String(heaterManualMode) + ",";
                payload_header += "\"newHeaterManualMode\":" + String(newHeaterManualMode);
                payload_header += "}";
            file.println(payload_header);
            file.close();
            clear_SD=0;
            digitalWrite(HSPI_CS, HIGH);   // отключить SD
          }
        }
    }
}
