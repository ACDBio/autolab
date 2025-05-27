#include <DS18B20.h>
#include <BluetoothSerial.h>  // Библиотека для Bluetooth
#include <SPI.h>
#include "SD.h"
#include <time.h>
unsigned long startMillis;

// Define SPIClass objects for HSPI and VSPI
SPIClass hspi(HSPI); // HSPI is SPI2
#define HSPI_MISO 19 //withou SD - ok config, after soldering SD - ok
#define HSPI_MOSI 23
#define HSPI_SCK  14
#define HSPI_CS   17

DS18B20 ds(4);  // пин

BluetoothSerial SerialBT;

const int fanPin = 18;
int dutyPercent = 20;   // начальная скважность для вентиляторного режима
//unsigned long interval = 100;  // цикл ШИМ
//unsigned long lastTime = 0;
int pwmValue = 20;       // начальная скважность

float temperatureThreshold_low = 25.0;  // Порог температуры низкий
float temperatureThreshold_high = 40.0;  // Порог температуры высокий
int fan_pwm_min = 20;  // минимальная скорость
int fan_pwm_max = 100;  // максимальная скорость
int manualMode = 0;  // Флаг ручного режима



unsigned long tempInterval = 500;  // интервал для обновления температуры (500 мс)
unsigned long fanControlInterval = 100;  // интервал для контроля вентилятора (100 мс)
unsigned long dataSendInterval = 1000; 

unsigned long lastTempTime = 0;
unsigned long lastFanControlTime = 0;
unsigned long lastDataSendTime = 0;

float currentTemp1 = 0;
float currentTemp2 = 0;
float TempDiff = 0;

uint8_t sensor1Address[8] = {0x28, 0x43, 0xB8, 0xBB, 0x00, 0x00, 0x00, 0x46}; //DS18B20 addressess (find with discover_onewire.h sketch)
uint8_t sensor2Address[8] = {0x28, 0x72, 0x69, 0x35, 0x00, 0x00, 0x00, 0xC3};

int num_dev=0;

int tar_sensor=1; //1 or 2

float currentTemp=0;

int write_SD=0;
int clear_SD=0;


void setup() {
    SerialBT.setPin("1515");
    pinMode(fanPin, OUTPUT);
    Serial.begin(115200);
    SerialBT.begin("ESP32_Fan_Control");  // Инициализация Bluetooth с именем устройства
    num_dev=ds.getNumberOfDevices();  // первый запрос на измерение
    pinMode(HSPI_SCK, OUTPUT);
    pinMode(HSPI_MOSI, OUTPUT);
    pinMode(HSPI_MISO, INPUT);
    pinMode(HSPI_CS, OUTPUT);
    digitalWrite(HSPI_CS, HIGH); // Ensure CS is high

    // Initialize HSPI with custom pins
    hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    if (!SD.begin(HSPI_CS, hspi)) 
    {
      Serial.println("SD Card MOUNT FAIL");
    } 
    startMillis = millis();
    
}



void loop() {
    unsigned long currentMillis = millis();

    // Чтение температуры каждые 500 мс
    if (currentMillis - lastTempTime >= tempInterval) {
        lastTempTime = currentMillis;
        
        if (num_dev==2) {         // измерения готовы по таймеру
            if (num_dev==2) {  // если чтение успешно
                ds.select(sensor1Address);
                currentTemp1=ds.getTempC();
                Serial.println(currentTemp1);
                ds.select(sensor2Address);
                currentTemp2=ds.getTempC();
                Serial.println(currentTemp2);
                TempDiff=currentTemp1-currentTemp2;
                Serial.println(TempDiff);
                if (tar_sensor==1){
                  currentTemp=currentTemp1;
                  } else {currentTemp=currentTemp2;
                  }

                
//                String tempStr = String(currentTemp);
//                SerialBT.print("Temperature: ");
//                SerialBT.println(tempStr);
//                
//                SerialBT.print("Duty Percent: ");
//                SerialBT.println(dutyPercent);
//                
//                SerialBT.print("Manual Mode: ");
//                SerialBT.println(manualMode);
//                
//                SerialBT.print("PWM Value: ");
//                SerialBT.println(pwmValue);
//                
//                SerialBT.print("Temperature Threshold Low: ");
//                SerialBT.println(temperatureThreshold_low);
//                
//                SerialBT.print("Temperature Threshold High: ");
//                SerialBT.println(temperatureThreshold_high);
//                
//                SerialBT.print("Fan PWM Min: ");
//                SerialBT.println(fan_pwm_min);
//                
//                SerialBT.print("Fan PWM Max: ");
//                SerialBT.println(fan_pwm_max);
//                
//                SerialBT.print("Temp Interval: ");
//                SerialBT.println(tempInterval);
//
//                SerialBT.print("Fan Control Interval: ");
//                SerialBT.println(fanControlInterval);

                // Управление вентилятором в автоматическом режиме
                if (manualMode==0) {
                    if (currentTemp > temperatureThreshold_low) {
                        pwmValue = map(currentTemp, temperatureThreshold_low, temperatureThreshold_high, fan_pwm_min, fan_pwm_max);  // Плавное увеличение
                    } else {
                        pwmValue = 0;  // Вентилятор выключен, если температура ниже порога
                    }
                }

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
                  payload += "\"dataSendInterval\":" + String(dataSendInterval);
                  payload += "\"millis\":" + String(currentMillis) + ",";
                  payload += "}";
                  
                  File file = SD.open("/templog.txt", FILE_APPEND);
                  if (file) {
                    file.println(payload);
                    file.close();
                  } else {
                    Serial.println("Failed to append to /templog.txt");
                  }
              }
            } else {
                Serial.println("Error reading temperature");
            }
//            ds.requestTemp();  // запрос следующего измерения
        }
    }
    if (currentMillis - lastDataSendTime >= dataSendInterval) {
                lastDataSendTime = currentMillis;
                // Отправка текущей температуры через Bluetooth
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
                payload += "\"dataSendInterval\":" + String(dataSendInterval);
                payload += "\"millis\":" + String(currentMillis) + ",";
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

    if (SerialBT.available()) {
        String command = SerialBT.readString();
       // String command = Serial.readStringUntil('\n');
       // command.trim();  // Убираем лишние пробелы
        if (command.startsWith("threshold_low:")) {
            temperatureThreshold_low = command.substring(14).toFloat();  // Установка нового низкого порога
        }
        if (command.startsWith("threshold_high:")) {
            temperatureThreshold_high = command.substring(15).toFloat();  // Установка нового высокого порога
        }
        if (command.startsWith("manual:")) {
            manualMode = command.substring(7).toInt();  // Переключение в ручной режим
//            Serial.print("Received manual command: ");
//            Serial.println(command);
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
        if (command.startsWith("fan_int:")) {
            fanControlInterval = command.substring(8).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("temp_int:")) {
            tempInterval = command.substring(9).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("ds_int:")) {
            dataSendInterval = command.substring(7).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("tar_sensor:")) { //1 or 2 (the sensor to control the fan)
            tar_sensor = command.substring(11).toInt();  // Установка максимальной скорости
        }
        if (command.startsWith("write_SD:")) {
            write_SD = command.substring(9).toInt();  
            if (write_SD==1){
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
                  payload_header += "\"millis\":" + String(millis()) + ",";
                  payload_header += "}";
                file.println(payload_header);
                file.close();
              }
          }
        }
        if (command.startsWith("clear_SD:")){
          clear_SD = command.substring(9).toInt();
          if (clear_SD==1){
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
                payload_header += "\"millis\":" + String(millis()) + ",";
                payload_header += "}";
            file.println(payload_header);
            file.close();
            clear_SD=0;
            }
          
       }
    }
}
