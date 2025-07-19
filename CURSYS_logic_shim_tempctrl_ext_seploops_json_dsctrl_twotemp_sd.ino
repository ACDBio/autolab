#include <DS18B20.h>
#include <BluetoothSerial.h>  // Библиотека для Bluetooth
#include <SPI.h>
#include "SD.h"
#include <time.h>
#include <Adafruit_MAX31865.h>
#include <L298N.h>

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
//unsigned long interval = 100;  // цикл ШИМ
//unsigned long lastTime = 0;
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



void setup() {
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
    digitalWrite(HSPI_CS, HIGH); // Ensure CS is high
    digitalWrite(THERMO_CS, HIGH); // Ensure CS is high

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
}



void loop() {
    unsigned long currentMillis = millis();
    //SerialBT.println(millis());
    curMins=currentMillis/60000;
    curElapsedTime=currentMillis-curExpStartTime;
    curElapsedTime=curElapsedTime/60000;
    // Чтение температуры каждые 500 мс
    if (currentMillis - lastTempTime >= tempInterval) {
        lastTempTime = currentMillis;
        digitalWrite(HSPI_CS, HIGH);  // отключить SD
        digitalWrite(THERMO_CS, LOW); // включить термопару

        thermotemp=thermo.temperature(RNOMINAL, RREF);

        digitalWrite(THERMO_CS, HIGH); // отключить термопару
        //Serial.print("RTD value: "); Serial.println(rtd);
//        SerialBT.println("Out");
//        SerialBT.println();
        
        if (1==1) {         // измерения готовы по таймеру
            if (1==1) {  // если чтение успешно
                ds.select(sensor1Address);
                if (num_dev==2){
                currentTemp1=ds.getTempC();
                //Serial.println(currentTemp1);
                ds.select(sensor2Address);
                currentTemp2=ds.getTempC();}
                //Serial.println(currentTemp2);
//                SerialBT.println("In");
//                SerialBT.println(String(compar_sensor));
//                SerialBT.println();
                
//                if (compar_sensor==3){
//                  //SerialBT.println("Setting ctemp2 to thermo");
//                  currentTemp2=thermotemp;
//                  }
//                 if (compar_sensor==1){
//                  currentTemp2=currentTemp1;
//                  }
                
                //TempDiff=currentTemp1-currentTemp2;
                //Serial.println(TempDiff);
                if (tar_sensor==1){
                  currentTemp=currentTemp1;
                  } 
                  
                 if (tar_sensor==2) {currentTemp=currentTemp2;
                  } 
                  
                 if (tar_sensor==3) {
                    //SerialBT.println("Setting ctemp to thermo");
                    currentTemp=thermotemp;}

                TempDiff=currentTemp-currentTemp1;
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
                  payload += "\"dataSendInterval\":" + String(dataSendInterval) + ",";
                  payload += "\"mins\":" + String(curMins) + ",";
                  payload += "\"elapsedMins\":" + String(curElapsedTime) + ",";
                  payload += "\"expID\":" + String(exp_ID) + ",";
                  payload += "\"ishead\":" + String(0) + ",";
                  payload += "\"writeon\":" + String(write_SD) + ",";
                  payload += "\"tarSensor\":" + String(tar_sensor) + ",";
                  payload += "\"thermotemp\":" + String(thermotemp);            
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
                payload += "\"dataSendInterval\":" + String(dataSendInterval) + ",";
                payload += "\"mins\":" + String(curMins) + ",";
                payload += "\"elapsedMins\":" + String(curElapsedTime) + ",";
                payload += "\"expID\":" + String(exp_ID) + ",";
                payload += "\"ishead\":" + String(0) + ",";
                payload += "\"writeon\":" + String(write_SD) + ",";
                payload += "\"tarSensor\":" + String(tar_sensor) + ",";
                payload += "\"thermotemp\":" + String(thermotemp);  
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
            //SerialBT.println(exp_ID);
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
                  payload_header += "\"tarSensor\":" + String(tar_sensor);
                  payload_header += "}";
                file.println(payload_header);
                file.close();
            
                SerialBT.println("SD write ON");

              }
             digitalWrite(HSPI_CS, HIGH);   // отключить SD
          } else {SerialBT.println("SD write OFF");}
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
                payload_header += "\"tarSensor\":" + String(tar_sensor);
                payload_header += "}";
            file.println(payload_header);
            file.close();
            SerialBT.println("SD cleared");
            clear_SD=0;
            digitalWrite(HSPI_CS, HIGH);   // отключить SD
            }
       }
    }
    //delay(10);
}
