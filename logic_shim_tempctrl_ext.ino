#include <GyverDS18.h>
#include <BluetoothSerial.h>  // Библиотека для Bluetooth
GyverDS18Single ds(4);  // пин

BluetoothSerial SerialBT;

const int fanPin = 18;
int dutyPercent = 20;   // начальная скважность для вентиляторного режима
unsigned long interval = 100;  // цикл ШИМ
unsigned long lastTime = 0;
int pwmValue = 20;       // начальная скважность

float temperatureThreshold_low = 25.0;  // Порог температуры низкий
float temperatureThreshold_high = 40.0;  // Порог температуры высокий
int fan_pwm_min = 20;  // минимальная скорость
int fan_pwm_max = 100;  // максимальная скорость
int manualMode = 0;  // Флаг ручного режима

void setup() {
    pinMode(fanPin, OUTPUT);
    Serial.begin(115200);
    SerialBT.begin("ESP32_Fan_Control");  // Инициализация Bluetooth с именем устройства
    ds.requestTemp();  // первый запрос на измерение
}

void loop() {
    // Чтение температуры
    if (ds.ready()) {         // измерения готовы по таймеру
        if (ds.readTemp()) {  // если чтение успешно
            float currentTemp = ds.getTemp();
            Serial.print("temp: ");
            Serial.println(currentTemp);

            // Отправка текущей температуры через Bluetooth
            String tempStr = String(currentTemp);
            SerialBT.print("Temperature: ");
            SerialBT.println(tempStr);
            
            SerialBT.print("Duty Percent: ");
            SerialBT.println(dutyPercent);
            
            SerialBT.print("Manual Mode: ");
            SerialBT.println(manualMode);
            
            SerialBT.print("PWM Value: ");
            SerialBT.println(pwmValue);
            
            SerialBT.print("Temperature Threshold Low: ");
            SerialBT.println(temperatureThreshold_low);
            
            SerialBT.print("Temperature Threshold High: ");
            SerialBT.println(temperatureThreshold_high);
            
            SerialBT.print("Fan PWM Min: ");
            SerialBT.println(fan_pwm_min);
            
            SerialBT.print("Fan PWM Max: ");
            SerialBT.println(fan_pwm_max);
            
            SerialBT.print("Interval: ");
            SerialBT.println(interval);


            // Управление вентилятором в автоматическом режиме
            if (manualMode==0) {
                if (currentTemp > temperatureThreshold_low) {
                    // Чем выше температура, тем больше скорость (логический ШИМ)
                    pwmValue = map(currentTemp, temperatureThreshold_low, temperatureThreshold_high, fan_pwm_min, fan_pwm_max);  // Плавное увеличение
                } else {
                    pwmValue = 0;  // Вентилятор выключен, если температура ниже порога
                }

                unsigned long onTime = interval * pwmValue / 100;  // время включения
                unsigned long offTime = interval - onTime;         // время выключения

                unsigned long now = millis();
                if (now - lastTime >= interval) {
                    lastTime = now;
                    if (pwmValue > 0) {
                        digitalWrite(fanPin, HIGH);
                        delay(onTime);
                    }
                    if (pwmValue < 100) {
                        digitalWrite(fanPin, LOW);
                        delay(offTime);
                    }
                }
            } else {
                // Ручной режим: вентиляция по заданной скорости
                pwmValue = dutyPercent;  // в ручном режиме pwmValue равен заданной скорости

                unsigned long onTime = interval * pwmValue / 100;  // время включения
                unsigned long offTime = interval - onTime;         // время выключения

                unsigned long now = millis();
                if (now - lastTime >= interval) {
                    lastTime = now;
                    if (pwmValue > 0) {
                        digitalWrite(fanPin, HIGH);
                        delay(onTime);
                    }
                    if (pwmValue < 100) {
                        digitalWrite(fanPin, LOW);
                        delay(offTime);
                    }
                }
            }
        } else {
            Serial.println("Error reading temperature");
        }

        ds.requestTemp();  // запрос следующего измерения
    }

    // Получение данных от Bluetooth
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
        if (command.startsWith("interval:")) {
            interval = command.substring(9).toInt();  // Установка нового интервала для ШИМ
        }
        if (command.startsWith("fan_pwm_min:")) {
            fan_pwm_min = command.substring(12).toInt();  // Установка минимальной скорости
        }
        if (command.startsWith("fan_pwm_max:")) {
            fan_pwm_max = command.substring(12).toInt();  // Установка максимальной скорости
        }
    }
}
