#include <DS18B20.h>

DS18B20 ds(4);

uint8_t sensor1Address[8] = {0x28, 0x43, 0xB8, 0xBB, 0x00, 0x00, 0x00, 0x46};
uint8_t sensor2Address[8] = {0x28, 0x72, 0x69, 0x35, 0x00, 0x00, 0x00, 0x00};
float temp1 = 0;
float temp2 = 0;
float diff = 0;

void setup() {
  Serial.begin(9600);
  Serial.print("Devices: ");
  Serial.println(ds.getNumberOfDevices());
}

void loop() {
  ds.select(sensor1Address);
  Serial.println("First sensor:");
  temp1=ds.getTempC();
  Serial.print(temp1);
  ds.select(sensor2Address);
  temp2=ds.getTempC();
  Serial.println("Second sensor:");
  Serial.print(temp2);
  Serial.println("Diff:");
  diff=temp1-temp2;
  Serial.println(diff);
//  while (ds.selectNext()) {
//    switch (ds.getFamilyCode()) {
//      case MODEL_DS18S20:
//        Serial.println("Model: DS18S20/DS1820");
//        break;
//      case MODEL_DS1822:
//        Serial.println("Model: DS1822");
//        break;
//      case MODEL_DS18B20:
//        Serial.println("Model: DS18B20");
//        break;
//      default:
//        Serial.println("Unrecognized Device");
//        break;
//    }
//
//    uint8_t address[8];
//    ds.getAddress(address);
//
//    Serial.print("Address:");
//    for (uint8_t i = 0; i < 8; i++) {
//      Serial.print(" ");
//      Serial.print(address[i]);
//    }
//    Serial.println();
//
//    Serial.print("Resolution: ");
//    Serial.println(ds.getResolution());
//
//    Serial.print("Power Mode: ");
//    if (ds.getPowerMode()) {
//      Serial.println("External");
//    } else {
//      Serial.println("Parasite");
//    }
//
//    Serial.print("Temperature: ");
//    Serial.print(ds.getTempC());
//    Serial.print(" C / ");
//    Serial.print(ds.getTempF());
//    Serial.println(" F");
//    Serial.println();
//  }

  delay(1000);
}
