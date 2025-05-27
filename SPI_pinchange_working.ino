#include <SPI.h>
#include "SD.h"

// Define SPIClass objects for HSPI and VSPI
SPIClass hspi(HSPI); // HSPI is SPI2
SPIClass vspi(VSPI); // VSPI is SPI3

// Define HSPI pins (default or custom)
//#define HSPI_MISO 7
//#define HSPI_MOSI 8
//#define HSPI_SCK  14
//#define HSPI_CS   5

#define HSPI_MISO 19 //withou SD - ok config, after soldering SD - ok
#define HSPI_MOSI 23
#define HSPI_SCK  14
#define HSPI_CS   17

//#define HSPI_MISO 7 //with only 7 tied to SD - problem
//#define HSPI_MOSI 21
//#define HSPI_SCK  13
//#define HSPI_CS   5


//#define HSPI_MISO 7
//#define HSPI_MOSI 8
//#define HSPI_SCK  14
//#define HSPI_CS   17
// Define VSPI pins (default or custom, for reference)
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCK  18
#define VSPI_CS   5

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(100);
  while (!Serial) {
    ; // Wait for Serial to initialize
  }
  // Set HSPI pin modes explicitly
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
  // Optionally, initialize VSPI (if transitioning from VSPI to HSPI)
  // vspi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, VSPI_CS);

  // Print HSPI pin configuration
  Serial.println("HSPI Pin Configuration:");
  Serial.print("MISO: GPIO "); Serial.println(HSPI_MISO);
  Serial.print("MOSI: GPIO "); Serial.println(HSPI_MOSI);
  Serial.print("SCK: GPIO ");  Serial.println(HSPI_SCK);
  Serial.print("CS: GPIO ");   Serial.println(HSPI_CS);

  // Optional: If using VSPI, you can print its pins too
  /*
  Serial.println("\nVSPI Pin Configuration:");
  Serial.print("MISO: GPIO "); Serial.println(VSPI_MISO);
  Serial.print("MOSI: GPIO "); Serial.println(VSPI_MOSI);
  Serial.print("SCK: GPIO ");  Serial.println(VSPI_SCK);
  Serial.print("CS: GPIO ");   Serial.println(VSPI_CS);
  */
}

void loop() {
// Print HSPI pin configuration
  Serial.println("HSPI Pin Configuration:");
  Serial.print("MISO: GPIO "); Serial.println(HSPI_MISO);
  Serial.print("MOSI: GPIO "); Serial.println(HSPI_MOSI);
  Serial.print("SCK: GPIO ");  Serial.println(HSPI_SCK);
  Serial.print("CS: GPIO ");   Serial.println(HSPI_CS);
  Serial.println("---------------------");
  if (!SD.begin(HSPI_CS)) 

  {

    Serial.println("SD Card MOUNT FAIL");

  } 
  uint32_t cardSize = SD.cardSize() / (1024 * 1024);
  String str = "SDCard Size: " + String(cardSize) + "MB";
  Serial.println(str);
  // Delay to make output readable (adjust as needed)
  delay(2000); // Print every 2 seconds
}
