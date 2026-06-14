#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_QMC5883P.h>
#include <DFRobot_SIM7000.h>

#define TFT_CS 32
#define TFT_DC 33
#define TFT_RST 25
#define MAG_SDA 16
#define MAG_SCL 22
#define SIM_TX 4
#define SIM_RX 2

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Adafruit_QMC5883P mag = Adafruit_QMC5883P();
DFRobot_SIM7000 sim7000(&Serial2);
SemaphoreHandle_t dataMutex;

int posX = 10, posY = 10;
uint32_t loopTimer = 0, screenTimer = 0, transmisionTimer = 0;
bool APNStatus = false, connected = false;
byte mode = 0;
String latitude, longitude, currentTime;
float azimuth;

int ok = 0, error = 0;

const char* SERVER = "http://frog03.mikr.us:21491/api/gps";
const char* DEVICE_ID = "tracker01";
const char* OFFLINE_FILE = "offline.txt";

bool wait(uint32_t &lastTime, uint32_t interval);
void printLine(const String text, int &posX, int &posY, uint16_t color = ILI9341_WHITE, int textSize = 1);
void replaceLine(const String text, int &posX, int &posY, uint16_t color = ILI9341_WHITE, int textSize = 1);
void updateScreen();
bool initializeSIM7000();
bool initializeSIMCard();
bool initializeNetwork();
bool configureAPN();
bool initializeGPS();
bool waitForGPS();
String getTime();
void transmission(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  Wire.begin(MAG_SDA, MAG_SCL);

  Serial.println("MCU starting...");

  if (!mag.begin()) {
    Serial.println("Magnetometer has not been found!");
    while (1) {
      delay(10);
    }
  } else {
    Serial.println("Magnetometer OK");
  }

  mag.setMode(QMC5883P_MODE_NORMAL);
  mag.setODR(QMC5883P_ODR_50HZ);

  tft.begin();
  tft.setRotation(1);

  tft.fillScreen(ILI9341_BLACK);

  initializeSIM7000();
  initializeSIMCard();
  initializeNetwork();
  configureAPN();
  initializeGPS();

  delay(1000);
  posY = 10;
  waitForGPS();
  tft.fillScreen(ILI9341_BLACK);

  // dataMutex = xSemaphoreCreateMutex();
  // xTaskCreatePinnedToCore(
  //   transmission,
  //   "Task_HTTP",
  //   16384,
  //   NULL,
  //   1,
  //   NULL,
  //   0
  // );
}

void loop() {
  switch (mode) {
    case 0:
      if (wait(loopTimer, 500)) {
        if (!sim7000.getPosition()) {
          waitForGPS();
        } else {
          latitude = sim7000.getLatitude();
          longitude = sim7000.getLongitude();
          currentTime = getTime();

          if (mag.isDataReady()) {
            int16_t x, y, z;
            mag.getRawMagnetic(&x, &y, &z);

            azimuth = atan2(y, x) * 180 / PI;
            if (azimuth < 0) {
              azimuth += 360;
            }
          }
        }
      }

      if (wait(screenTimer, 500)) {
        updateScreen();
      }
      
      if (wait(transmisionTimer, 5000)) {
        currentTime = getTime();

        String json = "{";
        json += "\"device_id\":\"" + String(DEVICE_ID) + "\"";
        json += ",\"timestamp\":\"" + currentTime + "\"";
        json += ",\"lat\":" + latitude;
        json += ",\"lon\":" + longitude;
        json += "}";

        Serial.println(json);

        // if (sim7000.httpPost(SERVER, json)) {
        //   connected = true;
        //   ok++;
        //   //process SD
        // } else {
        //   connected = false;
        //   error++;
        //   // save to SD
        // }
      }
      break;
  }
}

bool wait(uint32_t &lastTime, uint32_t interval) {
  uint32_t currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;
    return true;
  }
  return false;
}

void printLine(const String text, int &posX, int &posY, uint16_t color, int textSize) {
  tft.setCursor(posX, posY);
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  tft.fillRect(posX, posY, tft.width() - posX * 2, textSize * 10, ILI9341_BLACK);
  tft.println(text);
  posY += textSize * 10; // Add proper spacing: 8 pixels for char height + 2 for line spacing
}

void replaceLine(const String text, int &posX, int &posY, uint16_t color, int textSize) {
  tft.fillRect(posX, posY - textSize * 10, tft.width() - posX * 2, textSize * 10, ILI9341_BLACK); // Wyczyść poprzednią linię
  int originalPosY = posY - textSize * 10;
  printLine(text, posX, originalPosY, color, textSize);
}

void updateScreen() {
  posY = 10;

  if (mode == 0) {
    printLine("MODE: TRACKER", posX, posY, ILI9341_YELLOW, 2);
  } else if (mode == 1) {
    printLine("MODE: NAVIGATION", posX, posY, ILI9341_YELLOW, 2);
  }

  if (connected) {
    printLine("ONLINE", posX, posY, ILI9341_GREEN, 2);
  } else {
    printLine("OFFLINE", posX, posY, ILI9341_RED, 2);
  }
  printLine(latitude, posX, posY, ILI9341_WHITE, 2);
  printLine(longitude, posX, posY, ILI9341_WHITE, 2);
  printLine(String(azimuth), posX, posY, ILI9341_WHITE, 2);
  printLine("OK: " + String(ok) + " Error: " + String(error), posX, posY, ILI9341_WHITE, 2);
  posY = 222;
  printLine(currentTime, posX, posY, ILI9341_WHITE, 1);
}

bool initializeSIM7000() {
  printLine("Initializing SIM7000...", posX, posY, ILI9341_WHITE, 2);

  if (sim7000.isON()) {
    printLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  } else {
    printLine("ERROR", posX, posY, ILI9341_RED, 2);

    while (!sim7000.isON()) {
      delay(500);
    }

    replaceLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  }
}

bool initializeSIMCard() {
  printLine("Checking SIM card...", posX, posY, ILI9341_WHITE, 2);

  if (sim7000.checkSIMStatus()) {
    printLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  } else {
    printLine("ERROR", posX, posY, ILI9341_RED, 2);

    while (!sim7000.checkSIMStatus()) {
      delay(500);
    }

    replaceLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  }
}

bool initializeNetwork() {
  printLine("Setting network mode...", posX, posY, ILI9341_WHITE, 2);

  if (sim7000.setNetMode(sim7000.eGPRS)) {
    printLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  } else {
    printLine("ERROR", posX, posY, ILI9341_RED, 2);

    while (!sim7000.setNetMode(sim7000.eGPRS)) {
      delay(500);
    }

    replaceLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  }
}

bool configureAPN() {
  printLine("Configuring APN...", posX, posY, ILI9341_WHITE, 2);

  if (sim7000.httpInit(sim7000.eGPRS, APNStatus)) {
    printLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  } else {
    printLine("ERROR", posX, posY, ILI9341_RED, 2);

    while (!sim7000.httpInit(sim7000.eGPRS, APNStatus)) {
      delay(500);
    }

    replaceLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  }
}

bool initializeGPS() {
  printLine("Initializing GPS...", posX, posY, ILI9341_WHITE, 2);

  if (sim7000.initPos()) {
    printLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  } else {
    printLine("ERROR", posX, posY, ILI9341_RED, 2);

    while (!sim7000.initPos()) {
      delay(500);
    }

    replaceLine("OK", posX, posY, ILI9341_GREEN, 2);
    return true;
  }
}

bool waitForGPS() {
  posY = 10;
  tft.fillScreen(ILI9341_BLACK);
  printLine("Fixing GPS...", posX, posY, ILI9341_WHITE, 2);

  while (1) {
    if (sim7000.getPosition()) {
      printLine("GPS fixed!", posX, posY, ILI9341_GREEN, 2);
      delay(1000);
      return true;
    }
    delay(300);
  }
  return false;
}

String getTime() {
  Serial2.println("AT+CGNSINF");
  unsigned long start = millis();
  String response = "";

  while (millis() - start < 1000) {
    if (Serial2.available()) {
      response += (char)Serial2.read();
    }
  }

  int fixIdx = response.indexOf("+CGNSINF: 1,1");
  if (fixIdx != -1) {
    int firstComma = response.indexOf(',', fixIdx);
    int secondComma = response.indexOf(',', firstComma + 1);
    int thirdComma = response.indexOf(',', secondComma + 1);

    String raw = response.substring(secondComma + 1, thirdComma);
    
    if (raw.length() >= 14) {
      String y = raw.substring(0, 4);
      String m = raw.substring(4, 6);
      String d = raw.substring(6, 8);
      String hh = raw.substring(8, 10);
      String mm = raw.substring(10, 12);
      String ss = raw.substring(12, 14);

      return y + "-" + m + "-" + d + " " + hh + ":" + mm + ":" + ss;
    }
  }
  return "";
}

void transmission(void *pvParameters) {
  for (;;) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    String currentLat, currentLon, currTime;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    currentLat = latitude;
    currentLon = longitude;
    currTime = currentTime;
    xSemaphoreGive(dataMutex);

    String json = "{";
    json += "\"device_id\":\"" + String(DEVICE_ID) + "\"";
    json += ",\"timestamp\":\"" + currTime + "\"";
    json += ",\"lat\":" + currentLat;
    json += ",\"lon\":" + currentLon;
    json += "}";

    bool success = sim7000.httpPost(SERVER, json);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    if (success) {
      connected = true;
      ok++;
      // proces SD
    } else {
      connected = false;
      error++;
      // proces SD
    }
    xSemaphoreGive(dataMutex);
  }
}