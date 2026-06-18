#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_QMC5883P.h>
#include <DFRobot_SIM7000.h>
#include <SdFat.h>

#define TFT_CS 32
#define TFT_DC 33
#define TFT_RST 25
#define MAG_SDA 16
#define MAG_SCL 22
#define SIM_TX 4
#define SIM_RX 2
#define SD_CS 5

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Adafruit_QMC5883P mag = Adafruit_QMC5883P();
DFRobot_SIM7000 sim7000(&Serial2);
SdFat SD;

SemaphoreHandle_t dataMutex;
SemaphoreHandle_t simMutex;
SemaphoreHandle_t spiMutex;

int posX = 10, posY = 10;
uint32_t loopTimer = 0, screenTimer = 0, transmisionTimer = 0;
bool APNStatus = false, connected = false;
byte mode = 0;
String latitude, longitude, currentTime;
float azimuth;
int gpsErrorCounter = 0;

int ok = 0, error = 0;

const int MAX_GPS_ERRORS = 6;
const char* SERVER = "http://frog03.mikr.us:21491/api/gps";
const char* DEVICE_ID = "tracker01";
const char* OFFLINE_FILE = "offlineData.txt";
const char* NAVI_FILE = "naviData.txt";
// const char* SERVER = "http://frog03.mikr.us:21491/api/device/telemetry";
// const char* MODE_URL = "http://frog03.mikr.us:21491/api/device/mode";
// const char* WAYPOINTS_URL = "http://frog03.mikr.us:21491/api/device/pending-waypoints";

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
void saveToSD(String json);
void processOfflineDataBatch();

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);
  Wire.begin(MAG_SDA, MAG_SCL);

  dataMutex = xSemaphoreCreateMutex();
  simMutex = xSemaphoreCreateMutex();
  spiMutex = xSemaphoreCreateMutex();

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

  xSemaphoreTake(spiMutex, portMAX_DELAY);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  xSemaphoreGive(spiMutex);

  xSemaphoreTake(spiMutex, portMAX_DELAY);
  if (!SD.begin(SD_CS, SD_SCK_MHZ(4))) {
    Serial.println("SD Card Mount Failed!");
  } else {
    Serial.println("SD Card Initialized.");
  }
  xSemaphoreGive(spiMutex);

  initializeSIM7000();
  initializeSIMCard();
  initializeNetwork();
  configureAPN();
  initializeGPS();

  delay(1000);
  posY = 10;
  waitForGPS();
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  tft.fillScreen(ILI9341_BLACK);
  xSemaphoreGive(spiMutex);

  // =========================================================================
  // --- NOWY FRAGMENT: SPRAWDZANIE TRYBU I POBIERANIE WSPÓŁRZĘDNYCH ---
  // =========================================================================
  int setupX = 10, setupY = 10;

  printLine("Waiting for server...", setupX, setupY, ILI9341_WHITE, 2);
  
  String modeResponse = "";
  
  if (xSemaphoreTake(simMutex, portMAX_DELAY) == pdTRUE) {
    while(Serial2.available()) Serial2.read();
    
    Serial2.print("AT+HTTPPARA=\"URL\",\"");
    Serial2.print(MODE_URL);
    Serial2.println("\"");
    delay(200);
    
    Serial2.println("AT+HTTPACTION=0");
    
    unsigned long start = millis();
    bool actionOk = false;
    while(millis() - start < 6000) {
      if(Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        if(line.indexOf("+HTTPACTION: 0,200") != -1) { // Kod 200 OK
          actionOk = true;
          break;
        }
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    if(actionOk) {
      Serial2.println("AT+HTTPREAD");
      delay(500);
      while(Serial2.available()) {
        modeResponse += (char)Serial2.read();
      }
    }
    xSemaphoreGive(simMutex);
  }

  // Analiza odpowiedzi serwera dotyczącej trybu
  if (modeResponse.indexOf("\"mode\":\"planning\"") != -1) {
    mode = 1; // Ustawienie zmiennej na tryb nawigacji
    replaceLine("MODE: NAVIGATION", setupX, setupY, ILI9341_YELLOW, 2);
    printLine("Downloading waypoints...", setupX, setupY, ILI9341_WHITE, 2);
    
    String wpResponse = "";
    
    // Wejście w stan oczekiwania i pobieranie punktów trasy
    if (xSemaphoreTake(simMutex, portMAX_DELAY) == pdTRUE) {
      while(Serial2.available()) Serial2.read();
      
      Serial2.print("AT+HTTPPARA=\"URL\",\"");
      Serial2.print(WAYPOINTS_URL);
      Serial2.println("\"");
      delay(200);
      
      Serial2.println("AT+HTTPACTION=0");
      
      unsigned long start = millis();
      bool actionOk = false;
      while(millis() - start < 8000) { // Dłuższy timeout na wypadek wielu punktów
        if(Serial2.available()) {
          String line = Serial2.readStringUntil('\n');
          if(line.indexOf("+HTTPACTION: 0,200") != -1) {
            actionOk = true;
            break;
          }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
      
      if(actionOk) {
        Serial2.println("AT+HTTPREAD");
        delay(1500); // Czas na odebranie całego pakietu danych
        while(Serial2.available()) {
          wpResponse += (char)Serial2.read();
        }
      }
      xSemaphoreGive(simMutex);
    }
    
    // Parsowanie punktów oraz zapis na kartę SD (naviData.txt)
    int pointsIdx = wpResponse.indexOf("\"points\":[");
    if (pointsIdx != -1) {
      pointsIdx += 10;
      
      xSemaphoreTake(spiMutex, portMAX_DELAY);
      SD.remove(NAVI_FILE); // Usunięcie starych punktów nawigacyjnych
      FsFile naviFile = SD.open(NAVI_FILE, O_RDWR | O_CREAT | O_APPEND);
      
      if (naviFile) {
        int savedCount = 0;
        while (true) {
          int latIndex = wpResponse.indexOf("\"lat\":", pointsIdx);
          if (latIndex == -1) break;
          int latEnd = wpResponse.indexOf(",", latIndex);
          String latVal = wpResponse.substring(latIndex + 6, latEnd);
          
          int lngIndex = wpResponse.indexOf("\"lng\":", latEnd);
          if (lngIndex == -1) break;
          int lngEnd = wpResponse.indexOf("}", lngIndex);
          String lngVal = wpResponse.substring(lngIndex + 6, lngEnd);
          
          // Zapis koordynatów w formacie "szerokość,długość" linia po linii (w kolejności z mapy)
          naviFile.println(latVal + "," + lngVal);
          savedCount++;
          
          pointsIdx = lngEnd;
        }
        naviFile.close();
        printLine("Saved " + String(savedCount) + " points!", setupX, setupY, ILI9341_GREEN, 2);
      } else {
        printLine("SD Write Error!", setupX, setupY, ILI9341_RED, 2);
      }
      xSemaphoreGive(spiMutex);
    } else {
      printLine("No active route found.", setupX, setupY, ILI9341_WHITE, 2);
    }
    delay(2000);
    
  } else {
    mode = 0; // Ustawienie zmiennej na tryb domyślny (tracker)
    replaceLine("MODE: TRACKER", setupX, setupY, ILI9341_YELLOW, 2);
    delay(1000);
  }
  
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  tft.fillScreen(ILI9341_BLACK);
  xSemaphoreGive(spiMutex);
  // =========================================================================

  xTaskCreatePinnedToCore(
    transmission,
    "Task_HTTP",
    16384,
    NULL,
    1,
    NULL,
    0
  );
}

void loop() {
  switch (mode) {
    case 0:
      if (wait(loopTimer, 500)) {
        bool gotPosition = false;
        String localLat = "", localLon = "";

        if (xSemaphoreTake(simMutex, portMAX_DELAY) == pdTRUE) {
          gotPosition = sim7000.getPosition();
          if (gotPosition) {
            localLat = sim7000.getLatitude();
            localLon = sim7000.getLongitude();
          }
          xSemaphoreGive(simMutex);
        }

        if (!gotPosition) {
          gpsErrorCounter++;
          
          if (gpsErrorCounter >= MAX_GPS_ERRORS) {
            waitForGPS();
            gpsErrorCounter = 0;
          }
        } else {
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          latitude = localLat;
          longitude = localLon;
          xSemaphoreGive(dataMutex);

          if (mag.isDataReady()) {
            int16_t x, y, z;
            mag.getRawMagnetic(&x, &y, &z);

            float localAzimuth = atan2(y, x) * 180 / PI;
            if (localAzimuth < 0) {
              localAzimuth += 360;
            }

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            azimuth = localAzimuth;
            xSemaphoreGive(dataMutex);
          }
        }
      }

      if (wait(screenTimer, 500)) {
        updateScreen();
      }
      
      break;

    case 1:
      
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
  if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {  
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

    xSemaphoreGive(spiMutex);
  }
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
    bool gotPos = false;

    if (simMutex != NULL) {
      if (xSemaphoreTake(simMutex, portMAX_DELAY) == pdTRUE) {
        gotPos = sim7000.getPosition();
        xSemaphoreGive(simMutex);
      }
    } else {
      gotPos = sim7000.getPosition();
    }

    if (gotPos) {
      printLine("GPS fixed!", posX, posY, ILI9341_GREEN, 2);
      delay(1000);
      return true;
    }
    delay(300);
  }
  return false;
}

String getTime() {
  if (simMutex != NULL) {
    if (xSemaphoreTake(simMutex, portMAX_DELAY) != pdTRUE) return "";
  }

  Serial2.println("AT+CGNSINF");
  unsigned long start = millis();
  String response = "";

  while (millis() - start < 1000) {
    if (Serial2.available()) {
      response += (char)Serial2.read();
    }
  }

  if (simMutex != NULL) {
    xSemaphoreGive(simMutex);
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
    vTaskDelay(7000 / portTICK_PERIOD_MS);

    String localTime = getTime();

    String currentLat, currentLon;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    currentLat = latitude;
    currentLon = longitude;
    currentTime = localTime;
    xSemaphoreGive(dataMutex);

    String json = "{";
    json += "\"device_id\":\"" + String(DEVICE_ID) + "\"";
    json += ",\"timestamp\":\"" + localTime + "\"";
    json += ",\"lat\":" + currentLat;
    json += ",\"lon\":" + currentLon;
    json += "}";

    Serial.println(json);

    bool success = false;
    if (xSemaphoreTake(simMutex, portMAX_DELAY) == pdTRUE) {
      success = sim7000.httpPost(SERVER, json);
      xSemaphoreGive(simMutex);
    }

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    if (success) {
      connected = true;
      ok++;
    } else {
      connected = false;
      error++;
    }
    xSemaphoreGive(dataMutex);

    // Proces karty SD w zależności od stanu połączenia
    if (success) {
      processOfflineDataBatch();
    } else {
      saveToSD(json);
    }
  }
}

void saveToSD(String json) {
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  FsFile file = SD.open(OFFLINE_FILE, O_RDWR | O_CREAT | O_APPEND);
  if (file) {
    file.println(json);
    file.close();
    Serial.println("Data appended to SD");
  } else {
    Serial.println("Failed to open SD for writing");
  }
  xSemaphoreGive(spiMutex);
}

void processOfflineDataBatch() {
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  if (!SD.exists(OFFLINE_FILE)) {
    xSemaphoreGive(spiMutex);
    return;
  }
  FsFile file = SD.open(OFFLINE_FILE, O_READ);
  FsFile newFile = SD.open("temp.txt", O_RDWR | O_CREAT | O_TRUNC);
  xSemaphoreGive(spiMutex);

  if (!file || !newFile) {
    if (file) { xSemaphoreTake(spiMutex, portMAX_DELAY); file.close(); xSemaphoreGive(spiMutex); }
    if (newFile) { xSemaphoreTake(spiMutex, portMAX_DELAY); newFile.close(); xSemaphoreGive(spiMutex); }
    return;
  }

  Serial.println("Processing offline buffer in batches...");
  const int BATCH_SIZE = 10;
  bool batchFailed = false;

  while (true) {
    String linesCache[BATCH_SIZE];
    int linesRead = 0;

    xSemaphoreTake(spiMutex, portMAX_DELAY);
    while (file.available() && linesRead < BATCH_SIZE) {
      String jsonLine = file.readStringUntil('\n');
      jsonLine.trim();
      if (jsonLine.length() > 0) {
        linesCache[linesRead++] = jsonLine;
      }
    }
    bool hasMore = file.available();
    xSemaphoreGive(spiMutex);

    if (linesRead == 0) break;

    // Jeżeli nie było awarii wysyłania - spróbuj wysłać batchem
    if (!batchFailed) {
      String batchJson = "[";
      for (int i = 0; i < linesRead; i++) {
        if (i > 0) batchJson += ",";
        batchJson += linesCache[i];
      }
      batchJson += "]";

      bool success = false;
      if (xSemaphoreTake(simMutex, portMAX_DELAY) == pdTRUE) {
        success = sim7000.httpPost(SERVER, batchJson);
        xSemaphoreGive(simMutex);
      }

      if (!success) {
        batchFailed = true;
        Serial.println("Batch send FAILED!");
      } else {
        Serial.println("Batch sent successfully!");
      }
    }

    // Przepisywanie do nowego pliku (jeżeli batchFailed w tym obrocie pętli lub od poprzedniego)
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    if (batchFailed) {
      for (int i = 0; i < linesRead; i++) {
        newFile.println(linesCache[i]);
      }
    }
    xSemaphoreGive(spiMutex);

    if (!hasMore) break;
  }

  // Zamykanie i zmiana nazw plików pod kontrolą Mutexa SPI
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  file.close();
  newFile.close();

  SD.remove(OFFLINE_FILE);
  FsFile checkTemp = SD.open("temp.txt", O_READ);
  if (checkTemp && checkTemp.size() > 0) {
    checkTemp.close();
    SD.rename("temp.txt", OFFLINE_FILE);
  } else {
    if (checkTemp) checkTemp.close();
    SD.remove("temp.txt");
  }
  xSemaphoreGive(spiMutex);
}