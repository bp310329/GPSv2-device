#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_QMC5883P.h>
#include <DFRobot_SIM7000.h>
#include <SdFat.h>
#include <ArduinoJson.h>
#include <math.h>

#define TFT_CS 32
#define TFT_DC 33
#define TFT_RST 25
#define MAG_SDA 16
#define MAG_SCL 22
#define SIM_TX 4
#define SIM_RX 2
#define SD_CS 5

struct Waypoint {
  float lat;
  float lon;
  bool visited;
};

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Adafruit_QMC5883P mag = Adafruit_QMC5883P();
DFRobot_SIM7000 sim7000(&Serial2);
SdFat SD;

Waypoint* waypoints = nullptr;
int currentTargetIndex = -1;

int posX = 10, posY = 10;
uint32_t loopTimer = 0, screenTimer = 0, transmisionTimer = 0;
bool APNStatus = false, connected = false;
byte mode = 0;
String currentTime, latitude, longitude;
float azimuth;
int gpsErrorCounter = 0;
float distanceToTarget = 0.0;
int arrowAngle = 0;

int savedCount = 0, totalWaypoints = 0;
int ok = 0, error = 0;

const int MAX_GPS_ERRORS = 6;
const char* DEVICE_ID = "tracker01";
const char* OFFLINE_FILE = "offlineData.txt";
const char* NAVI_FILE = "naviData.txt";
const char* SERVER = "http://frog03.mikr.us:21491/api/device/telemetry";
const char* MODE_URL = "http://frog03.mikr.us:21491/api/device/mode";
const char* WAYPOINTS_URL = "http://frog03.mikr.us:21491/api/device/pending-waypoints";

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
void handleTransmission();
void saveToSD(String json);
void processOfflineDataBatch();
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);
void findNextClosestWaypoint(float currentLat, float currentLon);
void drawNavigationArrow(int cx, int cy, int angle, uint16_t arrowColor);
float getAzimuth();

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);
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

  if (!SD.begin(SD_CS, SD_SCK_MHZ(4))) {
    Serial.println("SD Card Mount Failed!");
  } else {
    Serial.println("SD Card Initialized.");
  }
  SD.remove(OFFLINE_FILE);

  initializeSIM7000();
  initializeSIMCard();
  initializeNetwork();
  delay(5000);
  configureAPN();
  initializeGPS();

  delay(1000);
  posY = 10;
  waitForGPS();
  tft.fillScreen(ILI9341_BLACK);

  // =========================================================================
  // --- SPRAWDZANIE TRYBU I POBIERANIE WSPÓŁRZĘDNYCH ---
  // =========================================================================
  int setupX = 10, setupY = 10;

  printLine("Waiting for server...", setupX, setupY, ILI9341_WHITE, 2);
  
  String modeResponse = "";

  sim7000.httpGet(MODE_URL);
  
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
      if(line.indexOf("+HTTPACTION: 0,200") != -1) {
        actionOk = true;
        break;
      }
    }
    delay(10);
  }
  
  if(actionOk) {
    Serial2.println("AT+HTTPREAD");
    delay(500);
    while(Serial2.available()) {
      modeResponse += (char)Serial2.read();
    }
  }

  if (modeResponse.indexOf("\"mode\":\"planning\"") != -1) {
    mode = 1;
    replaceLine("MODE: NAVIGATION", setupX, setupY, ILI9341_YELLOW, 2);
    printLine("Downloading waypoints...", setupX, setupY, ILI9341_WHITE, 2);
    
    SD.remove(NAVI_FILE);
    FsFile naviFile = SD.open(NAVI_FILE, O_RDWR | O_CREAT | O_APPEND);
    
    if (!naviFile) {
      printLine("SD Write Error!", setupX, setupY, ILI9341_RED, 2);
      mode = 0;
      return;
    }

    bool morePointsAvailable = true;

    while (morePointsAvailable) {
      while(Serial2.available()) Serial2.read();
      
      Serial2.print("AT+HTTPPARA=\"URL\",\"");
      Serial2.print(WAYPOINTS_URL);
      Serial2.println("\"");
      delay(200);
      
      Serial2.println("AT+HTTPACTION=0");
      
      unsigned long startWp = millis();
      bool wpActionOk = false;
      
      while(millis() - startWp < 8000) {
        if(Serial2.available()) {
          String line = Serial2.readStringUntil('\n');
          
          if(line.indexOf("+HTTPACTION:") != -1) {
            if(line.indexOf("0,200") != -1) {
              wpActionOk = true;
            } else {
              Serial.print("MODEM RETURNED ERROR STATE: ");
              Serial.println(line);
            }
            break;
          }
        }
        delay(10);
      }
      
      if(!wpActionOk) {
        Serial.println("Pętla zatrzymana: Brak statusu HTTP 200.");
        morePointsAvailable = false;
        break;
      }
      
      Serial2.println("AT+HTTPREAD");
      
      String wpResponse = "";
      unsigned long lastCharTime = millis();
      while (millis() - lastCharTime < 1000) {
        while (Serial2.available()) {
          char c = (char)Serial2.read();
          wpResponse += c;
          lastCharTime = millis();
        }
        delay(5);
      }
      
      int pointsIdx = wpResponse.indexOf("\"points\":[");
      if (pointsIdx != -1) {
        pointsIdx += 10;
        int batchSaved = 0;
        
        while (true) {
          int latIndex = wpResponse.indexOf("\"lat\":", pointsIdx);
          if (latIndex == -1) break;
          int latEnd = wpResponse.indexOf(",", latIndex);
          String latVal = wpResponse.substring(latIndex + 6, latEnd);
          
          int lngIndex = wpResponse.indexOf("\"lng\":", latEnd);
          if (lngIndex == -1) break;
          int lngEnd = wpResponse.indexOf("}", lngIndex);
          String lngVal = wpResponse.substring(lngIndex + 6, lngEnd);
          
          naviFile.println(latVal + "," + lngVal);
          savedCount++;
          batchSaved++;
          
          pointsIdx = lngEnd;
        }
        
        if (batchSaved == 0) {
          morePointsAvailable = false;
        }
        
        Serial.print("Pomyślnie zapisano partię ");
        Serial.print(batchSaved);
        Serial.println(" punktów.");
        
      } else {
          morePointsAvailable = false;
      }
      
      delay(1500); 
    }
    
    naviFile.close();
    
    if (savedCount > 0) {
      totalWaypoints = savedCount;

      printLine("Saved " + String(savedCount) + " points!", setupX, setupY, ILI9341_GREEN, 2);
      waypoints = new Waypoint[totalWaypoints];
      
      // 2. Otwarcie pliku ponownie - tym razem tylko DO ODCZYTU
      FsFile readFile = SD.open(NAVI_FILE, O_RDONLY);
      if (readFile) {
        int idx = 0;
        
        while (readFile.available() && idx < totalWaypoints) {
          String line = readFile.readStringUntil('\n');
          line.trim();
          
          int commaIdx = line.indexOf(',');
          if (commaIdx != -1) {
            String latStr = line.substring(0, commaIdx);
            String lonStr = line.substring(commaIdx + 1);
            
            waypoints[idx].lat = latStr.toFloat();
            waypoints[idx].lon = lonStr.toFloat();
            waypoints[idx].visited = false;
            
            idx++;
          }
        }
        readFile.close();
        
        Serial.print("[NAVI] Pomyślnie załadowano do RAM: ");
        Serial.print(idx);
        Serial.println(" punktów trasy.");
      } else {
        Serial.println("[CRITICAL] Nie udało się otworzyć pliku nawigacji do odczytu!");
        mode = 0;
      }
      
    } else {
      printLine("No active route found.", setupX, setupY, ILI9341_WHITE, 2);
      mode = 0;
    }
    delay(2000);
    
  } else {
    mode = 0;
  }

  tft.fillScreen(ILI9341_BLACK);
}

void loop() {
  switch (mode) {
    case 0:
      if (wait(loopTimer, 500)) {
        bool gotPosition = sim7000.getPosition();
        
        if (gotPosition) {
          latitude = sim7000.getLatitude();
          longitude = sim7000.getLongitude();
        } else {
          gpsErrorCounter++;
          
          if (gpsErrorCounter >= MAX_GPS_ERRORS) {
            waitForGPS();
            gpsErrorCounter = 0;
          }
        }
      }

      if (wait(screenTimer, 500)) {
        azimuth = getAzimuth();

        arrowAngle = (int)(round(azimuth / 15.0) * 15.0);
        if (arrowAngle >= 360) arrowAngle -= 360;
        updateScreen();
      }

      if (wait(transmisionTimer, 10000)) {
        handleTransmission();
      }
      
      break;

    case 1:
      if (wait(loopTimer, 500)) {
        bool gotPosition = sim7000.getPosition();
        
        if (gotPosition) {
          latitude = sim7000.getLatitude();
          longitude = sim7000.getLongitude();

          float tempLat = latitude.toFloat();
          float tempLon = longitude.toFloat();
          azimuth = getAzimuth();

          if (currentTargetIndex == -1) {
            findNextClosestWaypoint(tempLat, tempLon);
          }

          if (currentTargetIndex != -1) {
            distanceToTarget = calculateDistance(tempLat, tempLon, waypoints[currentTargetIndex].lat, waypoints[currentTargetIndex].lon);

            if (distanceToTarget <= 25.0) {
              Serial.print("PUNKT ZDOBYTY! Indeks: "); Serial.println(currentTargetIndex);
              waypoints[currentTargetIndex].visited = true;
              
              findNextClosestWaypoint(tempLat, tempLon);
            }

            if (currentTargetIndex != -1) {
              float bearingToTarget = calculateBearing(tempLat, tempLon, waypoints[currentTargetIndex].lat, waypoints[currentTargetIndex].lon);
              
              float relativeAngle = bearingToTarget - azimuth;
              if (relativeAngle < 0) relativeAngle += 360.0;

              arrowAngle = (int)(round(relativeAngle / 15.0) * 15.0);
              if (arrowAngle >= 360) arrowAngle -= 360;
            }
          } else {
            Serial.println("Wszystkie punkty zostały zdobyte! Koniec podróży.");
            arrowAngle = 0;
            distanceToTarget = 0.0;
            mode = 0;
          }

        } else {
          gpsErrorCounter++;
          if (gpsErrorCounter >= MAX_GPS_ERRORS) {
            waitForGPS();
            gpsErrorCounter = 0;
          }
        }
      }

      if (wait(screenTimer, 500)) {
        updateScreen();
      }

      break;
  }
}

bool wait(uint32_t &lastTime, uint32_t interval) {
  uint32_t current = millis();
  if (current - lastTime >= interval) {
    lastTime = current;
    return true;
  }
  return false;
}

void printLine(const String text, int &posX, int &posY, uint16_t color, int textSize) {
  tft.setCursor(posX, posY);
  tft.setTextColor(color);
  tft.setTextSize(textSize);
  tft.fillRect(posX, posY, 120, textSize * 10, ILI9341_BLACK);
  tft.println(text);
  posY += textSize * 10;
}

void replaceLine(const String text, int &posX, int &posY, uint16_t color, int textSize) {
  tft.fillRect(posX, posY - textSize * 10, tft.width() - posX * 2, textSize * 10, ILI9341_BLACK);
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

  if (mode == 0) {
    printLine(String(ok) + "/" + String(error), posX, posY, ILI9341_WHITE, 2);
  } else if (mode == 1) {
    printLine("Ang: " + String(arrowAngle), posX, posY, ILI9341_WHITE, 2);
  }

  drawNavigationArrow(230, 120, arrowAngle, ILI9341_RED);

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
    bool gotPos = sim7000.getPosition();

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
  Serial2.println("AT+CGNSINF");
  unsigned long start = millis();
  String response = "";

  while (millis() - start < 1000) {
    if (Serial2.available()) {
      response += (char)Serial2.read();
    }
    delay(10);
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

void handleTransmission() {
  float tempLat = latitude.toFloat();
  float tempLon = longitude.toFloat();


  if (tempLat == 0.0 || tempLon == 0.0) {
    Serial.println("BŁĄD WALIDACJI: Współrzędne to (0.0, 0.0) - brak poprawnego fixu GPS. Anulowano.");
    return; 
  }

  // B. Sprawdzenie fizycznych granic geograficznych (Ziemia)
  if (tempLat < -90.0 || tempLat > 90.0 || tempLon < -180.0 || tempLon > 180.0) {
    Serial.println("KRYTYCZNY BŁĄD: Współrzędne GPS poza zakresem fizycznym globu! Dane odrzucone.");
    return;
  }

  delay(100);

  StaticJsonDocument<256> doc;

  doc["device_id"] = String(DEVICE_ID);
  doc["timestamp"] = "";
  doc["lat"] = tempLat;
  doc["lon"] = tempLon;

  String json;
  serializeJson(doc, json);

  Serial.println("Wygenerowano poprawny i zweryfikowany pakiet telemetryczny:");
  Serial.println(json);

  bool success = sim7000.httpPost(SERVER, json);

  if (success) {
    connected = true;
    ok++;
    processOfflineDataBatch();
  } else {
    connected = false;
    error++;
    saveToSD(json);
  }
}

void saveToSD(String json) {
  FsFile file = SD.open(OFFLINE_FILE, O_RDWR | O_CREAT | O_APPEND);
  if (file) {
    file.println(json);
    file.close();
    Serial.println("Data appended to SD");
  } else {
    Serial.println("Failed to open SD for writing");
  }
}

void processOfflineDataBatch() {
  if (!SD.exists(OFFLINE_FILE)) {
    return;
  }
  FsFile file = SD.open(OFFLINE_FILE, O_READ);
  FsFile newFile = SD.open("temp.txt", O_RDWR | O_CREAT | O_TRUNC);

  if (!file || !newFile) {
    if (file) file.close();
    if (newFile) newFile.close();
    return;
  }

  Serial.println("Processing offline buffer in batches...");
  const int BATCH_SIZE = 10;
  bool batchFailed = false;

  while (true) {
    String linesCache[BATCH_SIZE];
    int linesRead = 0;

    while (file.available() && linesRead < BATCH_SIZE) {
      String jsonLine = file.readStringUntil('\n');
      jsonLine.trim();
      if (jsonLine.length() > 0) {
        linesCache[linesRead++] = jsonLine;
      }
    }
    bool hasMore = file.available();

    if (linesRead == 0) break;

    if (!batchFailed) {
      String batchJson = "[";
      for (int i = 0; i < linesRead; i++) {
        if (i > 0) batchJson += ",";
        batchJson += linesCache[i];
      }
      batchJson += "]";

      bool success = sim7000.httpPost(SERVER, batchJson);

      if (!success) {
        batchFailed = true;
        Serial.println("Batch send FAILED!");
      } else {
        Serial.println("Batch sent successfully!");
        delay(1500);
      }
    }

    if (batchFailed) {
      for (int i = 0; i < linesRead; i++) {
        newFile.println(linesCache[i]);
      }
    }
    delay(100);

    if (!hasMore) break;
  }

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
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * PI / 180.0;
  float dLon = (lon2 - lon1) * PI / 180.0;
  
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371000.0 * c;
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float lat1Rad = lat1 * PI / 180.0;
  float lat2Rad = lat2 * PI / 180.0;
  float dLon = (lon2 - lon1) * PI / 180.0;

  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
  
  float bearing = atan2(y, x) * 180.0 / PI;
  if (bearing < 0) bearing += 360.0;
  return bearing;
}

void findNextClosestWaypoint(float currentLat, float currentLon) {
  float minDistance = 9999999.0;
  int closestIdx = -1;

  for (int i = 0; i < totalWaypoints; i++) {
    if (!waypoints[i].visited) {
      float dist = calculateDistance(currentLat, currentLon, waypoints[i].lat, waypoints[i].lon);
      if (dist < minDistance) {
        minDistance = dist;
        closestIdx = i;
      }
    }
  }
  currentTargetIndex = closestIdx;
}

void drawNavigationArrow(int cx, int cy, int angle, uint16_t arrowColor) {
  static int lastAngle = -1;

  if (angle == lastAngle) return;

  tft.fillRect(cx - 70, cy - 70, 140, 140, ILI9341_BLACK);
  lastAngle = angle;

  tft.drawCircle(cx, cy, 68, ILI9341_DARKGREEN);
  tft.drawFastVLine(cx, cy - 68, 6, ILI9341_WHITE);
  tft.drawFastVLine(cx, cy + 62, 6, ILI9341_WHITE);
  tft.drawFastHLine(cx - 68, cy, 6, ILI9341_WHITE);
  tft.drawFastHLine(cx + 62, cy, 6, ILI9341_WHITE);

  float radMain   = angle * PI / 180.0;
  float radLeft   = (angle + 145) * PI / 180.0;
  float radRight  = (angle - 145) * PI / 180.0;
  float radIndent = (angle + 180) * PI / 180.0;

  int xTip    = cx + (int)(60 * sin(radMain));
  int yTip    = cy - (int)(60 * cos(radMain));

  int xLeft   = cx + (int)(45 * sin(radLeft));
  int yLeft   = cy - (int)(45 * cos(radLeft));

  int xRight  = cx + (int)(45 * sin(radRight));
  int yRight  = cy - (int)(45 * cos(radRight));

  int xIndent = cx + (int)(20 * sin(radIndent));
  int yIndent = cy - (int)(20 * cos(radIndent));
  
  tft.fillTriangle(xTip, yTip, xLeft, yLeft, xIndent, yIndent, arrowColor);
  
  uint16_t darkColor = (arrowColor == ILI9341_RED) ? 0x9000 : ILI9341_DARKGREY;
  tft.fillTriangle(xTip, yTip, xRight, yRight, xIndent, yIndent, darkColor);

  tft.drawTriangle(xTip, yTip, xLeft, yLeft, xIndent, yIndent, ILI9341_WHITE);
  tft.drawTriangle(xTip, yTip, xRight, yRight, xIndent, yIndent, ILI9341_WHITE);
}

float getAzimuth() {
  float headingDegrees = 0.0;

  if (mag.isDataReady()) {
    int16_t x, y, z;
    mag.getRawMagnetic(&x, &y, &z);

    const float X_OFFSET = -108.50;
    const float Y_OFFSET = -109.50;
    const float X_SCALE = 1.01;
    const float Y_SCALE = 1.02;

    float x_cal = ((float)x - X_OFFSET) * X_SCALE;
    float y_cal = ((float)y - Y_OFFSET) * Y_SCALE;

    float heading = atan2(-y_cal, x_cal);

    float declinationAngle = 0.1047;
    heading += declinationAngle;

    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;

    headingDegrees = heading * 180 / M_PI;
  }
  
  return headingDegrees;
}