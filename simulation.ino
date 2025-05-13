

/*

ESP32 based solar tracker
setup assumes "zero" (hall sensor) position facing NORTH
28BYJ-48 electric motors used for azimuth and elevation tracking
Current location calculated from IP address
*/

//motor control declarations
#include <AccelStepper.h>

#define FULLSTEP 4      //from motor specs
#define REV 2048        //steps for full revolution from motor specs
#define PIN_AZI_HOME 5  //Hall sensor digital input
#define PIN_ELEV_HOME 19    //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

AccelStepper stepperAzi(FULLSTEP, 13, 14, 12, 27);
AccelStepper stepperElev(FULLSTEP, 26, 33, 25, 32);   //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//web request declarations
#include <WiFi.h>
#include <time.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#define WIFI_SSID "autobra"
#define WIFI_PASSWORD "731731177"  //todo: find a way to avoid hardcoding SSID and password into the chip
#define LOCATION_REQ_ADDR "http://ip-api.com/json/?fields=33579200"
#define NTP_SERVER "pool.ntp.org"

JsonDocument JSONDoc;  //Memory pool for JSON data from ip-api server
struct tm timeinfo;    //time returned from NTP server
time_t utc;            //unix timestamp

//position calculation declarations
#include <SolarCalculator.h>

double lat;
double lon;
double azi = 0;
double elev = 0;

//debug LED declarations
#define PIN_STAT_LED 18
#define WAIT_SHORT 50
#define WAIT_LONG 500

unsigned long prevMillis;
unsigned long currentMillis;

void setup() {
  // Настройка пинов
  pinMode(PIN_AZI_HOME, INPUT_PULLDOWN);
  pinMode(PIN_ELEV_HOME, INPUT_PULLDOWN);
  pinMode(PIN_STAT_LED, OUTPUT);

  // Настройка Serial
  Serial.begin(115200);
  Serial.println("Delay before setup...");
  delay(2000);

  // Настройка шаговых двигателей
  stepperAzi.setMaxSpeed(500);
  stepperAzi.setAcceleration(100);
  stepperAzi.setSpeed(500);

  stepperElev.setMaxSpeed(500);
  stepperElev.setAcceleration(100);
  stepperElev.setSpeed(500);

  // Инициализация переменных
  bool WiFiConnected = false;
  bool stepperAziHome = false;
  bool stepperElevHome = false;

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Initial setuo...");
  blinkLED(WAIT_SHORT, 1);
  prevMillis = millis();

  // Переменные для мотора азимута
  long initialAziPosition = stepperAzi.currentPosition();
  int aziDirection = 1; // 1 - по часовой, -1 - против часовой
  bool directionChanged = false; // Флаг смены направления

  // Цикл калибровки и подключения к Wi-Fi
  while (!WiFiConnected || !stepperAziHome || !stepperElevHome) {
    currentMillis = millis();

    // Проверка подключения Wi-Fi
    if (WiFi.status() == WL_CONNECTED && !WiFiConnected) {
      WiFiConnected = true;
      Serial.print("[Wi-Fi connected]");
    }

    // Управление мотором азимута
    if (!stepperAziHome) {
      if (digitalRead(PIN_AZI_HOME)) {
        stepperAziHome = true;
        Serial.print("[Azi motor homing complete]");
        stepperAzi.setCurrentPosition(0);
        stepperAzi.disableOutputs();
      } else {
        long stepsMoved = abs(stepperAzi.currentPosition() - initialAziPosition);
        if (stepsMoved >= 1024 && !directionChanged) {
          // Смена направления после 180 градусов (1024 шага)
          aziDirection *= -1;
          initialAziPosition = stepperAzi.currentPosition();
          stepperAzi.setSpeed(500 * aziDirection);
          directionChanged = true;
          Serial.println("[Azi motor changed direction]");
        }
        stepperAzi.runSpeed(); // Продолжаем движение
      }
    }

    // Управление мотором высоты
    if (!stepperElevHome) {
      if (digitalRead(PIN_ELEV_HOME)) {
        stepperElevHome = true;
        Serial.print("[Elev motor homing complete]");
        stepperElev.setCurrentPosition(0);
        stepperElev.disableOutputs();
      } else {
        stepperElev.runSpeed();
      }
    }

    // Индикация процесса
    if (currentMillis - prevMillis > 100) {
      Serial.print("-");
      digitalWrite(PIN_STAT_LED, !digitalRead(PIN_STAT_LED));
      prevMillis = currentMillis;
    }
  }
  Serial.println("");
  digitalWrite(PIN_STAT_LED, LOW);

  // Получение данных о местоположении
  Serial.println("Getting location data...");
  blinkLED(WAIT_SHORT, 2);
  HTTPClient http;
  http.begin(LOCATION_REQ_ADDR);
  int httpCode = http.GET();
  String payload;

  if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
    payload = http.getString();
    Serial.println(payload);
  } else {
    Serial.printf("[HTTP GET failed...: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();

  // Разбор JSON для получения координат
  DeserializationError error = deserializeJson(JSONDoc, payload);
  if (!error) {
    lat = JSONDoc["lat"];
    lon = JSONDoc["lon"];
  Serial.printf("JSON parse success: Latitude: %f, longtitude: %f\n", lat, lon);
  } else {
    Serial.println("JSON parse failed.");
  }

  adjustPanel();
  Serial.println("Setup done.");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);  //wait 10 minutes
  adjustPanel();
}

void adjustPanel() {
  blinkLED(WAIT_SHORT,5);

  // Симуляция движения солнца
  azi += 15;
  if (azi >= 360) {
    azi -= 360;  // Сброс азимута после полного оборота
  }
  bool isRising = true;
  if (isRising) {
    elev += 5;
    if (elev >= 55) {
      elev = 55;
      isRising = false;  // Начать закат
    }
  } else {
    elev -= 1;
    if (elev <= 0) {
      elev = 0;
      isRising = true;  // Начать новый день
    }
  }

  Serial.printf("Simulated coordinates: azimuth: %f, elevation: %f...\n", azi, elev);

  Serial.println("Rotating platform into position...");

  //rotating Azi motor
  stepperAzi.enableOutputs();
  double bufazi;
  if (azi > 180) {
    bufazi = 360 - azi;
  }
  if (0 <= azi && azi <= 180) {
    bufazi = -azi;
  }
  stepperAzi.moveTo(convertDegToRev(bufazi));
  while (stepperAzi.distanceToGo() != 0) {
    stepperAzi.run();
  }
  stepperAzi.disableOutputs();

  //rotating Elev motor
  stepperElev.enableOutputs();
  stepperElev.moveTo(convertDegToRev(elev - 90));
  while (stepperElev.distanceToGo() != 0) {
    stepperElev.run();
  }
  stepperElev.disableOutputs();
}

void blinkLED(int msEach, int amtBlinks){
  digitalWrite(PIN_STAT_LED, LOW);
  for (int i = 0; i < (amtBlinks*2); i++){
    digitalWrite(PIN_STAT_LED, !digitalRead(PIN_STAT_LED));
    delay(msEach/2);
  }
}

double convertDegToRev(double deg) {
  //360 degrees = 1 full revolution (REV macro)
  //i.e. 180 degrees = 1/2 turn = REV * 1/2
  //to convert, multiply REV by deg/360
  double revOut = REV * (deg / 360);
  return revOut;
}
