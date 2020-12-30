
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <EEPROM.h>
#include "FastLED.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <DS3232RTC.h>
#include <Streaming.h>
#include <TimeAlarms.h>
#include <Time.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define SEALEVELPRESSURE_HPA (1013.25)

#define DATA_PIN 12
#define INTERRUPT_PIN 23
#define BRIGHTNEES 40
#define NUM_LEDS 1
#define RESET_15SEG 23
#define ADC_S1 32 //Sensor vegetronix 1 vh400
#define ADC_S2 33 //Sensor vegetronix 2 therm200
#define ADC_S3 34 //Sensor vegetronix 1
#define adBatery 35
#define adPanel 36
#define DEBUG

#ifdef DEBUG
#define Sprintln(x) Serial.println(x)
#define Sprint(x) Serial.print(x)

#define DEBUG_ON
#define wbdebug true

#else
#define Sprintln(x)
#define Sprint(x)

#define wbdebug false

#endif
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10       /* ESP32 should sleep more seconds  (note SIM7000 needs ~20sec to turn off if sleep is activated) */
#define TIME_TO_SLEEP2 25       /* ESP32 should sleep more seconds  (note SIM7000 needs ~20sec to turn off if sleep is activated) */
RTC_DATA_ATTR int bootCount = 0;

String ssdit = "", passwordt = "", ip = "", subnet = "", gateway = "", dServer = "", correo = "", dDatos = "", sMqtt = "", usrApn = "", pasApn = "", apn = "";
int rt1 = 0, idDevice = 0;
float b[5] = {1, 17.5, 47.5, 7.89, 87.5};
float m[5] = {10, 25, 48.08, 26.32, 62.5};
const char *host = "esp32fs";
IPAddress ipa, suba, gat;
IPAddress dns(8, 8, 8, 8);

int16_t ax, ay, az;
int16_t gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

MPU6050 mpu(0x69);
DS3232RTC RTC(false);
CRGB leds[NUM_LEDS];
WebServer server(80);
File fsUploadFile;
Adafruit_BME680 bme; // I2C
typedef struct
{
  char ssdit[20];
  char passwordt[15];
  char ip[22];
  char gateway[22];
  char subnet[22];
  char dServer[60];
  char correo[60];
  char dDatos[60];
  char sMqtt[60];
  char usrApn[10];
  char pasApn[15];
  char apn[25];
  int idDevice;
  int rt1;
} dat;

dat dataSaved;
void ledBlink(byte r, byte g, byte b)
{
  for (size_t i = 0; i < 3; i++)
  {

    leds[0] = CRGB(r, g, b);
    FastLED.show();
    Alarm.delay(200);
    leds[0] = CRGB::Black;
    FastLED.show();
    Alarm.delay(200);
  }
}

float therm200(float v)
{

  return v * 41.67 - 40;
}
float vh400(float v)
{
  float vwc = 0;
  if (v > 0 && v < 1.1)
  {
    vwc = m[0] * v - b[0];
  }
  if (v >= 1.1 && v < 1.3)
  {
    vwc = m[1] * v - b[1];
  }
  if (v >= 1.3 && v < 1.82)
  {
    vwc = m[2] * v - b[2];
  }
  if (v > 1.82 && v < 2.2)
  {
    vwc = m[3] * v - b[3];
  }
  if (v > 2.2 && v < 3.0)
  {
    vwc = m[4] * v - b[4];
  }

  return vwc;
}
String formatBytes(size_t bytes)
{
  if (bytes < 1024)
  {
    return String(bytes) + "B";
  }
  else if (bytes < (1024 * 1024))
  {
    return String(bytes / 1024.0) + "KB";
  }
  else if (bytes < (1024 * 1024 * 1024))
  {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  }
  else
  {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

bool exists(String path)
{
  bool yes = false;
  File file = SPIFFS.open(path, "r");
  if (!file.isDirectory())
  {
    yes = true;
  }
  file.close();
  return yes;
}

bool handleFileRead(String path)
{
  Sprintln("handleFileRead: " + path);

  if (exists(path))
  {

    File file = SPIFFS.open(path, "r");
    server.streamFile(file, "text/html");
    file.close();
    return true;
  }
  return false;
}

void handleHome()
{
  handleFileRead("/home.html");
}
void handleScan()
{
  int n = WiFi.scanNetworks();
  // Sprintln("scan done");
  String g = "{\"Scan\":[";
  //Sprint(n);
  // Sprintln(" networks found");
  if (n == 0)
  {
    // Sprintln("no networks found");
  }
  else
  {
    for (int i = 0; i < n; ++i)
    {
      g += "\"" + String(WiFi.SSID(i)) + "\",";
      Alarm.delay(10);
    }
  }
  g = g.substring(0, g.lastIndexOf(","));
  g += "]}";
  Sprintln(g);
  server.send(200, "text/json", g);
}
void handlesaveRed()
{
  ssdit = server.arg("ssdit");
  passwordt = server.arg("passwordt");
  ip = server.arg("ip");
  subnet = server.arg("subnet");
  gateway = server.arg("gateway");
  strncpy(dataSaved.ssdit, ssdit.c_str(), sizeof(dataSaved.ssdit));
  strncpy(dataSaved.passwordt, passwordt.c_str(), sizeof(dataSaved.passwordt));
  strncpy(dataSaved.ip, ip.c_str(), sizeof(dataSaved.ip));
  strncpy(dataSaved.subnet, subnet.c_str(), sizeof(dataSaved.subnet));
  strncpy(dataSaved.gateway, gateway.c_str(), sizeof(dataSaved.gateway));
  EEPROM.put(0, dataSaved);
  EEPROM.commit();
  server.send(200, "text/html", "OK");
  ledBlink(255, 255, 255);
}
void handlesaveDatos()
{
  rt1 = server.arg("rt1").toInt();
  dServer = server.arg("dServer");
  correo = server.arg("correo");
  dDatos = server.arg("dDatos");
  sMqtt = server.arg("sMqtt");
  usrApn = server.arg("usrApn");
  pasApn = server.arg("pasApn");
  apn = server.arg("apn");
  idDevice = server.arg("idDevice").toInt();
  strncpy(dataSaved.dServer, dServer.c_str(), sizeof(dataSaved.dServer));
  strncpy(dataSaved.correo, correo.c_str(), sizeof(dataSaved.correo));
  strncpy(dataSaved.dDatos, dDatos.c_str(), sizeof(dataSaved.dDatos));
  strncpy(dataSaved.sMqtt, sMqtt.c_str(), sizeof(dataSaved.sMqtt));
  strncpy(dataSaved.usrApn, usrApn.c_str(), sizeof(dataSaved.usrApn));
  strncpy(dataSaved.pasApn, pasApn.c_str(), sizeof(dataSaved.pasApn));
  strncpy(dataSaved.apn, apn.c_str(), sizeof(dataSaved.apn));
  dataSaved.rt1 = rt1;
  dataSaved.idDevice = idDevice;
  EEPROM.put(0, dataSaved);
  EEPROM.commit();
  server.send(200, "text/html", "OK");
  ledBlink(255, 255, 255);
}
void handleinit()
{
  String data = "";
  data += "{ \"host\":\"" + ssdit + "\"";
  data += ",\"passowrd\":\"" + passwordt + "\"";
  data += ",\"ip\":\"" + ip + "\"";
  data += ",\"r1\":\"" + String(rt1) + "\"";
  data += ",\"gateway\":\"" + gateway + "\"";
  data += ",\"subnet\":\"" + subnet + "\"";
  data += ",\"dServer\":\"" + dServer + "\"";
  data += ",\"Correo\":\"" + correo + "\"";
  data += ",\"dDatos\":\"" + dDatos + "\"";
  data += ",\"sMqtt\":\"" + sMqtt + "\"";
  data += ",\"usrApn\":\"" + usrApn + "\"";
  data += ",\"pasApn\":\"" + pasApn + "\"";
  data += ",\"apn\":\"" + apn + "\"";
  data += ",\"idDevice\":\"" + String(idDevice);
  data += "\"}";
  Sprintln(data);
  server.send(200, "text/json", data);
}

void handlereset()
{
  Sprintln("reset");
  Sprintln(EEPROM.length());
  for (int i = 0; i < 512; i++)
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  server.send(200, "text/html", "ok");
  ESP.restart();
}
IPAddress rip(String ip)
{
  int Parts[4] = {0, 0, 0, 0};
  int Part = 0;
  for (int i = 0; i < ip.length(); i++)
  {
    char c = ip[i];
    if (c == '.')
    {
      Part++;
      continue;
    }
    Parts[Part] *= 10;
    Parts[Part] += c - '0';
  }
  IPAddress ipa(Parts[0], Parts[1], Parts[2], Parts[3]);
  return ipa;
}

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}
void printDigits(int digits)
{

  Sprint(':');
  if (digits < 10)
    Sprint('0');
  Sprint(digits);
}
void digitalClockDisplay()
{

  Sprint(hour());
  printDigits(minute());
  printDigits(second());
  Sprint(' ');
  Sprint(day());
  Sprint(' ');
  Sprint(month());
  Sprint(' ');
  Sprint(year());
  Sprintln();
}
void mpu_loop()
{
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  //Calcular los ángulos con acelerometro
  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

  //Calcular angulo de rotación con giroscopio y filtro complemento
  ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
}
void printI00(int val, char delim)
{
  if (val < 10)
    Serial << '0';
  Serial << _DEC(val);
  if (delim > 0)
    Serial << delim;
  return;
}
void printTime(time_t t)
{
  printI00(hour(t), ':');
  printI00(minute(t), ':');
  printI00(second(t), ' ');
}

void printDate(time_t t)
{
  printI00(day(t), 0);
  Serial << monthShortStr(month(t)) << _DEC(year(t));
}
void printDateTime(time_t t)
{
  printDate(t);
  Serial << ' ';
  printTime(t);
}

void restartT()
{

  ESP.restart();
}
void handleset()
{
  int hour = server.arg("hr").toInt();
  int min = server.arg("min").toInt();
  int sec = server.arg("sec").toInt();
  int month = server.arg("month").toInt();
  int day = server.arg("day").toInt();
  int year = server.arg("year").toInt();
  Sprintln();

  Sprint(hour);
  Sprint(":");
  Sprint(min);
  Sprint(":");
  Sprint(sec);
  Sprint(" ");
  Sprint(day);
  Sprint("/");
  Sprint(month);
  Sprint("/");
  Sprintln(year);
  time_t t;
  tmElements_t tm;
  tm.Year = CalendarYrToTm(year);
  tm.Month = month;
  tm.Day = day;
  tm.Hour = hour;
  tm.Minute = min;
  tm.Second = sec;
  t = makeTime(tm);
  RTC.set(t);
  setTime(t);
  printDateTime(t);

  ledBlink(255, 0, 100);
  server.send(200, "text/html", "ok");

  Alarm.timerOnce(0, 0, 1, restartT);
}
float readMeasure(int pin)
{
  float f = 0;
  for (size_t i = 0; i < 100; i++)
  {
    f += (analogRead(pin) * 3.3) / 4096.0;
  }
  f = f / 100.0;
  Serial.print(f, 4);
  Sprint(" ");
  return f;
}
void showData()
{

  // if (!bme.performReading())
  // {
  //   Serial.println("Failed to perform reading :(");
  //   return;
  // }
  // Sprint("Temperature = ");
  // Sprint(bme.temperature);
  // Sprintln(" *C");

  // Sprint("Pressure = ");
  // Sprint(bme.pressure / 100.0);
  // Sprintln(" hPa");

  // Sprint("Humidity = ");
  // Sprint(bme.humidity);
  // Sprintln(" %");

  // Sprint("Gas = ");
  // Sprint(bme.gas_resistance / 1000.0);
  // Sprintln(" KOhms");

  // Sprint("Approx. Altitude = ");
  // Sprint(bme.readAltitude(SEALEVELPRESSURE_HPA));
  // Sprintln(" m");

  Sprint("X:  ");
  Sprint(ang_x);
  Sprint(" Y: ");
  Sprintln(ang_y);

  Sprint(" bat: ");
  Sprint(readMeasure(adBatery));
  Sprint(" panel ");
  Sprintln(readMeasure(adPanel));
  Sprint(" vh400: ");
  Serial.print(vh400((readMeasure(ADC_S1))), 4);
  Sprint("% therm200: ");
  Serial.print(therm200((readMeasure(ADC_S2))), 4);
  Sprintln("ºC");
  digitalClockDisplay();
}
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Sprintln("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Sprintln("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Sprintln("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Sprintln("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Sprintln("Wakeup caused by ULP program");
    break;
  default:

    break;
  }
}

void setup(void)
{

  Serial.begin(9600);
  Sprint("\n");
  Serial.setDebugOutput(true);
  pinMode(adPanel, INPUT);
  pinMode(adBatery, INPUT);
  pinMode(ADC_S1, INPUT);
  ++bootCount;
  Sprintln("Boot number: " + String(bootCount));
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection())
    Sprintln("mpu iniciado correctamente");
  print_wakeup_reason();
 

  if (!EEPROM.begin(512))
  {
    Sprintln("failed to initialise EEPROM");
    delay(1000000);
  }

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNEES);
  leds[0] = CRGB::Green;
  FastLED.show();
  RTC.begin();
  //SPIFFS.format();
  setSyncProvider(RTC.get);
  EEPROM.get(0, dataSaved);
  ssdit = dataSaved.ssdit;
  passwordt = dataSaved.passwordt;
  ip = dataSaved.ip;
  gateway = dataSaved.gateway;
  subnet = dataSaved.subnet;
  rt1 = dataSaved.rt1;
  dDatos = dataSaved.dDatos;
  correo = dataSaved.correo;
  dServer = dataSaved.dServer;
  sMqtt = dataSaved.sMqtt;
  idDevice = dataSaved.idDevice;
  usrApn = dataSaved.usrApn;
  pasApn = dataSaved.pasApn;
  apn = dataSaved.apn;
  Sprint(ssdit);
  Sprint(" ");
  Sprint(passwordt);
  Sprint(" ");
  Sprint(ip);
  Sprint(" ");
  Sprint(gateway);
  Sprint(" ");
  Sprint(subnet);
  Sprint(" ");
  Sprint(rt1);
  Sprint(" ");
  Sprint(dDatos);
  Sprint(" ");
  Sprint(correo);
  Sprint(" ");
  Sprint(dServer);
  Sprint(" ");
  Sprint(sMqtt);
  Sprint(" ");
  Sprint(idDevice);
  Sprint(" ");
  Sprint(usrApn);
  Sprint(" ");
  Sprint(pasApn);
  Sprint(" ");
  Sprintln(apn);
  Serial.setDebugOutput(true);
  SPIFFS.begin();

  if (ssdit.length() != 0)
  {
    WiFi.mode(WIFI_STA);
    ipa = rip(ip);
    suba = rip(subnet);
    gat = rip(gateway);
    WiFi.config(ipa, gat, suba, dns);
    WiFi.begin(ssdit.c_str(), passwordt.c_str());
    int8_t timeout = 15;

    while (WiFi.status() != WL_CONNECTED && timeout)
    {
      leds[0] = CRGB::Blue;
      FastLED.show();
      delay(250);
      Sprint(".");
      leds[0] = CRGB::Black;
      FastLED.show();
      delay(250);
      --timeout;
    }
    if (timeout != 0)
    {
      // if (!bme.begin()) {
      //   Serial.println("Could not find a valid BME680 sensor, check wiring!");
      //   while (1);
      // }
      // bme.setTemperatureOversampling(BME680_OS_8X);
      // bme.setHumidityOversampling(BME680_OS_2X);
      // bme.setPressureOversampling(BME680_OS_4X);
      // bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      //bme.setGasHeater(320, 150);
      leds[0] = CRGB::Green;
      FastLED.show();
      Sprintln("");
      Sprint("Connected! IP address: ");
      Sprintln(WiFi.localIP());
      Sprintln("A Dormir");
      for(int i =0; i<100;i++)
      mpu_loop();
      showData();
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Sprintln("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
      leds[0] = CRGB::Black;
      FastLED.show();

      esp_deep_sleep_start();
      
    }
    else
    {
      leds[0] = CRGB::Red;
      FastLED.show();
      WiFi.mode(WIFI_AP);
      WiFi.softAP("modulo");
      IPAddress IP = WiFi.softAPIP();
      Sprint("AP IP address: ");
      Sprintln(IP);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP2 * uS_TO_S_FACTOR);
      Sprintln("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP2) + " Seconds");
      leds[0] = CRGB::Black;
      FastLED.show();

      esp_deep_sleep_start();
      
    }
  }
  else
  {
    leds[0] = CRGB::Red;
    FastLED.show();
    WiFi.mode(WIFI_AP);
    WiFi.softAP("modulo");
    IPAddress IP = WiFi.softAPIP();
    Sprint("AP IP address: ");
    Sprintln(IP);
  }

  MDNS.begin(host);
  Sprint("Open http://");
  Sprintln(host);

  server.on("/", HTTP_GET, handleHome);

  server.on("/conf", HTTP_GET, []() {
    if (!handleFileRead("/datos.html"))
    {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/dashBoard", HTTP_GET, []() {
    if (!handleFileRead("/dashboard.html"))
    {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/red", HTTP_GET, []() {
    if (!handleFileRead("/red.html"))
    {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/scan", handleScan);
  server.on("/saveRed", handlesaveRed);
  server.on("/saveDatos", handlesaveDatos);
  server.on("/init", handleinit);
  server.on("/reset", handlereset);
  server.on("/set", handleset);
  server.on("/all", HTTP_GET, []() {
    String json = "{";
    json += "\"heap\":" + String(ESP.getFreeHeap());
    json += ", \"analog\":" + String(analogRead(A0));
    json += ", \"gpio\":" + String((uint32_t)(0));
    json += "}";
    server.send(200, "text/json", json);
    json = String();
  });
  server.begin();
  Sprintln("HTTP server started");
  Alarm.timerRepeat(0, 0, 1, showData);
}

void loop(void)
{
  mpu_loop();
  server.handleClient();
  Alarm.delay(0);
}