#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define DUMP_AT_COMMANDS
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
#include <PubSubClient.h>
#include "TinyGsmClient.h"

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
#define isNBIOT true
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_DTR 25

#define MODEM_TX 26
#define MODEM_RX 27

#define I2C_SDA 21
#define I2C_SCL 22

#define TINY_GSM_USE_GPRS true

HardwareSerial serialGsm(1);
TinyGsm modem(serialGsm);

#define SerialAT serialGsm


bool modemConnected = false;





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
#define TIME_TO_SLEEP 25       /* ESP32 should sleep more seconds  (note SIM7000 needs ~20sec to turn off if sleep is activated) */
#define TIME_TO_SLEEP2 25      /* ESP32 should sleep more seconds  (note SIM7000 needs ~20sec to turn off if sleep is activated) */
RTC_DATA_ATTR int bootCount = 0;

String mqtt_server = "";
int mqtt_port = 1883;
String mqtt_user = "";
String mqtt_pass = "";
String root_topic_subscribe = "";
String root_topic_publish = "";

String uroot = "";
String ssdit = "", passwordt = "", ip = "", subnet = "", gateway = "", usrApn = "", pasApn = "", apn = "";

int tSleep1 = 0, idDevice = 0;
float b[5] = {1, 17.5, 47.5, 7.89, 87.5};
float m[5] = {10, 25, 48.08, 26.32, 62.5};
bool sensorS[6];
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

WiFiClient espClient;
PubSubClient client(espClient);

char msg[256];
long count = 0;
File fsUploadFile;
Adafruit_BME680 bme; // I2C

typedef struct
{
  char ssdit[20];
  char passwordt[15];

  char ip[22];
  char subnet[22];
  char gateway[22];

  char mqtt_server[22];
  char mqtt_user[20];
  char mqtt_pass[20];
  char uroot[20];
  char root_topic_subscribe[20];
  char root_topic_publish[20];

  char usrApn[15];
  char pasApn[15];
  char apn[25];
  int idDevice;
  int tSleep1;
  int mqtt_port;
  bool sensorR[6];
} dat;

dat dataSaved;
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void setup_wifi();

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
  tSleep1 = server.arg("tSleep1").toInt();

  mqtt_server = server.arg("mqtt_server");
  mqtt_port = server.arg("mqtt_port").toInt();
  mqtt_user = server.arg("uServer");
  mqtt_pass = server.arg("pServer");

  uroot = server.arg("root");
  root_topic_subscribe = server.arg("rootSubs");
  root_topic_publish = server.arg("rootPublish");

  usrApn = server.arg("usrApn");
  pasApn = server.arg("pasApn");
  apn = server.arg("apn");
  idDevice = server.arg("idDevice").toInt();
  for (size_t i = 0; i < 6; i++)
    sensorS[i] = server.arg("s" + String(i + 1)) == "true" ? 1 : 0;

  strncpy(dataSaved.mqtt_server, mqtt_server.c_str(), sizeof(dataSaved.mqtt_server));
  strncpy(dataSaved.mqtt_user, mqtt_user.c_str(), sizeof(dataSaved.mqtt_user));
  strncpy(dataSaved.mqtt_pass, mqtt_pass.c_str(), sizeof(dataSaved.mqtt_pass));
  strncpy(dataSaved.uroot, uroot.c_str(), sizeof(dataSaved.uroot));
  strncpy(dataSaved.root_topic_subscribe, root_topic_subscribe.c_str(), sizeof(dataSaved.root_topic_subscribe));
  strncpy(dataSaved.root_topic_publish, root_topic_publish.c_str(), sizeof(dataSaved.root_topic_publish));

  strncpy(dataSaved.usrApn, usrApn.c_str(), sizeof(dataSaved.usrApn));
  strncpy(dataSaved.pasApn, pasApn.c_str(), sizeof(dataSaved.pasApn));
  strncpy(dataSaved.apn, apn.c_str(), sizeof(dataSaved.apn));

  dataSaved.tSleep1 = tSleep1;
  dataSaved.mqtt_port = mqtt_port;
  memcpy(dataSaved.sensorR, sensorS, 6);

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
  data += ",\"r1\":\"" + String(tSleep1) + "\"";
  data += ",\"gateway\":\"" + gateway + "\"";
  data += ",\"subnet\":\"" + subnet + "\"";
  data += ",\"mqtt_server\":\"" + mqtt_server + "\"";
  data += ",\"mqtt_port\":\"" + String(mqtt_port) + "\"";
  data += ",\"mqtt_user\":\"" + mqtt_user + "\"";
  data += ",\"mqtt_pass\":\"" + mqtt_pass + "\"";
  data += ",\"root_topic_subscribe\":\"" + root_topic_subscribe + "\"";
  data += ",\"root_topic_publish\":\"" + root_topic_publish + "\"";
  data += ",\"uroot\":\"" + uroot + "\"";
  data += ",\"usrApn\":\"" + usrApn + "\"";
  data += ",\"pasApn\":\"" + pasApn + "\"";
  data += ",\"apn\":\"" + apn + "\"";
  for (size_t i = 0; i < 6; i++)
  {
    data += ",\"s" + String(i + 1) + "\":" + sensorS[i];
  }

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
  ledBlink(255, 0, 0);
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
String printDigits(int digits)
{

  String g = ":";
  if (digits < 10)
    g += "0";
  g += digits;
  return g;
}
String digitalClockDisplay()
{
  String r = String(hour()) + printDigits(minute()) + printDigits(second()) + " " +
             String(day()) + "/" + String(month()) + "/" + String(year());

  return r;

  Sprint(' ');
  Sprint(day());

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

String showData()
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
  String jsonData = "{";
  jsonData += "\"id\":";
  jsonData += idDevice;
  jsonData += ",\"x\":";
  jsonData += ang_x;
  jsonData += ",\"y\": ";
  jsonData += ang_y;

  jsonData += ",\"bat\":";
  jsonData += readMeasure(adBatery);
  jsonData += ",\"panel\":";
  jsonData += readMeasure(adPanel);
  jsonData += ",\"vh400\":";
  jsonData += (vh400((readMeasure(ADC_S1))), 4);
  jsonData += ",\"therm200\": ";
  jsonData += (therm200((readMeasure(ADC_S2))), 4);
  jsonData += ",\"date\":\"";
  jsonData += digitalClockDisplay();
  jsonData += "\"}";
  return jsonData;
}

void sendData(){
if (client.connected())
      {
        String g = showData();
        Sprintln(g);
        g.toCharArray(msg, 256);
        
        client.publish(String(uroot + "/" + root_topic_publish).c_str(),msg );
      

      }


}
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    delay(5000);
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
void callback(char *topic, byte *payload, unsigned int length)
{
  String incoming = "";
  Sprint("Mensaje recibido desde -> ");
  Sprint(topic);
  Sprintln("");
  for (int i = 0; i < length; i++)
  {
    incoming += (char)payload[i];
  }
  incoming.trim();
  if (incoming.indexOf("tSleep1") != -1 )
  {
    incoming = incoming.substring(incoming.indexOf(":")+1,incoming.indexOf("}"));
    Sprintln("tSleep1 " + incoming);
    tSleep1 = incoming.toInt();
    dataSaved.tSleep1 = tSleep1;

  }
  else if (incoming.indexOf("broker") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("broker " + incoming);
    strncpy(dataSaved.mqtt_server, incoming.c_str(), sizeof(dataSaved.mqtt_server));

  }

  else if (incoming.indexOf("mqtt_port") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("mqtt_port " + incoming);
    mqtt_port = incoming.toInt();
    dataSaved.mqtt_port = mqtt_port;

  }
  else if (incoming.indexOf("uServer") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("uServer " + incoming);
    strncpy(dataSaved.mqtt_user, incoming.c_str(), sizeof(dataSaved.mqtt_user));

  }
  else if (incoming.indexOf("uPass") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("uPass " + incoming);
    strncpy(dataSaved.mqtt_pass, incoming.c_str(), sizeof(dataSaved.mqtt_pass));

  }
  else if (incoming.indexOf("idDevice") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("idDevice " + incoming);
    idDevice = incoming.toInt();
    dataSaved.idDevice = idDevice;

  }
  else if (incoming.indexOf("rootSubs") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("rootSubs " + incoming);
    strncpy(dataSaved.root_topic_subscribe, incoming.c_str(), sizeof(dataSaved.root_topic_subscribe));

  }
  else if (incoming.indexOf("rootPublish") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("rootPublish " + incoming);
    strncpy(dataSaved.root_topic_publish, incoming.c_str(), sizeof(dataSaved.root_topic_publish));

  }
  else if (incoming.indexOf("root") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("root " + incoming);
    strncpy(dataSaved.uroot, incoming.c_str(), sizeof(dataSaved.uroot));

  }
  else if (incoming.indexOf("usrApn") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("usrApn " + incoming);
    strncpy(dataSaved.usrApn, incoming.c_str(), sizeof(dataSaved.usrApn));

  }
  else if (incoming.indexOf("pasApn") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("pasApn " + incoming);
    strncpy(dataSaved.pasApn, incoming.c_str(), sizeof(dataSaved.pasApn));

  }
  else if (incoming.indexOf("apn") != -1)
  {
    incoming = incoming.substring(incoming.indexOf(":")+2,incoming.indexOf("}")-1);
    Sprintln("apn " + incoming);
    strncpy(dataSaved.apn, incoming.c_str(), sizeof(dataSaved.apn));

  }else
  {
    Sprintln("data: " + incoming);
    
  }
  
  // EEPROM.put(0, dataSaved);
  // EEPROM.commit();

}
void reconnect()
{

  int  time_out=3;
  while (!client.connected() && time_out != 0)
  {
    Serial.print("Intentando conexión Mqtt...");
    // Creamos un cliente ID
    String clientId = "IOTICOS_H_W_";
    clientId += String(random(0xffff), HEX);
    // Intentamos conectar
    if (client.connect(clientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str()))
    {
      Sprintln("Conectado!");
      // Nos suscribimos
      if (client.subscribe(String(uroot + "/" + root_topic_subscribe).c_str()))
      {
        Sprintln("Suscripcion ok");
      }
      else
      {
        Sprintln("fallo Suscripciión");
      }
    }
    else
    {
      Sprint("falló :( con error -> ");
      Sprint(client.state());
      Sprintln(" Intentamos de nuevo en 5 segundos");
      delay(5000);
      time_out--;
    }
  }
  if(time_out == 0) { 
      Sprintln("Error Conexion guardando data");
       esp_deep_sleep_start();
  }
}
void modem_reset()
{
  Sprintln("Modem hardware reset");
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, LOW);
  delay(260); //Treset 252ms
  digitalWrite(MODEM_RST, HIGH);
  delay(4000); //Modem takes longer to get ready and reply after this kind of reset vs power on

  //modem.factoryDefault();
  //modem.restart(); //this results in +CGREG: 0,0
}

void modem_on()
{
  // Set-up modem  power pin
  pinMode(MODEM_PWKEY, OUTPUT);
  digitalWrite(MODEM_PWKEY, HIGH);
  delay(10);
  digitalWrite(MODEM_PWKEY, LOW);
  delay(1010); //Ton 1sec
  digitalWrite(MODEM_PWKEY, HIGH);

  //wait_till_ready();
  Sprintln("Waiting till modem ready...");
  delay(4510); //Ton uart 4.5sec but seems to need ~7sec after hard (button) reset
               //On soft-reset serial replies immediately.
}

void modem_off()
{
  //if you turn modem off while activating the fancy sleep modes it takes ~20sec, else its immediate
  Sprintln("Going to sleep now with modem turned off");
  //modem.gprsDisconnect();
  //modem.radioOff();
  modem.sleepEnable(false); // required in case sleep was activated and will apply after reboot
  modem.poweroff();
}
void modem_sleep() // will have an effect after reboot and will replace normal power down
{
  Sprintln("Going to sleep now with modem in power save mode");
  // needs reboot to activa and takes ~20sec to sleep
 // modem.PSM_mode();    //if network supports will enter a low power sleep PCM (9uA)
 // modem.eDRX_mode14(); // https://github.com/botletics/SIM7000-LTE-Shield/wiki/Current-Consumption#e-drx-mode
  modem.sleepEnable(); //will sleep (1.7mA), needs DTR or PWRKEY to wake
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, HIGH);
}

void modem_wake()
{
  Sprintln("Wake up modem from sleep");
  // DTR low to wake serial
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, LOW);
  delay(50);
  //wait_till_ready();
}
void shutdown()
{

  //modem_sleep();
  modem_off();

}
void setup(void)
{

  Serial.begin(9600);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNEES);
  leds[0] = CRGB::Black;
  FastLED.show();
  pinMode(23, INPUT);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_23, 1); //1 = High, 0 = Low
  Sprint("Inicio\n");
  print_wakeup_reason();

  if (!EEPROM.begin(512))
  {
    Sprintln("failed to initialise EEPROM");
    delay(1000000);
  }

  if (digitalRead(23))
  {
    int timeout=15;
    while (digitalRead(RESET_15SEG) && timeout )
    {
      leds[0] = CRGB::Red;
      FastLED.show();
      delay(500);
      leds[0] = CRGB::Black;
      FastLED.show();
      delay(500);
      timeout--;
    }
    if(timeout==0){
      Sprint("Reinicio\n");
    for (int i = 0; i < 512; i++)
    {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();

    ledBlink(255, 0, 0);
  
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(2000);
    ESP.restart();
    }
    
  }

  Serial.setDebugOutput(true);
  pinMode(adPanel, INPUT);
  pinMode(adBatery, INPUT);
  pinMode(ADC_S1, INPUT);
  pinMode(RESET_15SEG, INPUT);
  ++bootCount;
  Sprintln("Boot number: " + String(bootCount));
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection())
    Sprintln("mpu iniciado correctamente");

  RTC.begin();
  //SPIFFS.format();
  setSyncProvider(RTC.get);
  EEPROM.get(0, dataSaved);
  ssdit = dataSaved.ssdit;
  passwordt = dataSaved.passwordt;
  ip = dataSaved.ip;
  gateway = dataSaved.gateway;
  subnet = dataSaved.subnet;
  tSleep1 = dataSaved.tSleep1;
  mqtt_port = dataSaved.mqtt_port;
  mqtt_server = dataSaved.mqtt_server;
  mqtt_user = dataSaved.mqtt_user;
  mqtt_pass = dataSaved.mqtt_pass;
  uroot = dataSaved.uroot;
  root_topic_subscribe = dataSaved.root_topic_subscribe;
  root_topic_publish = dataSaved.root_topic_publish;
  idDevice = dataSaved.idDevice;
  usrApn = dataSaved.usrApn;
  pasApn = dataSaved.pasApn;
  apn = dataSaved.apn;
  memcpy(sensorS, dataSaved.sensorR, 6);
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
  Sprint(tSleep1);
  Sprint(" ");
  Sprint(mqtt_server);
  Sprint(" ");
  Sprint(mqtt_port);
  Sprint(" ");
  Sprint(mqtt_user);
  Sprint(" ");
  Sprint(mqtt_pass);
  Sprint(" ");
  Sprint(uroot);
  Sprint(" ");
  Sprint(root_topic_subscribe);
  Sprint(" ");
  Sprint(root_topic_publish);
  Sprint(" ");
  Sprint(idDevice);
  Sprint(" ");
  Sprint(usrApn);
  Sprint(" ");
  Sprint(pasApn);
  Sprint(" ");
  Sprint(apn);
  for (size_t i = 0; i < 6; i++)
  {
    Sprint(" ");
    Sprint(sensorS[i]);
  }
  Sprintln(" ");

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
    for (int i = 0; i < 100; i++)
      mpu_loop();
    String g = showData();
    Sprintln(g);
    esp_sleep_enable_timer_wakeup(tSleep1 * uS_TO_S_FACTOR);

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

      Sprintln("");
      Sprint("Connected! IP address: ");
      Sprintln(WiFi.localIP());
      client.setServer(mqtt_server.c_str(), mqtt_port);
      client.setCallback(callback);
      while (!client.connected())
      {
        reconnect();
      }
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
    
  

      if (client.connected())
      {
     
        g.toCharArray(msg, 256);
        client.publish(String(uroot + "/" + root_topic_publish).c_str(), msg);
      }
      timeout = 10;
      while (timeout)
      {
        client.loop();
        --timeout;
        delay(200);
      }

      leds[0] = CRGB::Black;
      FastLED.show();
      Sprintln("Setup ESP32 to sleep for every " + String(tSleep1) + " Seconds");
      Sprintln("A Dormir");
      //esp_deep_sleep_start();
    }
    else if (apn.length() != 0)
    {
      modem_on();
      SerialAT.begin(9600, SERIAL_8N1, MODEM_TX, MODEM_RX); //reversing them
      String modemInfo = modem.getModemInfo();
      Sprint(F("Modem: "));
      Sprintln(modemInfo);

      Sprintln("configuring GSM mode"); // AUTO or GSM ONLY

      modem.setPreferredMode(13); //2 Auto // 13 GSM only // 38 LTE only

      Serial.print(F("Waiting for network..."));
      if (!modem.waitForNetwork(60000L))
      {
        Sprintln(" fail");
        modem_reset();
        shutdown();
      }
      Sprintln(" OK");
      Sprint("Signal quality:");
      Sprintln(modem.getSignalQuality());
      Sprint(F("Connecting to "));
      Sprint(apn);
      if (!modem.gprsConnect(apn.c_str(), usrApn.c_str(), pasApn.c_str()))
      {
        Sprintln(" failed apn connect");
        modem_reset();
        shutdown();
      }
      if (modem.isGprsConnected()) {
       Sprintln("GPRS connected");
     }
      client.setServer(mqtt_server.c_str(), mqtt_port);
      client.setCallback(callback);
      // mqtt.setServer(broker, 1883);
      // mqtt.setCallback(mqttCallback);
      modemConnected = true;
      Sprintln(" GSM OK");

      if (true)
      {
        
        Sprint("Connecting to: ");
        Sprint(mqtt_server);
        Sprint(" with token ");
        Sprintln(uroot);
        // if (!tb.connect(THINGSBOARD_SERVER, TOKEN))
        // {
        //   Serial.println("Failed to connect");
        //   modem_reset();
        //   shutdown();
        // }
      } 
      shutdown();
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
      //esp_deep_sleep_start();
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
    Alarm.timerOnce(0, 1, 0, restartT);

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
  server.on("/datosInit", handleinit);
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
  Alarm.timerRepeat(0, 0, 3, sendData);
}


void loop(void)
{

  if (WiFi.getMode() != 2)
  {
    while (!client.connected()) {
         reconnect();
      }
      client.loop();
    }
  
  //  
  
   
  mpu_loop();
  server.handleClient();
  Alarm.delay(0);
}