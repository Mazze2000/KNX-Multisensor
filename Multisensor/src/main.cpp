#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_HTU21DF.h"
#include <KnxTpUart.h>
#include <MHZ19.h>
#include <Bounce2.h>
#include <SoftwareSerial.h>
#include <LightDependentResistor.h>
#include <Adafruit_NeoPixel.h>

#define PHYSICAL_ADDRESS "1.1.25"

#define CO2_REFRESH_INTERVAL 10000
#define CO2_PPM_GROUP_ADDRESS "10/1/1"

#define HTU_TEMP_GROUP_ADDRESS "10/1/2"
#define HTU_HUM_GROUP_ADDRESS "10/1/3"

#define LDR_GROUP_ADDRESS "10/1/4"

#define LED1_GROUP_ADDRESS "10/1/11"
#define LED2_GROUP_ADDRESS "10/1/12"
#define LED3_GROUP_ADDRESS "10/1/13"

#define BUTTON_GROUP_ADDRESS "10/1/3"

#define LED1 10 // Green
#define LED2 16 // Yellow
#define LED3 14 // Red

//Pixel LEDs
#define PIN D3
#define NUMPIXELS 3
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);
long ledStopTime = 0;
boolean ledsRunning = false;

#define OTHER_RESISTOR 1000 //ohms
#define USED_PIN A0
#define USED_PHOTOCELL LightDependentResistor::GL5516
LightDependentResistor photocell(USED_PIN, OTHER_RESISTOR, USED_PHOTOCELL);

#define BUTTON1 15 // Test Button and Terminal Block

SoftwareSerial mySerialKNX(13, 15); // RX, TX 13 15

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

Bounce debouncer = Bounce();
KnxTpUart knx(&mySerialKNX, PHYSICAL_ADDRESS);
MHZ19 mhz19;

void showLEDs(long showtime, byte red, byte green, byte blue){
  ledStopTime = millis() + showtime;
  if(showtime==0){
    pixels.clear();
    pixels.show();
    ledsRunning = false;
  } else {
    pixels.clear();
    for(int i=0; i<NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(red, green, blue));
    }
    pixels.show();
    ledsRunning = true;
  }
}

float getCo2SensorPpm()
{
  measurement_t m = mhz19.getMeasurement();
  Serial.print("Co2 (MHZ19): ");
  Serial.println(m.co2_ppm);
  return float(m.co2_ppm);
}

float getTempHtu()
{
  float temp = htu.readTemperature();
  Serial.print("Temp (Htu21d): ");
  Serial.println(temp);
  return float(temp);
}

float getHumHtu()
{
  
  float rel_hum = htu.readHumidity();
  Serial.print("Humidity (Htu21d): ");
  Serial.println(rel_hum);
  return float(rel_hum);
}

float getBrigthLDR()
{
  float intensity_in_lux = photocell.getCurrentLux();
  Serial.print("Brigthness (LDR): ");
  Serial.println(intensity_in_lux);
  return intensity_in_lux;
}

void maintainCo2Sensor()
{
  static unsigned long lastRefreshTime = 0;

  if (millis() - lastRefreshTime >= CO2_REFRESH_INTERVAL)
  {
    lastRefreshTime += CO2_REFRESH_INTERVAL;
    knx.groupWrite2ByteFloat(CO2_PPM_GROUP_ADDRESS, getCo2SensorPpm());
    knx.groupWrite2ByteFloat(HTU_TEMP_GROUP_ADDRESS, getTempHtu()); 
    knx.groupWrite2ByteFloat(HTU_HUM_GROUP_ADDRESS, getHumHtu());
    knx.groupWrite2ByteFloat(LDR_GROUP_ADDRESS, getBrigthLDR());

    showLEDs(500,0,0,150);
  }
}

void maintainKnxSerial()
{
  if (Serial1.available() > 0)
  {
    KnxTpUartSerialEventType eType = knx.serialEvent();
    if (eType == KNX_TELEGRAM)
    {
      KnxTelegram *telegram = knx.getReceivedTelegram();

      String target =
          String(0 + telegram->getTargetMainGroup()) + "/" +
          String(0 + telegram->getTargetMiddleGroup()) + "/" +
          String(0 + telegram->getTargetSubGroup());

      if (telegram->getCommand() == KNX_COMMAND_READ)
      {
        // Answer to Read
        if (target == CO2_PPM_GROUP_ADDRESS)
        {
          knx.groupAnswer2ByteFloat(CO2_PPM_GROUP_ADDRESS, getCo2SensorPpm());
        }
        else if (target == HTU_TEMP_GROUP_ADDRESS)
        {
          knx.groupAnswer2ByteFloat(HTU_TEMP_GROUP_ADDRESS, getTempHtu());
        }
        else if (target == HTU_HUM_GROUP_ADDRESS)
        {
          knx.groupAnswer2ByteFloat(HTU_TEMP_GROUP_ADDRESS, getHumHtu());
        }
      }
      else if (telegram->getCommand() == KNX_COMMAND_WRITE)
      {
        // Write commands
        if (target == LED1_GROUP_ADDRESS)
        {
          digitalWrite(LED1, telegram->getBool() ? HIGH : LOW);
        }
        else if (target == LED2_GROUP_ADDRESS)
        {
          digitalWrite(LED2, telegram->getBool() ? HIGH : LOW);
        }
        else if (target == LED3_GROUP_ADDRESS)
        {
          digitalWrite(LED3, telegram->getBool() ? HIGH : LOW);
        }
      }
    }
  }
}

void ledLoop(){
  if(ledStopTime - millis()<=0 && ledsRunning){
    showLEDs(0,0,0,0);
  }
}

void setup()
{
  Serial.begin(9600);

  if (!htu.begin()) {
    Serial.println("Couldn't find sensor!");
  }

  pixels.begin();
  showLEDs(1000,0,150,0);

  // LED config
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  // Button config
  pinMode(BUTTON1, INPUT_PULLUP);
  debouncer.attach(BUTTON1, INPUT_PULLUP);
  debouncer.interval(25);

  // KNX config
  Serial1.begin(19200, SERIAL_8E1);
  knx.uartReset();
  knx.addListenGroupAddress(CO2_PPM_GROUP_ADDRESS);
  knx.addListenGroupAddress(HTU_TEMP_GROUP_ADDRESS);
  knx.addListenGroupAddress(HTU_HUM_GROUP_ADDRESS);
  knx.addListenGroupAddress(LDR_GROUP_ADDRESS);
  
  knx.addListenGroupAddress(LED1_GROUP_ADDRESS);
  knx.addListenGroupAddress(LED2_GROUP_ADDRESS);
  knx.addListenGroupAddress(LED3_GROUP_ADDRESS);

  // CO2 sensor config
  mhz19.begin(14, 12);
  mhz19.setAutoCalibration(false);
}

void loop()
{
  maintainCo2Sensor();
  maintainKnxSerial();
  ledLoop();

  // Button Event
  debouncer.update();
  if (debouncer.fell())
  {
    knx.groupWriteBool(BUTTON_GROUP_ADDRESS, true);
  }
}