#include <LiquidCrystal_I2C.h>

#include <DHT.h>


// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

//#include <Wire.h>

#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <OneWire.h> // ds18d20 
#include <Adafruit_BMP085.h> //библиотека для датчика атмосферного давления и температуры GY-68 BMP085 и нашего GY-68 BMP180
#define DHTPIN 2     // what digital pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT21   // DHT 22  (AM2302), AM2321

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp; //GY-68 BMP180
LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display
OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
boolean flag = true; // button in future?
String sSensorFlag = "dht";
int pin_soli = A0;
const int relayPin = 11; //the base of the transistor attach to

void setup() {
  //lcd.begin(9600);
  dht.begin();
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  digitalWrite(relayPin, LOW);//turn off the pump
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
 
}

void loop() {
  byte i; //ниже все 6 строк это к датчику ds18d20
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  int mositure_soli;
  boolean relay;

  
  // Wait a few seconds between measurements.
  delay(5000);
  lcd.clear();
  
  //if (flag == true) // input internal data
  if (sSensorFlag == "dht") //
  {
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      float h = dht.readHumidity();
      // Read temperature as Celsius (the default)
      float t = dht.readTemperature();
      lcd.setCursor(1,0); // 1 symbol 1 string
      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t)) {
        lcd.print("Failed to read from DHT sensor!");
        return;
      }
      lcd.print("H= ");
      lcd.print(h);
      lcd.print("%");
      lcd.setCursor(1,1); // 1 symbol 2 string
      lcd.print("T= ");
      lcd.print(t);
      lcd.print("*C");
      //flag = false;
      sSensorFlag = "ds18d20";
  }
  else if (sSensorFlag == "ds18d20") 
  {
      lcd.setCursor(1,0); // 1 symbol 1 string
      
      
      if ( !ds.search(addr)) {
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250);
      return;
    }
    
    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
  
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();
   
    // the first ROM byte indicates which chip
    switch (addr[0]) {
      case 0x10:
        Serial.println("  Chip = DS18S20");  // or old DS1820
        type_s = 1;
        break;
      case 0x28:
        Serial.println("  Chip = DS18B20");
        type_s = 0;
        break;
      case 0x22:
        Serial.println("  Chip = DS1822");
        type_s = 0;
        break;
      default:
        Serial.println("Device is not a DS18x20 family device.");
        return;
    } 
  
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
    
    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad
  
    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
  
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    lcd.print("T= ");
    lcd.print(celsius);
    lcd.print("*C");
    //flag = true;
    sSensorFlag = "gy68";
    
  }
  else if (sSensorFlag == "gy68")
  {
    lcd.setCursor(1,0); // 1 symbol 1 string
    bmp.readTemperature();
    lcd.print("P = ");
    lcd.print(bmp.readSealevelPressure());
    lcd.print(" Pa");
    lcd.setCursor(1,1); // 1 symbol 2 string
  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    lcd.print("Altitude=");
    lcd.print(bmp.readAltitude(101500));
    lcd.print("m");

    sSensorFlag = "soli";
  }

  else if (sSensorFlag == "soli")
  { 
    relay = true;
    while (relay == true)
    {
      lcd.clear();
      mositure_soli= analogRead(pin_soli);
      lcd.print("Mositure : ");
      lcd.print(mositure_soli);
      mositure_soli = map(mositure_soli,795,366,0,100);
      lcd.setCursor(1,1); // 1 symbol 2 string
      lcd.print("Mositure : ");
      lcd.print(mositure_soli);
      lcd.print("%");
      if (mositure_soli < 87)
        {
          relay = true;
          digitalWrite(relayPin, HIGH);//turn on the pump
          delay(3000);
          digitalWrite(relayPin, LOW);//turn off the pump
          delay(10000);
        }
      else
        {
          relay = false;
          digitalWrite(relayPin, LOW);//turn off the pump
          lcd.clear();
          lcd.print("Watering is finished");
        }
      }
    sSensorFlag = "dht";
  }
  
}
