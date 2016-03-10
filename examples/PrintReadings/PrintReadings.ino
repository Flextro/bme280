/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
//Modified by Christian Tamburilla for use in (MOA) @ BrunoAir
//I2C functionality removed & code reduced to barebones

#include <SPI.h>
#include <Sensor.h>
#include <BME280.h>

//Standard SPI pin positions
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

BME280 sensor(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

void setup() {
  Serial.begin(9600);
  sensor.init();
}

void loop() {
    Serial.print("TEMP: ");
    Serial.print(sensor.getTemp());
    Serial.println(" °F");
    Serial.print("PRESSURE: ");
    Serial.print(sensor.getPressure());
    Serial.println(" hPa");
    Serial.print("Approx. Altitude: ");
    Serial.print(sensor.getAltitude());
    Serial.println(" m");
    Serial.print("Humidity: ");
    Serial.print(sensor.getHumidity());
    Serial.println(" %");
    Serial.println();
    
    delay(2000);
}
