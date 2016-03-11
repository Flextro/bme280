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

/*
 * Redesigned by Christian Tamburilla for use in (MOA) @ BrunoAir
 * I2C functionality removed & code reduced to barebones
 */


#ifndef _BME280_H_
#define _BME280_H_

#include "Arduino.h"

#include <Sensor.h>
#include <Wire.h>

/*
 * REGISTERS
 */
enum
{
  //Temperature
  BME280_REGISTER_DIG_T1              = 0x88,
  BME280_REGISTER_DIG_T2              = 0x8A,
  BME280_REGISTER_DIG_T3              = 0x8C,
  
  //Pressure
  BME280_REGISTER_DIG_P1              = 0x8E,
  BME280_REGISTER_DIG_P2              = 0x90,
  BME280_REGISTER_DIG_P3              = 0x92,
  BME280_REGISTER_DIG_P4              = 0x94,
  BME280_REGISTER_DIG_P5              = 0x96,
  BME280_REGISTER_DIG_P6              = 0x98,
  BME280_REGISTER_DIG_P7              = 0x9A,
  BME280_REGISTER_DIG_P8              = 0x9C,
  BME280_REGISTER_DIG_P9              = 0x9E,
  
  //Humidity
  BME280_REGISTER_DIG_H1              = 0xA1,
  BME280_REGISTER_DIG_H2              = 0xE1,
  BME280_REGISTER_DIG_H3              = 0xE3,
  BME280_REGISTER_DIG_H4              = 0xE4,
  BME280_REGISTER_DIG_H5              = 0xE5,
  BME280_REGISTER_DIG_H6              = 0xE7,
  
  BME280_REGISTER_CHIPID             = 0xD0,
  BME280_REGISTER_VERSION            = 0xD1,
  BME280_REGISTER_SOFTRESET          = 0xE0,
  
  BME280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0
  
  BME280_REGISTER_CONTROLHUMID       = 0xF2,
  BME280_REGISTER_CONTROL            = 0xF4,
  BME280_REGISTER_CONFIG             = 0xF5,
  BME280_REGISTER_PRESSUREDATA       = 0xF7,
  BME280_REGISTER_TEMPDATA           = 0xFA,
  BME280_REGISTER_HUMIDDATA          = 0xFD,
};

/*
 * CALIBRATION DATA
 */
typedef struct
{
  //Temperature Compensation
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  
  //Pressure Compensation
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
  
  //Humidity Compensation
  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
} bme280_calib_data;


class BME280
{
public:
  BME280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);
  

  bool  init();
  
  float readTemperature(void);
  float readPressure(void);
  float readHumidity(void);
  
  /*
   * Calculates the altitude (in meters) from the specified atmospheric
   * pressure (in hPa), and sea-level pressure (in hPa).
   *
   * @param  seaLevel      Sea-level pressure in hPa
   *
   * Equation taken from BMP180 datasheet (page 16):
   * http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
   * 
   * Note that using the equation from wikipedia can give bad results
   * at high altitude.  See this thread for more information:
   * http://forums.adafruit.com/viewtopic.php?f=22&t=58064
   */
  float readAltitude(float seaLevel);
  
  
  /**
   *  Getters
   *
   *  @return performs read methods and retrieves metrics
   */
  uint8_t getTemp();
  uint8_t getPressure();
  uint8_t getAltitude();
  uint8_t getHumidity();
  
private:
  
  //SPI values defined in the sketch, set in the constructor
  uint8_t _cs, _mosi, _miso, _sck;
  
  uint8_t _temp;
  uint8_t _pressure;
  uint8_t _altitude;
  uint8_t _humidity;
  
  /*
   *  Handles bitshifting
   *
   *  @param x is the data to transfer / shift
   */
  uint8_t spixfer(uint8_t x);
  
  /*
   * @brief  Writes an 8 bit value over SPI
   */
  void      write8(byte reg, byte value);
  /*
   * @brief  Reads an 8 bit value over SPI
   */
  uint8_t   read8(byte reg);
  /*
   * @brief  Reads a 16 bit value over SPI
   */
  uint16_t  read16(byte reg);
  /*
   * @brief  Reads a 24 bit value over SPI
   */
  uint32_t  read24(byte reg);
  /*
   * @brief  Reads a signed 16 bit value over SPI
   */
  int16_t   readS16(byte reg);
  
  /*
   * @brief  Reads for 16 bit little endian notation over SPI
   */
  uint16_t  read16_LE(byte reg);
  int16_t   readS16_LE(byte reg);
  
  /*
   * @brief  Reads the factory-set coefficients
   */
  void readCoefficients(void);
  
  
  //Adds calibration data in the readTemperature method, also accessed in readPressure
  int32_t t_fine;
  
  //Calibration struct data value
  bme280_calib_data _bme280_calib;
  
  
  
};

#endif
