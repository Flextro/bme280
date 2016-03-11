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

#include "Arduino.h"
#include <SPI.h>
#include "BME280.h"

#define SEALEVELPRESSURE_HPA (1013.25)


BME280::BME280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin) : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin) { }


bool BME280::init()
{

  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);

  if (_sck == -1)
  {
    // hardware SPI
    SPI.begin();
  }
  else
  {
    // software SPI
    pinMode(_sck, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
  }

  if (read8(BME280_REGISTER_CHIPID) != 0x60)
    return false;

  readCoefficients();

  //Set before CONTROL_meas (DS 5.4.3)
  write8(BME280_REGISTER_CONTROLHUMID, 0x05); //16x oversampling 

  write8(BME280_REGISTER_CONTROL, 0xB7); // 16x ovesampling, normal mode
  return true;
}

uint8_t BME280::spixfer(uint8_t x)
{
  if (_sck == -1)
    return SPI.transfer(x);

  // software spi
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1<<i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}

void BME280::write8(byte reg, byte value)
{
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg & ~0x80); // write, bit 7 low
    spixfer(value);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
}

uint8_t BME280::read8(byte reg)
{
  uint8_t value;

    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus

  return value;
}

uint16_t BME280::read16(byte reg)
{
  uint16_t value;

    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = (spixfer(0) << 8) | spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus

  return value;
}

uint16_t BME280::read16_LE(byte reg)
{
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);

}

int16_t BME280::readS16(byte reg)
{
  return (int16_t)read16(reg);

}

int16_t BME280::readS16_LE(byte reg)
{
  return (int16_t)read16_LE(reg);

}


uint32_t BME280::read24(byte reg)
{
  uint32_t value;

    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    
    value = spixfer(0);
    value <<= 8;
    value |= spixfer(0);
    value <<= 8;
    value |= spixfer(0);

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus

  return value;
}

void BME280::readCoefficients(void)
{
    _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
    _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
    _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

    _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
    _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
    _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
    _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
    _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
    _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
    _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
    _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
    _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

    _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
    _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
    _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
    _bme280_calib.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
    _bme280_calib.dig_H5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}


float BME280::readTemperature(void)
{
  int32_t var1, var2;

  int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)_bme280_calib.dig_T1 <<1))) *
	   ((int32_t)_bme280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
	   ((int32_t)_bme280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  
  _temp = T/100;
  return _temp;
}


float BME280::readPressure(void)
{
  int64_t var1, var2, p;

  readTemperature(); // must be done first to get t_fine

  int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bme280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bme280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bme280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bme280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bme280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7)<<4);
  
  _pressure = (float)p/256;
  return _pressure;
}


float BME280::readHumidity(void)
{

  readTemperature(); // must be done first to get t_fine

  int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);

  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
		  (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
	       (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
		    (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
		  ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
			     ((int32_t)_bme280_calib.dig_H1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r>>12);
  _humidity = h / 1024.0;
  return  _humidity;
}


float BME280::readAltitude(float seaLevel)
{

  float atmospheric = readPressure() / 100.0F;
  _altitude = 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
  return _altitude;
}

uint8_t BME280::getTemp()
{
  readTemperature();
  return _temp * 1.8 + 32;
}

uint8_t BME280::getPressure()
{
  readPressure();
  return _pressure /100.0F;
}

uint8_t BME280::getAltitude()
{
  readAltitude(SEALEVELPRESSURE_HPA);
  return _altitude;
}

uint8_t BME280::getHumidity()
{
  readHumidity();
  return _humidity;
}