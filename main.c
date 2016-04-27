/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <uart.h>
#include <i2c_master.h>
#include <bme280.h>


uint16_t read8(uint8_t reg)
{
  uint16_t val;
  
  if (i2c_start(BME280_ADDRESS<<1 | I2C_WRITE) == 0)
  {
    i2c_write((uint8_t)reg);
    
    i2c_stop();
    
    if (i2c_start(BME280_ADDRESS<<1 | I2C_READ) == 0)
    {
      val |= ((uint16_t)i2c_read_nack());
      
      i2c_stop();
      
      return val;
    }
  }
}


uint16_t read16(uint8_t reg)
{
  uint16_t val;
  
  if (i2c_start(BME280_ADDRESS<<1 | I2C_WRITE) == 0)
  {
    i2c_write((uint8_t)reg);
    
    i2c_stop();
    
    if (i2c_start(BME280_ADDRESS<<1 | I2C_READ) == 0)
    {
      val = ((uint16_t)i2c_read_ack());
      val <<= 8;
      val |= ((uint16_t)i2c_read_nack());
      
      i2c_stop();
      
      return val;
    }
  }
}

uint16_t read16_LE(uint8_t reg)
{
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
  
}

int16_t readS16(uint8_t reg)
{
  return (int16_t)read16(reg);
  
}

int16_t readS16_LE(uint8_t reg)
{
  return (int16_t)read16_LE(reg);
  
}

uint32_t read24(uint8_t reg)
{
  uint32_t val;

  if (i2c_start(BME280_ADDRESS<<1 | I2C_WRITE) == 0)
  {
    i2c_write(reg);

    i2c_stop();

    if (i2c_start(BME280_ADDRESS<<1 | I2C_READ) == 0)
    {
      val = ((uint16_t)i2c_read_ack());
      val <<= 8;
      val |= ((uint16_t)i2c_read_ack());
      val <<= 8;
      val |= ((uint16_t)i2c_read_nack());
      
      i2c_stop();
      
      return val;
    } else
    {
      uart_puts("Could not connect to sensor");
    }
  }
}

void readCoefficients(void)
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


void init()
{
  
  if (i2c_start(BME280_ADDRESS<<1 | I2C_WRITE) == 0)
  {
    i2c_write(BME280_REGISTER_CONTROLHUMID);
    i2c_write(0x05);
    
    i2c_write(BME280_REGISTER_CONTROL);
    i2c_write(0xB7);
    
    i2c_write(BME280_REGISTER_CHIPID);
    
    readCoefficients();
    
    i2c_stop();
    
    if (i2c_start(BME280_ADDRESS<<1 | I2C_READ) == 0)
    {
      uint8_t val = i2c_read_nack();
      
      uart_putc((char)val);
      
      i2c_stop();
    }
  }
}


uint32_t t_fine; //must be global
uint32_t readTemperature(void)
{
  int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);

  int32_t var1, var2;
  
  adc_T >>= 4;
  
  var1  = ((((adc_T>>3) - ((int32_t)_bme280_calib.dig_T1 <<1))) *
           ((int32_t)_bme280_calib.dig_T2)) >> 11;
  
  var2  = (((((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1)) *
             ((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
           ((int32_t)_bme280_calib.dig_T3)) >> 14;
  
  t_fine = var1 + var2;
  
  float T  = (t_fine * 5 + 128) >> 8;
  
  return T/100;
}

//TODO: readPressure();
//TODO: readHumidity();
//TODO: readAltitude();


int main(void)
{
  init_uart(57600);

  i2c_init();

  init();

    for(;;)
    {
      uint32_t foo = readTemperature();
      
      
      char buf[10];
      itoa( foo, buf, 10);
      uart_puts(buf);
      
      PORTB = 0xFF;
      _delay_ms(500);
      PORTB = 0x00;
      _delay_ms(1000);
    }
    return 0;   /* never reached */
}
