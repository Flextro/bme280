/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */


#include <bme280.h>
#include <ftoa.h>


int main(void)
{
  init_uart(57600);
  
  i2c_init();
  
  init();
  
  for(;;)
  {
    
    float a = bme280_readTemperature();
    float b = bme280_readPressure();
    float c = bme280_readHumidity();
    float d = bme280_readAltitude(1013.25);
    
    char buf[10];
    uart_puts("Temperature: ");
    ftoa( buf, a, 2);
    uart_puts(buf);
    uart_puts("C ");
  
    uart_puts("Pressure: ");
    ftoa( buf, b, 2);
    uart_puts(buf);
    uart_puts(" ");
    
    uart_puts("Humidity: ");
    ftoa( buf, c, 2);
    uart_puts(buf);
    uart_puts("% ");
    
    uart_puts("Altitude: ");
    ftoa( buf, d, 2);
    uart_puts(buf);
    uart_puts("\n\r");
    
    PORTB = 0xFF;
    _delay_ms(500);
    PORTB = 0x00;
    _delay_ms(1000);
  }
  return 0;   /* never reached */
}

