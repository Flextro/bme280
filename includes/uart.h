#ifndef UART_H
#define UART_H

void init_uart(uint16_t baudrate);
void uart_putc(unsigned char c);
void uart_puts(char *s);

#endif // I2C_MASTER_H
