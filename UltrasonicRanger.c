#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "my_adc_lib.h"
#include "my_uart_lib.h"
#include "SSD1306.h"
#include "i2c.h"
#define TRIG PC5
#define ECHO PC0
#define RANGE_PER_CLOCK 1.098
void uart_init(void);
void timer0_init(void);
void send_to_LED(int);

int main(void)
{
    unsigned int timeToRisingEdge, timeToFallingEdge, pulseWidth;
    float range;

    while (1)
    {
        TCNT0 = 0;
        PORTC |= 1 << TRIG;
        _delay_us(10);
        PORTC &= ~(1 << TRIG);

        while ((PINC & (1 << ECHO)) == 0)
            ;
        rising_edge_clocks = TCNT0;
        while (!(PINC & (1 << ECHO)) == 0)
            ;
        falling_edge_clocks = TCNT0;

        if (timeToFallingEdge > timeToRisingEdge)
        {
            // compute range and send it to 7 segment LED
            pulseWidth = timeToFallingEdge - timeToRisingEdge;
            range = pulseWidth * RANGE_PER_CLOCK; // one way distance to target in cm
            send_to_LED(range);
        }
    }
}

void send_to_LED(int range)
{
    char ledDigits[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67};
    char DIG1, DIG2, DIG3, DIG4;

    DDRD = 0xFF;  // 7 segment pins
    DDRB = 0xFF;  // Digit enable pins
    PORTB = 0xFF; // Disable all the digits initially

    DIG4 = (int)range % 10;
    PORTD = ledDigits[DIG4];
    PORTB = ~(1 << 1);
    _delay_ms(50)

        DIG3 = ((int)range / 10) % 10;
    PORTD = ledDigits[DIG3];
    PORTB = ~(1 << 2);
    _delay_ms(50)

        DIG2 = ((int)range / 100) % 10;
    PORTD = ledDigits[DIG2];
    PORTB = ~(1 << 3);
    _delay_ms(50)

        DIG1 = (int)range / 1000;
    PORTD = ledDigits[DIG1];
    PORTB = ~(1 << 4);
    _delay_ms(50)
}

// Initialize timer0: normal mode (count up), divide clock by 1024
void timer0_init()
{
    TCCR0A = 0; // Timer 1 Normal mode (count up)
    TCCR0B = 5; // Divide clock by 1024
    TCNT0 = 0;  // Start the timer at 0
}

// Send a string, char by char, to UART via uart_send()
// Input is pointer to the string to be sent
void send_string(char *stringAddress)
{
    for (unsigned char i = 0; i < strlen(stringAddress); i++)
        uart_send(stringAddress[i]);
}

// Initialize the UART
void uart_init(void)
{
    UCSR0B = (1 << TXEN0);                  // enable the UART transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // set 8 bit character size
    UBRR0L = 103;                           // set baud rate to 9600 for 16 MHz crystal
}

// Sent a single character to serial monitor via UART
void uart_send(unsigned char ch)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;      // wait til tx data buffer empty
    UDR0 = ch; // write the character to the USART data register
}
