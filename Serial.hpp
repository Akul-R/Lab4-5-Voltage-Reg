#ifndef UART_HPP
#define UART_HPP

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#ifndef F_CPU
#define F_CPU 12000000UL
#endif

#define BAUD_RATE 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD_RATE) - 1)

#define MAX_BUFFER_SIZE 32 //maximum input size is 32 characters long


class Serial_interface{
public:
    void init_uart(){
        UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
        UBRR0L = (uint8_t)UBRR_VALUE;

        UCSR0B = _BV(RXEN0) | _BV(TXEN0);
        UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
    }

    void transmit(char data){
        while(!(UCSR0A & _BV(UDRE0)));
        UDR0 = data;
    }

    void transmit_ln(const char* line){ //transmits an entire line
        while(*line != '\0'){
            transmit(*line++);
        }
        transmit('\r');
        transmit('\n');
    }

    char receive(){
        while(!(UCSR0A & _BV(RXC0)));
        return UDR0;
    }

    void receive_ln(char* buffer){
        //we store the characters in buffer
        memset(buffer, '\0', MAX_BUFFER_SIZE);
        uint8_t count = 0;
        char received_letter;

        while(count < MAX_BUFFER_SIZE){
            received_letter = receive();

            //checks if received letter is the end of the line
            if(received_letter == '\r' || received_letter == '\n'){
                transmit('\r');
                transmit('\n');
                break;
            }

            buffer[count++] = received_letter;
        }

        buffer[count] = '\0';
    }
};

#endif // UART_HPP