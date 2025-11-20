#include <avr/io.h>
#include <string.h> 
#include "Serial.hpp"
#include "Functions.hpp" 
#include "PWM.hpp"
#include "ADC.hpp"
#include "PID.hpp"

Serial_interface uart;
PWM_Controller pwm;
ADC_Controller adc;
PID_Controller pid;

int main() {
    DDRB |= _BV(PINB7);

    uart.init_uart();

    char buffer[MAX_BUFFER_SIZE];
    while (true) {
        // 2. Receive a character
        PORTB |= _BV(PINB7);
        
        uart.receive_ln(buffer);

        Command_format cmd_token = tokenise(buffer);

        int cmd_found = 0;
        for(int i = 0; i < 2; ++i){
            if(strcmp(cmd_token.command, func_lookup[i].command_name) == 0){
                //if they match, run appropriate function
                cmd_found = 1;
                func_lookup[i].func(cmd_token.arguments);
                break;
            }
        }

        if(cmd_found == 0){
            cmd_unknown(buffer);
        }
    }
    
    return 0;
}