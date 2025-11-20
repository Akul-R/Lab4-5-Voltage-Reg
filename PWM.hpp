#ifndef PWM_HPP
#define PWM_HPP

#include <avr/io.h>

class PWM_Controller{
private:
    int prescalar = 8;
    int freq = 1000;
    double duty_cycle = 50; //expressed as a %

public:
    void init(){
        DDRD |= _BV(PD5);
        ICR1 = 1499;
        
        //using timer 1, fast pwm
	    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);

        //non inverting pwm on OC1A
        TCCR1A = _BV(COM1A1) | _BV(WGM11);

        OCR1A = ICR1 * (duty_cycle/100);
    }

    void set_freq(int nfreq){
        freq = nfreq;
    }

    void set_duty(double nduty){
        duty_cycle = nduty;
        OCR1A = ICR1 * (duty_cycle/100);
    }

    void set_pwm(unsigned int value){
        //method to set the value of ICR1 directly without calculating duty cycle
        if(value > ICR1){
            value = ICR1;
        }

        OCR1A = value;
        duty_cycle = (value/1499)*100; //calculating new duty cycle
    }

};

#endif