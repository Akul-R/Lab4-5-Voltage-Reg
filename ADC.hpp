#ifndef ADC_HPP
#define ADC_HPP

#include <avr\io.h>

class ADC_Controller{
public:
    void init(){
        ADMUX = _BV(REFS0);
        ADCSRA |= _BV(ADEN) | _BV(ADPS1) | _BV(ADPS2);
    }

    uint16_t measure(){
        ADCSRA |= _BV(ADSC); //start conversion
	
	    while(ADCSRA & (_BV(ADSC)));
        return ADC;
    }
};


#endif