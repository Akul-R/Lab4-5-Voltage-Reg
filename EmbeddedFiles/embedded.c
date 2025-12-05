#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "debug.h" 
#define MAX_BUFFER_SIZE 32
#define I_MAX 3000
#define DT 0.5


volatile int overflows = 0;
volatile uint16_t adc_val = 0;

uint16_t adc_measure(void);
void adjust(void);

int time = 0;

//system states
bool start = false;
bool pid_enabled = false;

//PID terms
int target = 750; //to test, roughly 1.7v
double prev_error = 0.0;
double accumulated_error = 0.0;
double kp = 0.8;
double ki = 0.3;
double kd = 0.0;

ISR(TIMER0_OVF_vect){
	//interupt only fires when timer 0 overflows
	//every 25 overflows, approximately 500ms should pass (since each overflow is like 20ms or smth)
	if(overflows == 25){
		if(start == true){
			adc_val = adc_measure();
			printf("#%d,%d#", time, (int)adc_val);
			if(pid_enabled == true){
				adjust();
			}
			overflows = 0;
			++time;
		}
	}
	else{
		++overflows;
	}
}


void init_timer0(){
	//this timer will control how often the ADC value is measured. Taken roughly every 500mS
	TCCR0A = 0;
	TCCR0B = 0; 
	TCNT0 = 0;

	//Enable timer overflow interupt
	TIMSK0 |= _BV(TOIE0);
	//applying a 1024 prescalar
	TCCR0B |= _BV(CS02) | _BV(CS00);
}

void init_pwm(){
	DDRD |= _BV(PD5);
	ICR1 = 1499;
	
	//using timer 1, fast pwm
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);

	//non inverting pwm on OC1A
	OCR1A = ICR1 * (50.0/100.0);
}

void set_pwm(unsigned int value){
	if(value > ICR1){
		value = ICR1;
    }
	OCR1A = value;
}

void init_adc(){
	ADMUX = _BV(REFS0);
    ADCSRA |= _BV(ADEN) | _BV(ADPS1) | _BV(ADPS2);
}


uint16_t adc_measure(){
	ADCSRA |= _BV(ADSC); //start conversion
	cli();
	while(ADCSRA & (_BV(ADSC)));
	sei();
	return ADC;
}

void adjust(){
	//this function hurts my head
	int error = (int)(target-adc_val); //error, im working in "adc counts" because i tried working in voltages and that didnt work
	
	double p_term = kp * (double)error;

	accumulated_error += ((double)(prev_error+error)/2)*DT;
	//windup protection is simply just clamping it, could implement something fancier but i would have to research a bit more
	if(accumulated_error > I_MAX){accumulated_error = I_MAX;}
	if(accumulated_error < -I_MAX){accumulated_error = -I_MAX;}
	double i_term = ki * accumulated_error;

	double d_term = kd * ((error - prev_error)/DT);

	double output = p_term + i_term + d_term; //summing up terms to get output
	prev_error = error;
	
	output = (output/1023)*1499; //scale output to match range for PWM
	
	if(output > 1499){output = 1499;} //1499 is the max values for OCR1A
	if(output < 0){output = 0;} //OCR1A goes from 0 to 1499, these just ensure the output stays in that range
	set_pwm(output);
}

int main(){
	char input[MAX_BUFFER_SIZE];
	char command[10];
	int p = 0;
	int i = 0;
	int d = 0; 
	int t = 0;
	
	init_debug_uart0();
	init_timer0();
	init_pwm();
	init_adc();
	sei();
	while(1){
		if (fgets(input, MAX_BUFFER_SIZE, stdin) == NULL) {
			continue;
		}
		
		//words will tell us how many words are in the input (shocking i know)
		int words = sscanf(input, "%d %d %d %d", &p, &i, &d, &t);
		
		//its 2am, im starting to see things. I need this to work please please please
		if(words == 4){
			//everything is divided by 1000 as on the host side, the numbers are multiplied by 1000 and sent as integers
			//this is because apparently you cant send floats over serial. Took me way too long to figure that out
			//although we may lose some precision, its better than writing a function to convert strings into floats
			kp = (double)(p/1000.0);
			ki = (double)(i/1000.0);
			kd = (double)(d/1000.0);
			target = (int)(t);
			printf("UPD CONST\r\n");
			continue;
		}
		
		//check each command word.
		sscanf(input, "%s", command);
		if(strcmp("START", command) == 0){
			//starts system
			start = true;
			printf("STARTED\r\n");
		}

		else if(strcmp("STOP", command) == 0){
			//stops system
			start = false;
			time = 0;
			printf("STOPPED\r\n");
		}

		else if(strcmp("EN", command) == 0){
			//enables PID controller
			pid_enabled = true;
			printf("PID ENABLED\r\n");
		}

		else if(strcmp("DIS", command) == 0){
			//disables PID controller
			pid_enabled = false;
			printf("PID DISABLED\r\n");
		}
		//there probably are better ways of doing this that putting a million if statements
		else{
			printf("UNKNOWN COMMAND");
		}
	}
}