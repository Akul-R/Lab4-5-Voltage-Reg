#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "Serial.hpp"

extern Serial_interface uart;
extern PWM_Controller pwm;
extern ADC_Controller adc;
extern PID_Controller pid;

#endif // GLOBALS_HPP