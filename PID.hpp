#ifndef PID_HPP
#define PID_HPP

#include <avr\io.h>

class PID_Controller {
private:
    //these are the 3 tunable constants
    double Kp; //proportional
    double Ki; //integral
    double Kd; //derivative

    double set_point; //the value the controller is trying to get to
    double accumulated_error = 0; //the accumulated error, used for integral response
    double prev_error = 0; //the previous error term, used in derivative calculation
    double sample_time = 0.0000693; //estimated time between adc samples (probably should change in future)

public:
    void init(double p, double i, double d){
        //initialise with some number (make sure to tune it right)
        Kp = p;
        Ki = i;
        Kd = d;
    }

    void adjust(uint16_t adc_val){
        double measure_v = (adc_val/1023.0)*3.3;
        double error_term = set_point - measure_v; //finding current error term

        double p_term = Kp * error_term;

        accumulated_error += error_term * sample_time;
        double i_term = Ki * accumulated_error;

        double d_term = Kd * ((error_term - prev_error)/sample_time);

        double output = p_term + i_term + d_term; //summing up terms to get output
    }
};

#endif