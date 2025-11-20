#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include "Globals.hpp"

void cmd_test(char* arguments); //test function
void cmd_unknown(char* arguments); //function when unknown input

typedef void (*command_function)(char* arguments);
//eg, command of FREQ 50 entered into terminal could set the PWM frequency to 50hz. 

/*
this will hopefully look something like this:
{FREQ, set_freq}
im trying to make a lookup table, its not going well :(
*/
struct Command_entry{
    const char* command_name; //this is what the user enters (eg FREQ 50)
    command_function func;  //this is the function that corresponds to input (eg, FREQ would correspond to set_freq function)
};


const Command_entry func_lookup[] = {
    {"L", cmd_test},
    {NULL, NULL}
};

struct Command_format{
    //eg FREQ 50, FREQ is the command and 50 is the argument
    char* command;
    char* arguments;
};
Command_format tokenise(const char* input){
    //tokenises input to match the format required
    Command_format output;
    const char* delim = " \r\n";
    char* cmd_tkn = strtok(input, delim); //gets command token (eg FREQ)
    char* arg_tkn = strtok(NULL, delim); //gets argument token (eg 50)

    output.command = cmd_tkn;
    output.arguments = arg_tkn;

    return output;
}

void cmd_test(char* arguments){
    uart.transmit_ln("67676767676767");
}

void cmd_unknown(char* arguments){
    uart.transmit_ln("UNKNOWN COMMAND");
    uart.transmit_ln(arguments);
}

#endif
