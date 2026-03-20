#ifndef COMMANDS
#define COMMANDS

// IMPORTANT: Also update the "src/roseybot_control/hardware/include/roseybot_arduino/commands.h" file with any changes!
// define character types for different commands
#define CLEAR_ERROR       'c' // exit the error state
#define GET_TELEM         't' // gets relevant telem info. m1pos,m2pos,m3pos,m4pos,m1speed,m2speed,m3speed,m4speed,m1cur,m2cur,m3cur,m4cur,rc1volt,rc2volt,m1pwm,m2pwm,m3pwm,m4pwm
#define PID               'p' // set PID values in EEPROM memory and reinitialize motors; p,i,d
#define RESET_ENCODERS    'r' // reset encoder positions
#define SET_MOTOR_SPEEDS  'm' // set motor speeds of all 4 motors. arg1 = left motor speed | arg2 = right motor speed
#define SET_MOTOR_SPEED   'd' // sets motor speed of 1 motor. arg 1 = motor index (FL,BL,FR,BR) | arg2 = speed
#define SET_SAFETY_PARAMS 's' // sets the saftey check parameters for triggering error states

#endif