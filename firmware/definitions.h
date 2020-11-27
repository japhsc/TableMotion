#ifndef DEFINITIONS_H
#define DEFINITIONS_H

/* HID */

// LEDs
#define PIN_LED_13          (13u)
#define PIN_LED_RXL         ( 0u)
#define PIN_LED_TXL         ( 1u)

// Indicator
#define PIN_IND_1           12    //    red, 
#define PIN_IND_2           13    // yellow, LED_BUILTIN
#define PIN_IND_3           1     // yellow, TXL
#define PIN_IND_4           0     // yellow, RXL
const uint8_t PIN_IND[4]={PIN_IND_1, PIN_IND_2, PIN_IND_3, PIN_IND_4};

// Buttons
#define PIN_BUTTON_1        14
#define PIN_BUTTON_2        15
#define PIN_BUTTON_3        16
#define PIN_BUTTON_4        17
const uint8_t PIN_BUTTON[4]={PIN_BUTTON_1, PIN_BUTTON_2, PIN_BUTTON_3, PIN_BUTTON_4};

/* MOTOR */

//number of motors
#define num_motor           2

// Overcurrent protection e.g. at end position
// Calibtate current sense (SHUNT) -> with table (370/800)
// ADC 800 -> 800/1023*1.1V*8.5 = 7.3A
// Rated for 4.8A -> 4.8A/8.5/1.1V*1023 = 525 
#define IS_OC               800
#define IS_OC_WARN          525

#define PWM_CRUISE          80
#define PWM_LOWER           50
#define PWM_UPPER           250
#define PWM_DEC             10
#define PWM_INC             5

#define MOTOR_EN_A          2
#define MOTOR_EN_B          4

const uint8_t MOTOR_EN[2] = {MOTOR_EN_A, MOTOR_EN_B};

// Motor A
#define A_MOTOR_R_PWM       9
#define A_MOTOR_L_PWM       3
#define A_MOTOR_R_IS        A6
#define A_MOTOR_L_IS        A7
#define A_HALL_SENSOR_1     5  // hall sensor 1: red (8)
#define A_HALL_SENSOR_2     6  // hall sensor 1: green (1)

// Motor B
#define B_MOTOR_R_PWM       10
#define B_MOTOR_L_PWM       11

#define B_MOTOR_R_IS        A5
#define B_MOTOR_L_IS        A4

#define B_HALL_SENSOR_1     7  // hall sensor 1: red (8)
#define B_HALL_SENSOR_2     8  // hall sensor 1: green (1)

const uint8_t MOTOR_PWM[2][2]    = {  {A_MOTOR_L_PWM,B_MOTOR_L_PWM},
                                      {A_MOTOR_R_PWM,B_MOTOR_R_PWM}};
const uint8_t MOTOR_IS[2][2]     = {  {A_MOTOR_L_IS,B_MOTOR_L_IS}, 
                                      {A_MOTOR_R_IS,B_MOTOR_R_IS}};
const uint8_t HALL_SENSOR[2][2]  = {  {A_HALL_SENSOR_1,B_HALL_SENSOR_1},
                                      {A_HALL_SENSOR_2,B_HALL_SENSOR_2}};

// Default positions
#define POS0    10
#define POS1    70
#define POS2    200
#define POS3    400

// Position limits
#define POS_LOWER     5
#define POS_UPPER     420


// EEPROM
#define ADDR_STEPS_A        0
#define ADDR_STEPS_B        ADDR_STEPS_A+sizeof(long)
#define ADDR_TARGETS        ADDR_STEPS_B+sizeof(long)
#define ADDR_INDEX          ADDR_TARGETS+4*sizeof(long)

const uint8_t ADDR_STEPS[2]={ADDR_STEPS_A, ADDR_STEPS_B};

#endif
