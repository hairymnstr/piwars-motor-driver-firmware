#include <stdint.h>
#include <stdbool.h>

#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_tim.h"

#include "address.h"
#include "command_handler.h"

enum comms_state {
    STATE_IDLE,
    STATE_COMMAND,
    STATE_PAYLOAD,
    STATE_CHECKSUM,
};

enum comms_state state = STATE_IDLE;

#define CMD_SET_MOTOR_SPEED     10
#define CMD_SET_SERVO_ANGLE     11
#define CMD_SET_MOTOR_REVERSED  12
#define CMD_SET_SERVO_REVERSED  13
#define CMD_SET_SERVO_TRIM      14

#define DIRECTION_FORWARD   0
#define DIRECTION_REVERSE   1

uint8_t cks;
uint8_t cmd;
uint8_t payload[256];
uint8_t payload_len;
uint8_t payload_idx;

bool invert_all = false;
bool servo_reversed = false;
uint16_t servo_trim = 900;

/*
 * ISR function
 */
void new_byte(uint16_t input)
{
    int pwm_counts;
    int direction;
    int motor_speed;
    // handle an address byte regardless of state
    if(input & 0x100)
    {
        // my_address() returns a bit mask so see if our bit is set, allows group commands
        if(input & my_address())
        {
            state = STATE_COMMAND;
            LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_1);        // toggle green LED whenever a command is received
        }
        else
        {
            state = STATE_IDLE;         // clear the state machine if we got a new address and were in any other state
        }
    }
    else
    {
        switch(state)
        {
            case STATE_IDLE:
                // never do anything in IDLE, we exit if we get an address
                // but that happens regardless of state so we can recover
                // from a comms error so nothing to do in this state
                break;
            case STATE_COMMAND:
                cks = (input & 0xff);
                payload_len = 0;
                payload_idx = 0;
                cmd = (input & 0xff);
                switch(input & 0xff)
                {
                    case CMD_SET_SERVO_ANGLE:
                        payload_len = 2;
                        state = STATE_PAYLOAD;
                        break;
                    case CMD_SET_MOTOR_SPEED:
                        payload_len = 1;
                        state = STATE_PAYLOAD;
                        break;
                    case CMD_SET_MOTOR_REVERSED:
                        payload_len = 1;
                        state = STATE_PAYLOAD;
                        break;
                    case CMD_SET_SERVO_REVERSED:
                        payload_len = 1;
                        state = STATE_PAYLOAD;
                        break;
                    case CMD_SET_SERVO_TRIM:
                        payload_len = 2;
                        state = STATE_PAYLOAD;
                        break;
                    default:
                        state = STATE_IDLE; // unhandled commands puts the state machine back to idle.  Nothing more will be done until a new command (address) is received
                }
                break;
            case STATE_PAYLOAD:
                cks ^= (input & 0xff);
                payload[payload_idx] = (input & 0xff);
                payload_idx++;
                if(payload_idx == payload_len)
                {
                    state = STATE_CHECKSUM;
                }
                break;
            case STATE_CHECKSUM:
                cks ^= (input & 0xff);
                if(cks != 0)
                {
                    state = STATE_IDLE; // bad checksum go back to waiting for a command
                    break;
                }
                switch(cmd)
                {
                    case CMD_SET_SERVO_ANGLE:
                        // servo angle sent in degrees
                        // accept a range of 0-180 degrees in tenths of a degree i.e. 0 to 1800, with 900 being centre
                        pwm_counts = *(int16_t *)(payload);
                        if(servo_reversed)
                        {
                            pwm_counts *= -1;
                        }
                        pwm_counts += servo_trim;
                        if(pwm_counts >= 1800)
                        {
                            pwm_counts = 1799;
                        }
                        if(pwm_counts < 0)
                        {
                            pwm_counts = 0;
                        }
                        pwm_counts = (1000 * pwm_counts) / 1800;    // only 1000 steps in our sweep so need to do some maths
                        LL_TIM_OC_SetCompareCH1(TIM3, 1000 + pwm_counts);   // minimum pulse width 1000 counts, maximum 2000
                        break;
                    case CMD_SET_MOTOR_SPEED:
                        // set speed
                        // accept values in range -100,100 being 100% reverse to 100% forward
                        motor_speed = (int8_t)payload[0];
                        if(invert_all)
                        {
                            direction = DIRECTION_REVERSE;
                        }
                        else
                        {
                            direction = DIRECTION_FORWARD;
                        }
                        if(motor_speed < 0)
                        {
                            if(invert_all)
                            {
                                direction = DIRECTION_FORWARD;
                            }
                            else
                            {
                                direction = DIRECTION_REVERSE;
                            }
                            motor_speed *= -1;
                        }
                        if(motor_speed > 100)
                        {
                            motor_speed = 100;
                        }
                        if(direction == DIRECTION_FORWARD)
                        {
                            LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
                            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
                        }
                        else
                        {
                            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
                            LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
                        }
                        LL_TIM_OC_SetCompareCH1(TIM2, motor_speed * 4);
                        break;
                    case CMD_SET_MOTOR_REVERSED:
                        // make all speed set commands from now inverted
                        if(payload[0])
                        {
                            invert_all = true;
                        }
                        else
                        {
                            invert_all = false;
                        }
                        break;
                    case CMD_SET_SERVO_REVERSED:
                        // swap the direction of servo angle setting commands from now on
                        if(payload[0])
                        {
                            servo_reversed = true;
                        }
                        else
                        {
                            servo_reversed = false;
                        }
                        break;
                    case CMD_SET_SERVO_TRIM:
                        // set the centre point for servo angle commands
                        servo_trim = payload[0] + 256 * payload[1];
                        if(servo_trim >= 1800)
                        {
                            servo_trim = 1799;
                        }
                        break;
                }
                state = STATE_IDLE;
                break;
        }
    }
}
