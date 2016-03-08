#ifndef _SENSORS_ACTUATORS_H_
#define _SENSORS_ACTUATORS_H_

#include "robot_state.h"

// Ports and bits for IO ports
// Two layers of macros so that prt gets expanded before concatenation
#define READ(prt) _READ(prt)
#define _READ(prt) READ_PORT_##prt
#define WRITE(prt) _WRITE(prt)
#define _WRITE(prt) WRITE_PORT_##prt

#define LINE_SENS_PORT 0
#define LINE_SENS_BIT_0 0x01
#define LINE_SENS_BIT_1 0x02
#define LINE_SENS_BIT_2 0x04
#define LINE_SENS_BIT_3 0x08

#define SWITCH_PORT 0
#define SWITCH_CLAW_CLOSED 0x10
#define SWITCH_BUMP_LEFT 0x20
#define SWITCH_BUMP_RIGHT 0x40

#define ACTUATOR_PORT 1
#define ACTUATOR_CLAW_CLOSE 0x40
#define ACTUATOR_CLAW_WIDEN 0x80
#define LED_LDR 0x08
#define LED_EGG 0x01
#define LED_CHICK 0x02
#define LED_WHITE 0x04

#define ADC_LDR ADC0

#define MOTOR_LEFT_GO MOTOR_1_GO
#define MOTOR_RIGHT_GO MOTOR_2_GO
#define MOTOR_PADDLE_GO MOTOR_3_GO


// LDR reading takes longer and is only needed at certain points, so is called separately.
void init_sensors_actuators(robot_state &state);
void update_sensor_values(robot_state &state);
void read_ldr(robot_state &state);

void close_claw(robot_state &state);
void open_claw(robot_state &state);
void widen_claw(robot_state &state);
void narrow_claw(robot_state &state);

void set_led(robot_state &state, int led_bit, bool on);
void update_status_leds(robot_state &state);

// speed: -1.f to 1.f
// turning_ratio: -1.f to 1.f
// left motor = clamp(speed + turning_ratio)  (i.e. turning right is +ve)
// right motor= clamp(speed - turning_ratio)
// turning in place: 0 speed, non zero turning ratio
// max speed, max turning = one motor on full, the other stopped.
void move(robot_state &state, float speed, float turning_ratio);

// punt that shit out:
void flap_flapper(robot_state &state);
void unflap_flapper(robot_state &state);

void debug_dump(robot_state &state);

#endif // _SENSORS_ACTUATORS_H_
