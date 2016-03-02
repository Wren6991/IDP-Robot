#include "sensors_actuators.h"

#include <cmath>
#include <iostream>
#include <robot_delay.h>
#include <robot_instr.h>
#include <robot_link.h>

#include "robot_state.h"
#include "line_following.h"
// Sensor control code

void init_sensors(robot_state &state)
{
	// Inputs need to be written 1 to set high impedance
	int line_bits = state.link->request(READ(LINE_SENS_PORT));
	state.link->command(WRITE(LINE_SENS_PORT), line_bits |
		LINE_SENS_BIT_0 |
		LINE_SENS_BIT_1 |
		LINE_SENS_BIT_2 |
		LINE_SENS_BIT_3);
	int switch_bits = state.link->request(READ(SWITCH_PORT));
	state.link->command(WRITE(SWITCH_PORT), switch_bits |
		SWITCH_CLAW_CLOSED |
		SWITCH_BUMP_LEFT |
		SWITCH_BUMP_RIGHT);
	state.link->command(RAMP_TIME, 0);
}

void update_sensor_values(robot_state &state)
{
	int line_bits, switch_bits;
	line_bits = state.link->request(READ(LINE_SENS_PORT));
	if (LINE_SENS_PORT == SWITCH_PORT)
		switch_bits = line_bits;
	else
		switch_bits = state.link->request(READ(SWITCH_PORT));
	
	state.line_sens[0] = line_bits & LINE_SENS_BIT_0;
	state.line_sens[1] = line_bits & LINE_SENS_BIT_1;
	state.line_sens[2] = line_bits & LINE_SENS_BIT_2;
	state.line_sens[3] = line_bits & LINE_SENS_BIT_3;

	state.claw_closed = !(switch_bits & SWITCH_CLAW_CLOSED);
	state.bump_left = !(switch_bits & SWITCH_BUMP_LEFT);
	state.bump_right = !(switch_bits & SWITCH_BUMP_RIGHT);

	state.line_state = line_state_from_sensors(state);
}

void read_ldr(robot_state &state)
{
	// 100 ms: about 10 time constants for 2.2kO, 4.7uF
	// Resistance against luminous intensity is a straight line on a log-log plot
	// So we are interested in determining the LDR resistance from value
	// v = V / Vcc  and then comparing logs
	const float R1 = 2200;
	set_led(state, LED_LDR, true);
	delay(100);
	float v_lighton = state.link->request(ADC_LDR) / 256.f;
	set_led(state, LED_LDR, false);
	delay(100);
	//float v_lightoff = state.link->request(ADC_LDR) / 256.f;

	// v = R / (R + R1)
	// so  R/R1 = v/(1-v)
	float r_lighton = R1 * v_lighton / (1 - v_lighton);
	//float r_lightoff = R1 * v_lightoff / (1 - v_lightoff);

	state.light_sensor = 500 / r_lighton;// - 500 / r_lightoff;	// lux =~= 500/R
}

// Actuator control code

void close_claw(robot_state &state)
{
	int prev_value = state.link->request(READ(ACTUATOR_PORT));
	state.link->command(WRITE(ACTUATOR_PORT), prev_value | ACTUATOR_CLAW_CLOSE);
}

void open_claw(robot_state &state)
{
	int prev_value = state.link->request(READ(ACTUATOR_PORT));
	state.link->command(WRITE(ACTUATOR_PORT), prev_value & ~ACTUATOR_CLAW_CLOSE);
}

void widen_claw(robot_state &state)
{
	int prev_value = state.link->request(READ(ACTUATOR_PORT));
	state.link->command(WRITE(ACTUATOR_PORT), prev_value | ACTUATOR_CLAW_WIDEN);
}

void narrow_claw(robot_state &state)
{
	int prev_value = state.link->request(READ(ACTUATOR_PORT));
	state.link->command(WRITE(ACTUATOR_PORT), prev_value & ~ACTUATOR_CLAW_WIDEN);
}

void set_led(robot_state &state, int led_bit, bool on)
{
	int bits = state.link->request(READ(ACTUATOR_PORT));
	state.link->command(WRITE(ACTUATOR_PORT), (bits & ~led_bit) | (on ? led_bit : 0x00));
}

void update_status_leds(robot_state &state)
{
	set_led(state, LED_EGG, state.have_egg);
	set_led(state, LED_CHICK, state.have_chick);
	set_led(state, LED_WHITE, state.have_white);
}

// Motor control

inline float clipf(float x, float min, float max)
{
	return x < min ? min : x > max ? max : x;
}

inline int float2signmag(float x)
{
	x = clipf(x, -1, 1);
	return ((int)(fabs(x) * 0x7f)) | (x < 0 ? 0x80 : 0x00);
}

void move(robot_state &state, float speed, float turning_ratio)
{
	speed = clipf(speed, -1, 1);
	turning_ratio = clipf(turning_ratio, -1, 1);
	float left = clipf(speed + turning_ratio, -1, 1);
	float right = clipf(speed - turning_ratio, -1, 1);
	state.link->command(MOTOR_LEFT_GO, float2signmag(left));
	state.link->command(MOTOR_RIGHT_GO, float2signmag(-right));
} 


void debug_dump(robot_state &state)
{
	std::cout << "Line sensors: ";
	for (int i = 0; i < state.N_LINE_SENSORS; ++i)
		std::cout << (state.line_sens[i] ? "1" : "0");
	std::cout << "\n";

	std::cout << "Bump sensors: " <<
		(state.bump_left ? "L" : "-") <<
		(state.bump_right ? "R" : "-") << "\n";
	std::cout << "Claw sensor:  " <<
		(state.claw_closed ? "Closed\n" : "Open\n");
	std::cout << "Light level:  " << state.light_sensor << "\n";
	std::cout << "Line state:   " << state.line_state << "\n";

	std::cout << "\n";
}
