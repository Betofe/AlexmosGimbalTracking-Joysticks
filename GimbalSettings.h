// GimbalSettings.h

#pragma once

#include <Arduino.h>

#define PITCH_PIN 26
#define YAW_PIN 13
#define ROLL_PIN 14

#define FREQUENCY 400
#define PITCH_CHANNEL 0
#define YAW_CHANNEL 1
#define ROLL_CHANNEL 2
#define RESOLUTION 8
#define DEBUG_BAUD_RATE 115200
#define START_BYTE 4
#define MAX_PWM 200
#define MIN_PWM 100





class Gimbal {
private:
	//uint16_t last_pitch_pwm;
	//uint16_t last_yaw_pwm;
	//uint16_t last_roll_pwm;
	//uint16_t current_pitch_pwm_value;
	//uint16_t current_roll_pwm_value;
	//uint16_t current_yaw_pwm_value;
	uint16_t pitch_pwm;
	uint16_t roll_pwm;
	uint16_t yaw_pwm;
	uint16_t  previousErrorX;
	uint16_t previousErrorY;
	uint16_t sumErrorX;
	uint16_t sumErrorY;
	uint16_t FRAME_CENTER_X;
	uint16_t FRAME_CENTER_Y;
	
	uint16_t objectX;
	uint16_t objectY;
	uint16_t errorX;
	uint16_t errorY;
	uint16_t value1;
	uint16_t width ;
	uint16_t height ;
	
	String msg_to_pi;

	int i = 0;

	

public:
	void init();
	void map_pwm();
	void track_object();
	void run();
	void convertToArray();
	void SendGpsCord();
	//void updatePWM(uint16_t channel[3], uint16_t last_pwm_value[3], uint16_t current_pwm_value[3]);

};
