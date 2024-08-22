// Globals.h
#pragma once
#include <Arduino.h>

#define STATUS_ARRAY_SIZE 25
#define STATUS_ARRAY_SIZE2 25
#define PITCH_INDEX 0
#define ROLL_INDEX 1
#define YAW_INDEX 2
extern float gps_coordinates[5];
extern char status_array[STATUS_ARRAY_SIZE2];
extern float X_Value;
extern float Y_Value;
extern uint16_t axes_pwms[3];
