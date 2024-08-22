// 
// 
// 

#include "GimbalSettings.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include "Globals.h"
#include "MavlinkSettings.h"

MavlinkConnection mav;
HardwareSerial espSerial(1);
byte character = 0;
const unsigned int MAX_MESSAGE_LENGTH = 12;


void Gimbal::init() {
 
    
    espSerial.begin(9600, SERIAL_8N1, 21, 22);
    ledcAttachPin(PITCH_PIN, PITCH_CHANNEL);
    ledcAttachPin(YAW_PIN, YAW_CHANNEL);
    ledcAttachPin(ROLL_PIN, ROLL_CHANNEL);
   
    ledcSetup(PITCH_CHANNEL, FREQUENCY, RESOLUTION);
    ledcSetup(YAW_CHANNEL, FREQUENCY, RESOLUTION);
    ledcSetup(ROLL_CHANNEL, FREQUENCY, RESOLUTION);

    pitch_pwm = 0;
    yaw_pwm = 0;
    roll_pwm = 0;
    gps_coordinates[START_BYTE] = 0.3;

  
    width = 630;
    height = 350;
    FRAME_CENTER_X = width/2;
    FRAME_CENTER_Y = height/2;
}
void Gimbal::map_pwm() {
    if (value1 == 0) {
        pitch_pwm = map(axes_pwms[PITCH_INDEX], CHANNEL_MIN, CHANNEL_MAX, MIN_PWM, MAX_PWM);
        roll_pwm = map(axes_pwms[ROLL_INDEX], CHANNEL_MIN, CHANNEL_MAX, MIN_PWM, MAX_PWM);
        yaw_pwm = map(axes_pwms[YAW_INDEX], CHANNEL_MIN, CHANNEL_MAX, MIN_PWM, MAX_PWM);
    //if ((axes_pwms[YAW_INDEX] >= 1492) && (axes_pwms[YAW_INDEX] <= 1502)) {
     //   yaw_pwm = 150;
    //}
    //else if ((axes_pwms[PITCH_INDEX] >= 1492) && (axes_pwms[PITCH_INDEX] <= 1502)) {
    //    pitch_pwm = 150;
    //}
    //else if ((axes_pwms[ROLL_INDEX] >= 1492) && (axes_pwms[ROLL_INDEX] <= 1502)) {
    //    roll_pwm = 150;
   // }
    //else if ((axes_pwms[YAW_INDEX] > 2000) && (axes_pwms[YAW_INDEX] < 0)) {
     //   yaw_pwm = 0;
    //}
    //else if ((axes_pwms[PITCH_INDEX] > 2000) && (axes_pwms[PITCH_INDEX] < 0)) {
   //    pitch_pwm = 0;
   // }
    //else if ((axes_pwms[ROLL_INDEX] > 2000) && (axes_pwms[ROLL_INDEX] < 0)) {
    //    roll_pwm = 0;
    //}
    }
    
}
void Gimbal::track_object() {
    
   // if (value1 == 1) {

    //    pitch_pwm = (map(Y_Value, -FRAME_CENTER_Y, FRAME_CENTER_Y, MIN_PWM, MAX_PWM));
    //    yaw_pwm = (map(X_Value, -FRAME_CENTER_X, FRAME_CENTER_X, MIN_PWM, MAX_PWM));

      
   // }
   // else if ((value1 == 1) && (X_Value >= 300) && (Y_Value >=165 )) {
   //     pitch_pwm = MAX_PWM;
   //     yaw_pwm = MAX_PWM;
   // }
   // else if ((value1 == 1) && (X_Value >= - 300) && (Y_Value >= - 165)) {
    //    pitch_pwm = MIN_PWM;
    //    yaw_pwm = MIN_PWM;
   // }
   // else
   // {
     //   roll_pwm = 0;
     //   pitch_pwm = 0;
    //    yaw_pwm = 0;
   // }
}
void Gimbal::convertToArray() {
    
    String message = "X,Y: ";
    message += X_Value;
    message += "px";
    message += " ";
    message += Y_Value;
    message += "px";
    message.toCharArray(status_array, STATUS_ARRAY_SIZE2);
}
//void Gimbal::updatePWM(uint16_t channel[3], uint16_t last_pwm_value[3], uint16_t current_pwm_value[3]) {
 //   last_pitch_pwm = pitch_pwm;
 //   last_roll_pwm = roll_pwm;
 //   last_yaw_pwm = yaw_pwm;
 //   for (int i = 0; i < 3; i++) {
        // Check if the current PWM value is different from the last PWM value
       // if (last_pwm_value[i] != current_pwm_value[i]) {
        //    ledcWrite(channel[i], current_pwm_value[i]);
        //    last_pwm_value[i] = current_pwm_value[i];
       // }
   // }
//}




void Gimbal::run() {
   // uint16_t channelArray[3] = { PITCH_CHANNEL, YAW_CHANNEL, ROLL_CHANNEL };
   // uint16_t last_pwm_valueArray[3] = { last_pitch_pwm, last_roll_pwm, last_yaw_pwm };
   // uint16_t current_pwm_valueArray[3] = { current_pitch_pwm_value, current_roll_pwm_value, current_yaw_pwm_value };
    mav.run();
    track_object();
    map_pwm();
    // Update PWM if values have changed
  //  updatePWM(channelArray, last_pwm_valueArray, current_pwm_valueArray);
     SendGpsCord();

        // Conditional serial output if the PWM values have changed
  
   
        Serial.print("yaw_pwm: ");
        Serial.print(yaw_pwm);
        Serial.print('\t');
        Serial.print("pitch_pwm: ");
        Serial.println(pitch_pwm);
        Serial.print('\t');
        Serial.print("roll_pwm: ");
        Serial.println(roll_pwm);
  //  }

    while (espSerial.available()) {

        //Create a place to hold the incoming message
        static char message[MAX_MESSAGE_LENGTH];
        static unsigned int message_pos = 0;

        //Read the next available byte in the serial receive buffer
        char inByte = espSerial.read();

        //Message coming in (check not terminating character) and guard for over message size
        if (inByte != '!' && (message_pos < MAX_MESSAGE_LENGTH - 1))
        {
            //Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        }
        //Full message received...
        else
        {
            //Add null character to string
            message[message_pos] = '\0';

            // Convert the received message to float values
            int parsedValues = sscanf(message, "%d,%f,%f", &value1, &X_Value, &Y_Value);
        
            if (parsedValues == 3) {
                
                Serial.print("yaw_pwm: ");
                Serial.print(yaw_pwm);
                Serial.print('\t');
                Serial.print("pitch_pwm: ");
                Serial.println(pitch_pwm);
             
            }
            else {
                Serial.println("Error parsing values from message");
            }

            //Reset for the next message
            message_pos = 0;
        }
    }
   
    
}

void Gimbal::SendGpsCord() {
    float data[5] = { gps_coordinates[START_BYTE], gps_coordinates[LONGITUDE], gps_coordinates[LATITUDE],gps_coordinates[ALTITUDE], gps_coordinates[RELATIVE_ALTITUDE] };

    espSerial.write((byte*)data, sizeof(data));
    //delay(500);
    if (i < 5) {
        i++;
    }
    else {
        i = 0;
    }
}

