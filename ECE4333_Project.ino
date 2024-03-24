
#include <Servo.h>
#include "object_numbers.h"
#include "FastLED.h"
#include "MPU6050_getdata.h"

// ------------------ Pin Definitions  ------------------
// UART pins 
#define TX_PIN 1
#define RX_PIN 2

#define LED_BUILTIN 13
// Motor pins
#define PIN_Motor_PWM_RGHT 5
#define PIN_Motor_PWM_LEFT 6
#define PIN_Motor_DIR_RGHT 7
#define PIN_Motor_DIR_LEFT 8
#define PIN_Motor_STBY 3

// Servo pins (use servo z)
#define PIN_Servo_z 10
#define PIN_Servo_y 11

// Ultrasonic pins
#define TRIG_PIN 13  
#define ECHO_PIN 12   

// Line tracking module pins
#define LINE_TRACKER_LEFT_PIN A2
#define LINE_TRACKER_MDLE_PIN A1
#define LINE_TRACKER_RGHT_PIN A0

// RGB Led pins
#define PIN_RBGLED 4
#define NUM_LEDS 1

// ------------------ Constants  ------------------ 

#define IMAGE_PIXEL_WIDTH 420
#define IMAGE_PIXEL_HEIGHT 69

#define K_P 1.0
#define K_D 4.0
#define K_I 4.0

#define K_P_OA1 0.37
#define K_P_OA2 1.5
#define K_D_OA 0.4

#define TURN_TIME 1500
#define STOP_TIME 3000
#define PICKUP_TOTAL 2800
#define MIDDLE_IR_TOL 500

#define TURN_SPEED 60
#define TURN_ANGLE 70
#define MOVE_SPEED_OA 60                    // This is the baseline velocity of the motors, onto which a mot_diff_speed will be added/subtracted during object avoidance

#define DISTANCE_OA 20                      // The distance that will trigger object avoidance mode
#define DISTANCE_SP 20                      // The setpoint (desired) distance between the robot and the obstacle during object avoidance mode

#define SPEED_OF_SOUND 343
#define ULTRASONIC_TIMEOUT 10000

// LED color indication of the current mode
#define LINE_TRACKING_COLOR Green
#define OBJECT_AVOIDING_COLOR Brown
#define SEARCHING_COLOR Yellow
#define STOP_SIGN_COLOR Red
#define PICKED_UP_COLOR White
#define NO_STATE_COLOR Snow

// ------------------ Type Definitions  ------------------ 

enum state {
    picked_up,
    line_tracking,
    object_avoiding_1,
    object_avoiding_2,
    searching,
    stop_sign
};

// ------------------ Local Variables  ------------------ 

// Servo motor stuff
Servo servo;
int servo_angle, servo_increment = -10;
int servo_pos, servo_diff;

// Message from the ESP32 stuff
camera_classification data_in;
String msg_in;
uint8_t detected_object;
bool new_image = false;
uint32_t object_pixel_x, object_pixel_y, object_pixel_position, object_pixel_width, object_pixel_height, object_pixel_area;

// Motor speeds
int mot_rght_speed = 0, mot_left_speed = 0;
int mot_diff_speed = 0;                       // The difference between motor speed and the baseline velocity during obstacle avoidance
float value;
float rightvalue;
float finalvalue;

// Timer stuff
float start_time;
bool first = true;
float timestamp_OA;
int timer1, timer2;
int timer_OA0, timer_OA1, timer_OA2;s

// Line tracking stuff
int line_left, line_mdle, line_rght;
bool turn_right = false;

// PID loop stuff
float control_action, error, sum, prev_error;
int process_start_time;

// State stuff
state current_state = line_tracking;
CRGB leds[NUM_LEDS];
bool occupied = false;        // Flag to indicate if a time-consuming task is happenning (such as object avoidance)


// Ultrasonic sensor stuff
float distance, distance_prev, distance_delta, distance_error;
float fly_time, pulse_time;
bool pulse_sent = false;

bool clear_area_found = false; // Flag to indicate if a clear area is found
int clear_direction = 0; // D
bool check = false;
float duration;
bool first_OA = true;

// IMU stuff
MPU6050_getdata AppMPU6050getdata;
static float yaw_init, yaw, yaw_diff;

// ------------------ Initial Setup  ------------------ 
void setup() {
    Serial.begin(9600); // Serial communication channel betweeen ESP32 and UNO

    // Set up motor pins
    pinMode(PIN_Motor_PWM_RGHT, OUTPUT);
    pinMode(PIN_Motor_PWM_LEFT, OUTPUT);
    pinMode(PIN_Motor_DIR_RGHT, OUTPUT);
    pinMode(PIN_Motor_DIR_LEFT, OUTPUT);
    pinMode(PIN_Motor_STBY, OUTPUT);

    // set up servo motor and move it to 90 degrees
    servo.attach(PIN_Servo_z);
    servo.write(90);

    // setup ultrasonic sensor pins
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);

    // set up IR module pins
    pinMode(LINE_TRACKER_LEFT_PIN , INPUT);
    pinMode(LINE_TRACKER_MDLE_PIN , INPUT);
    pinMode(LINE_TRACKER_RGHT_PIN , INPUT);

    // RGB Led setup
    FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
    FastLED.setBrightness(20);

    // IMU setup
    AppMPU6050getdata.MPU6050_dveInit();
    delay(2000);
    AppMPU6050getdata.MPU6050_calibration();
    process_start_time = micros();
}

// ------------------ Main loop  ------------------ 
void loop() {
    // Get information from ESP camera module about detected object
    // digitalWrite(LED_BUILTIN, Serial.available());
    //Serial.println("not a message");

    // ------------------ Update Measurement by Sensing the Environment ------------------

    // Get IR sensor values to check if the robot is off the ground
    line_left = analogRead(LINE_TRACKER_LEFT_PIN);            
    line_mdle = analogRead(LINE_TRACKER_MDLE_PIN);
    line_rght = analogRead(LINE_TRACKER_RGHT_PIN);
    // Serial.print(line_left);Serial.print(" | "); Serial.print(line_mdle); Serial.print(" | "); Serial.println(line_rght);

    distance = returnDistance();

    // ------------------ Update Prediction by updating the internal belief ------------------
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&yaw);

    
    // ------------------ Decide Next State ------------------ 
    if (!occupied) {
      if (line_mdle < MIDDLE_IR_TOL && line_left < MIDDLE_IR_TOL && line_rght < MIDDLE_IR_TOL) {
          current_state = searching;
      }

      if(Serial.available()) {
          new_image = true;

          msg_in = Serial.readStringUntil('\n');
          // Serial.write(msg_in[0]);
          // char test[17];
          // memcpy(&test, &msg_in[0], sizeof(camera_classification));
          // Serial.write(test, 17);

          memcpy(&data_in, &msg_in[0], sizeof(camera_classification));
          
          object_pixel_x = data_in.x;
          object_pixel_y = data_in.y;
          object_pixel_width = data_in.width;
          object_pixel_height = data_in.height;
          detected_object = data_in.label_encode;
          
          Serial.print(detected_object); Serial.print(", ");
          Serial.println(turn_right);

          object_pixel_area = object_pixel_width * object_pixel_height;

          if(detected_object == STOP_SIGN) {
              current_state = stop_sign;
          } else if(detected_object == BEAR || detected_object == BULLDOZER) {
              current_state = object_avoiding_1;
          }
      }

      if (distance < DISTANCE_OA)
          current_state = object_avoiding_1;
    }
    
    if (line_left + line_mdle + line_rght > PICKUP_TOTAL) {
      current_state = picked_up;
    }

    // ----------------- Checkpoint ------------------------- //
    // Serial.println(yaw);
    Serial.print(distance); Serial.print(" | ");
    // Serial.print(line_left); Serial.print(" | ");
    // Serial.print(line_mdle); Serial.print(" | ");
    // Serial.print(line_rght); Serial.print(" | ");
    // Serial.println(line_left+line_mdle+line_rght);
    // Serial.println(current_state);
    // Serial.print(current_state); Serial.print(" | "); Serial.print(line_left + line_mdle + line_rght); Serial.print(" | ");

    // ------------------ State Machine  ------------------ 
    // Do actions based on current state and check if we need to move to a new state
    switch(current_state) {
        case picked_up:
            Serial.print("PickedUp | ");
            mot_rght_speed = 0;
            mot_left_speed = 0;
            servo_angle = 90;
            
            occupied = false;
            first_OA = true;
            

            if (line_left + line_mdle + line_rght < PICKUP_TOTAL) {
                current_state = line_tracking;
            }
            leds[0] = CRGB::PICKED_UP_COLOR;
            break;
            
        case line_tracking: // Object is found and we want to check if no new object is found, otherwise, move toward the object
            mot_rght_speed = line_left * 1.2;
            mot_left_speed = line_rght * 1.2;
            servo_angle = 90;

            leds[0] = CRGB::LINE_TRACKING_COLOR;
            Serial.print("Tracking | ");
            break;

        case object_avoiding_1: // Turning the car toward the outside of the track while rotating the servo in the opposite direction
            if (first_OA) {
              first_OA = false;
              occupied = true;
              yaw_init = yaw;
              // timer_OA0 = millis();
            }

            mot_left_speed = -TURN_SPEED * pow(-1,turn_right);
            mot_rght_speed = +TURN_SPEED * pow(-1,turn_right);

            // mot_left_speed = +TURN_SPEED;
            // mot_rght_speed = -TURN_SPEED;
            yaw_diff = yaw - yaw_init;

            if (abs(yaw_diff) < TURN_ANGLE) {
              servo_angle = 90 + yaw_diff/TURN_ANGLE * 85;
            }
            else {
              servo_angle = max(min(servo_angle,175),5);
              distance_prev = distance;                       // Record the current distance as the previous distance PD control of object avoidance
              current_state = object_avoiding_2;
            }

            leds[0] = CRGB::OBJECT_AVOIDING_COLOR;
            Serial.print("Avoid(1) | ");
            break;

        case object_avoiding_2: 
            distance_delta = distance_prev - distance;
            distance_delta = max(min(distance_delta,300),-300);
            distance_error = DISTANCE_SP - distance;
            distance_error = max(min(distance_error,100),-100); // Clampping the error for more preditability of ultrasonic reading (sometime spikes to >1100 cm)
            
            if (distance_error < 0) {
              mot_diff_speed = K_P_OA1 * distance_error + K_D_OA * distance_delta; // PD control of the motor speed
            }
            else {
              mot_diff_speed = K_P_OA2 * distance_error + K_D_OA * distance_delta; // PD control of the motor speed
            }
            
            // mot_diff_speed = K_P_OA * distance_error; // P control of the motor speed
            distance_prev = distance;

            mot_left_speed = MOVE_SPEED_OA - mot_diff_speed * pow(-1,turn_right);
            mot_rght_speed = MOVE_SPEED_OA + mot_diff_speed * pow(-1,turn_right);

            // mot_left_speed = MOVE_SPEED_OA + mot_diff_speed;
            // mot_rght_speed = MOVE_SPEED_OA - mot_diff_speed;

            if (line_mdle > MIDDLE_IR_TOL) {
              first_OA = true;
              occupied = false;
              current_state = line_tracking;
            }
            leds[0] = CRGB::OBJECT_AVOIDING_COLOR;
            Serial.print("Avoid(2) | ");
            break;

        case searching: // No object is found, move the servo around to look for an object and move forward every now and then
            if (first) {
                start_time = millis();
                first = false;
            }
            float speed_multiplier = 0.8;
            // Quickly turn the opposite direction of turning to check for line, then turn the desired direction
            if (millis() - start_time > 100) {
                mot_rght_speed = + speed_multiplier * TURN_SPEED * pow(-1, turn_right); 
                mot_left_speed = - speed_multiplier * TURN_SPEED * pow(-1, turn_right);
            } else {
                mot_rght_speed = - speed_multiplier * TURN_SPEED * pow(-1, turn_right);
                mot_left_speed = + speed_multiplier * TURN_SPEED * pow(-1, turn_right);
            }

            // mot_rght_speed = round(mot_rght_speed);
            // mot_left_speed = round(mot_left_speed);
            
            // if line detected, go to line tracking mode
            if (line_mdle > MIDDLE_IR_TOL) {
                first = true;
                current_state = line_tracking;
                break;
            }

            // if image detected, act based on what that image is
            // if (new_image) {
            //     if (detected_object == NO_OBJECT) {
            //         servo_angle += servo_increment;

            //         if (servo_angle > 180 || servo_angle < 0) {
            //             servo_increment *= -1;

            //             mot_rght_speed = 200;
            //             mot_left_speed = 200;
            //         }
            //     } else {
            //         first = true;
            //         current_state = line_tracking;
            //     }
            // }
            leds[0] = CRGB::SEARCHING_COLOR;
            Serial.print("Searchin | ");
            break;

        

        case stop_sign:
            if (first) {
                start_time = millis();
                first = false;
            }

            mot_rght_speed = 0;
            mot_left_speed = 0;

            // State transition to line tracking
            if (millis() - start_time > STOP_TIME) {
                turn_right ^= 1;
                first = true;
                current_state = line_tracking;
            } 

            // LED indication
            leds[0] = CRGB::STOP_SIGN_COLOR;
            Serial.print("Stopped | ");
            break;


        default: // Unknown state, stop moving
            mot_rght_speed = 0;
            mot_left_speed = 0;
            servo_angle = 90;
            leds[0] = CRGB::NO_STATE_COLOR;
            Serial.print("Defaultt | ");
            break;

    }
    // Serial.print(current_state); Serial.print(" | "); Serial.println(line_left+line_mdle+line_rght);
    mot_left_speed = max(min(mot_left_speed,255),-255);
    mot_rght_speed = max(min(mot_rght_speed,255),-255);
    // Serial.print(mot_left_speed); Serial.print(" | "); Serial.print(mot_rght_speed);
    Serial.println("");


    // ------------------ Clean up actions  ------------------ 

    new_image = false;

    FastLED.show();

    // Activate servo motors and DC motors
    servo.write(servo_angle);

    // Drive the motors
    digitalWrite(PIN_Motor_STBY, 1);

    digitalWrite(PIN_Motor_DIR_RGHT, mot_rght_speed > 0);
    digitalWrite(PIN_Motor_DIR_LEFT, mot_left_speed > 0);

    analogWrite(PIN_Motor_PWM_RGHT, abs(mot_rght_speed) );
    analogWrite(PIN_Motor_PWM_LEFT, abs(mot_left_speed) ); 

    // Serial.print(line_left);
    // Serial.print(", ");
    // Serial.print(line_mdle);
    // Serial.print(", ");
    // Serial.println(line_rght);
}

float returnDistance(){
      servo.write(servo_angle);
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      duration = pulseIn(ECHO_PIN, HIGH);
      distance = duration / 2 * SPEED_OF_SOUND / 1000000 * 100;
      return distance;
}






