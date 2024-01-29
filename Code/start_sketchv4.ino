// MAZE SOLVER for Coppelia Sim simulation, made by MALHOUQ Anas and WAELES-DEVAUX Adrien

#include <assert.h> // library used to manually trigger errors.

// Definition of sensors
#define LIGHT_SENSOR_PIN A0
#define PROX_SENSOR_L_PIN A4
#define PROX_SENSOR_R_PIN A3
#define PROX_SENSOR_FL_PIN A1
#define PROX_SENSOR_FR_PIN 9
#define PROX_SENSOR_RL_PIN 6
#define PROX_SENSOR_RR_PIN 12
#define PROX_SENSOR_DL_PIN A2
#define PROX_SENSOR_DR_PIN A5

// Definition of motors
#define MOTOR_RF_PIN 2
#define MOTOR_RB_PIN 4
#define MOTOR_R_SPEED 3
#define MOTOR_LF_PIN 7
#define MOTOR_LB_PIN 8
#define MOTOR_L_SPEED 5

enum strategyEnum { FOLLOWING_BORDER, EXPLORING_CENTER_WALLS, RANDOM}; // Algorithm used by the robot to navigate through the maze
enum stateEnum {SEARCHING_BLACK_CELL, SEARCHING_RED_CELL, FINISH}; // Current state of the robot

// Initial state and strategy
stateEnum state = SEARCHING_BLACK_CELL;
strategyEnum strategy = FOLLOWING_BORDER;

// Variables definition
int light_sensor_value = 0; // Contains the value read by an infrared sensor under the robot
float error = 0; // Error between the distance from the wall we want and the real position
float corrector = 0; // PID applied to the error

unsigned long first_black_detected = -1; // Records the time at which black is first detected, -1 if the robot is not on black
unsigned long last_black_detected = -1; // Records the time at which black is not detected anymore
unsigned long black_detected_duration = 0;

unsigned long first_red_detected = -1; // Records the time at which red is first detected, -1 if the robot is not on red
unsigned long last_red_detected = -1; // Records the time at which black is not detected anymore
unsigned long red_detected_duration = 0;

int leftSpeed = 0; // Speed sent to the left motor after correction
int rightSpeed = 0; 

bool following_left_wall = true; // if false we follow the right wall
// when the robot switch from left to right wall following mode, as we follow the right wall we will wait that the left wall we left is out of sight before following a new left wall :
bool leaved_left_wall = false; 
// same principle when we switch from right to left wall following mode
bool leaved_right_wall = false; 


// -----------------------------------------------------------------------------------------------------------------------------------------------------------
// Please adapt these values to fit with your computer specifications which are affecting the simulation speed.

float proportional_coefficient = 0.4; // Proportional coefficient from PID corrector. Here Integral and Derivative coefficient have a insignificant impact on the simulation so there are zero
float speed_command = 200; // Speed command for straight lines and before correction
float followingBorder_diagonal_distance_command = 700; // rear diagonal distance to keep between the robot and the wall when using "FOLLOWING_BORDER" strategy
float exploringCenter_diagonal_distance_command = 920; // same when using "EXPLORING_CENTER_WALLS"

int switchWall_distance_detection = 1000; // Minimal distance from the wall from which we have to change the followed wall

// Infrared sensor color detection thresholds
int black_superior_limit = 60; // Under this value, considered as black
int white_inferior_limit = 500; // Above this value, considered as white
int red_inferior_limit = 150; // Above this value and under white inferior limit, considered as red

int red_cell_time_threshold = 300; // minimal red dectection duration to consider it as a red cell
int black_cell_time_threshold = 200; // minimal black dectection duration to consider it as a black cell
// -----------------------------------------------------------------------------------------------------------------------------------------------------------


void hardware_setup() {
  // Horosim simulation parameters, not modified
  new DCMotor_Hbridge(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_R_SPEED, "ePuck_rightJoint", 2.5, 3 * 3.14159, 1);
  new DCMotor_Hbridge(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_L_SPEED, "ePuck_leftJoint", 2.5, 3 * 3.14159, 1);

  new VisionSensor(LIGHT_SENSOR_PIN, "ePuck_lightSensor", 0.1);

  new ProximitySensor(PROX_SENSOR_FL_PIN, "ePuck_proxSensor3", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_FR_PIN, "ePuck_proxSensor4", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_L_PIN, "ePuck_proxSensor1", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_R_PIN, "ePuck_proxSensor6", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RL_PIN, "ePuck_proxSensor7", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RR_PIN, "ePuck_proxSensor8", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DL_PIN, "ePuck_proxSensor2", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DR_PIN, "ePuck_proxSensor5", 0.1, 1);
}


void setup() {
  Serial.begin(4800); // Communication speed
  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(MOTOR_R_SPEED, OUTPUT);
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_L_SPEED, OUTPUT);
}

void move(int leftSpeed, int rightSpeed){ 
  // Used to send commands to motors specifying for each one a value between -255 and 255 (a negative value means the wheel will roll backwards)
  analogWrite(MOTOR_L_SPEED, abs(leftSpeed));
  analogWrite(MOTOR_R_SPEED, abs(rightSpeed));

  // To avoid unwanted comportments, we trigger an error if one speed value is positive and the other is negative and they are not opposite.
  assert(leftSpeed==-rightSpeed || leftSpeed*rightSpeed>=0);
  
  if(leftSpeed>=0){ // Left wheel moves forward
    digitalWrite(MOTOR_LF_PIN, HIGH);
    digitalWrite(MOTOR_LB_PIN, LOW);
  }
  else{  // Left wheel moves backward
    digitalWrite(MOTOR_LF_PIN, LOW);
    digitalWrite(MOTOR_LB_PIN, HIGH);
  }
  
  if (rightSpeed>=0){ // Right wheel moves forward
    digitalWrite(MOTOR_RF_PIN, HIGH);
    digitalWrite(MOTOR_RB_PIN, LOW);
  }
  else{ // Right wheel moves backward
    digitalWrite(MOTOR_RF_PIN, LOW);
    digitalWrite(MOTOR_RB_PIN, HIGH);
  }

}

unsigned long black_duration(){
  // Returns black detection time
  if (light_sensor_value < black_superior_limit && first_black_detected == -1){ // If our light sensor detects black color and wasn't detecting it before
    first_black_detected = millis(); // We record this time as the first time black was detected
  }

  else if (first_black_detected!=-1  && light_sensor_value > white_inferior_limit){ // If our light sensor has detected black color before and now detects white
    last_black_detected = millis(); // We record this time as the last time black was detected
    black_detected_duration = last_black_detected - first_black_detected; 
    first_black_detected = -1; // We renitialize our variable for next function calls
  }

  return black_detected_duration;
}

unsigned long red_duration(){
  // Returns red detection time
  if (light_sensor_value > red_inferior_limit && light_sensor_value < white_inferior_limit && first_red_detected == -1){ // If our light sensor detects red color and wasn't detecting it before
    first_red_detected = millis(); // We record this time as the first time red was detected
  }

  else if (first_red_detected!=-1  && (light_sensor_value > white_inferior_limit || light_sensor_value < red_inferior_limit) ){ // If our light sensor has detected red color before and not anymore
    last_red_detected = millis(); // We record this time as the last time red was detected
    red_detected_duration = last_red_detected - first_red_detected;
    first_red_detected = -1;
  }

  return red_detected_duration;
}

void switchFollowingWall(){
  // Switch the followed wall between left and right when a new wall is detected to be able to explore all the walls from the maze.

  // If we are following the left wall, we have left the previous wall detected and a wall is detected to our right (except if we are on a red line which may be the red cell)
  if (following_left_wall && leaved_right_wall && analogRead(PROX_SENSOR_R_PIN)<switchWall_distance_detection && light_sensor_value < red_inferior_limit){
      following_left_wall = false; // now we follow the right wall
      leaved_right_wall = false; // we reset our variable for next time
    }

    // Verify if after switching, we have left the previous wall we were following to avoid switching over and over
    if(!following_left_wall && !leaved_left_wall && analogRead(PROX_SENSOR_L_PIN)>switchWall_distance_detection){ 
      leaved_left_wall = true; 
    }
    if(following_left_wall && !leaved_right_wall && analogRead(PROX_SENSOR_R_PIN)>switchWall_distance_detection){
      leaved_right_wall = true;
    }

    // If we are following the right wall, we have left the previous wall detected and a wall is detected to our left
    if (leaved_left_wall && !following_left_wall && analogRead(PROX_SENSOR_L_PIN)<switchWall_distance_detection){
      following_left_wall = true; // now we are following left wall
      leaved_left_wall = false; // we reset our variable for next time
    }
}

void leftWall_PID(){
  error = followingBorder_diagonal_distance_command - analogRead(PROX_SENSOR_DL_PIN); // Then the error (distance from the wall) is calculated using back diagonal left sensor 
  corrector = proportional_coefficient * error; // We apply our PID (proportional was enough here)
  // The correction is then added to our speed commands
  leftSpeed = speed_command + corrector; 
  rightSpeed = speed_command - corrector;
}

void rightWall_PID(){
  error = exploringCenter_diagonal_distance_command - analogRead(PROX_SENSOR_DR_PIN); // Then the error is calculated using back diagonal right sensor 
  corrector = proportional_coefficient * error; // Proportional correction
  leftSpeed = speed_command - corrector; 
  rightSpeed = speed_command + corrector;

}

void constrain_speed(){
  // We constrain leftSpeed and rightSpeed between 0 and 255 (because otherwise these values can be out of bounds due to our corrector)
  if (leftSpeed<0){
    leftSpeed = 0;
  }
  else if (leftSpeed>255){
    leftSpeed = 255;
  }
  if (rightSpeed<0){
    rightSpeed = 0;
  }
  else if (rightSpeed>255){
    rightSpeed = 255;
  }
}

void loop() {

  if (state!=FINISH){ // As long as the robot hasn't solved the maze, the programm is running

    if(strategy==FOLLOWING_BORDER){ // Our first strategy is following the left wall to follow the border
      leftWall_PID();
    }

    if(strategy==EXPLORING_CENTER_WALLS){  // Strategy : Exploring the center of the maze by switching followed wall 
      switchFollowingWall(); // We verify if we need to switch followed wall

      if(following_left_wall){ // In case following_left_wall is true, we follow the left wall
        leftWall_PID(); //we apply our left wall PID
      }
      else{ // otherwise we follow the right wall
        rightWall_PID();
      }
    }

    while(strategy==RANDOM){ // Random strategy : we simply move forward without correction and turn when a wall is detected
      move(255, 255);
      while (analogRead(PROX_SENSOR_FL_PIN)<750){ // If a wall is detected in front of the robot
        move(-255, 255); // the robot turns on itself
      }
    }

    while(analogRead(PROX_SENSOR_FL_PIN)<750 || digitalRead(PROX_SENSOR_FR_PIN)==1 ){ // While we detect a wall forward, the robot turns on itself
      switch(following_left_wall){ // The rotation direction depends on the wall the robot is following
        case true:
          move(255,-255);
          break;
        default:
          move(-255, 255);
          break;
      }
    }

    
    constrain_speed(); // We constrain the speeds between -255 and 255
    move(leftSpeed, rightSpeed); // We move in straight line with our corrected speeds

    light_sensor_value = analogRead(LIGHT_SENSOR_PIN); // We read the value of our light sensor

    if (state==SEARCHING_BLACK_CELL){ // If we are searching for black cell then we verify if we have found it
      if (black_duration() > black_cell_time_threshold){ // If we detect a black cell
        state=SEARCHING_RED_CELL; // now we are searching for the red cell
        Serial.println("BLACK CELL FOUND");
      }
    }

    if (state==SEARCHING_RED_CELL){ // If we are searching for red cell then we verify if we have found it
      if (red_duration() > red_cell_time_threshold){ // if we've found the red cell
      Serial.println("FINISH");
      state=FINISH; // we reached the final state : finish
      move(0,0); // stop the robot
      }
    }

    


    

    else if (strategy == FOLLOWING_BORDER && red_duration() > red_cell_time_threshold){
      strategy=EXPLORING_CENTER_WALLS;
    }

    else if (millis()-300000>=0){ // After 3 minutes if we still haven't solved the maze we go full random.
      strategy=RANDOM;
    }
  
  }
}
