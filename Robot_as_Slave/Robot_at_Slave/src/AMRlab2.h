// Header file for lab 2
// All tasks can be completed in this file

#define ENCODERTIME 20      // update time for encoder update in ms
#define CONTROLTIME 20      // update time for PID control loop in ms

const float Kp = 8;         // proportional factor, start here first
const float Ki = 150;       // integral factor, tune as second factor
const float Kd = 0.001;         // differential factor, tune last

int16_t maxSpeed = 400;  // maximum speed for the PID controller output
int16_t minSpeed = 5;    // minimum speed, everthing below will be set to zero

// calibrate here to centimeter per second
const float calibLeft = 1;//.353;
const float calibRight = 1;//.353;

enum {                   // states of the robot state machine
  stStopped,             // motors are stopped, wait for button
  stTuning,              // run the tuning setup 
  stTrajectory           // follow the trajectory
};

struct trajectory_t {
  float leftSpeed;
  float rightSpeed;
  uint16_t segmentTime;
};

const uint8_t segmentsNo = 5; // length of the trajectory
uint8_t currentSegment = 0;

trajectory_t tuning = {10,10,2000}; // edit if necessary

trajectory_t trajectory[segmentsNo] = {  // format {left,right,time}
  {10,10,2000},
  {12.42,7.58,2000}, 
  {10,10,5000},
  {7.58,12.42,2500},
  {10,10,4000}
};

//trajectory_t trajectory[segmentsNo] = {  // format {left,right,time}
//  {20,20,1000},
//  {12.42*2,7.58*2,1000}, 
//  {20,20,2500},
//  {7.58*2,12.42*2,1250},
//  {20,20,2000}
//};

//trajectory_t trajectory[segmentsNo] = {  // format {left,right,time}
//  {10,10,4000},
//  {10,0,1333}, 
//  {10,10,4000},
//  {10,0,1333},
//  {10,10,4000},
//  {10,0,1333},
//  {10,10,4000},
//  {10,0,1333}
//};
