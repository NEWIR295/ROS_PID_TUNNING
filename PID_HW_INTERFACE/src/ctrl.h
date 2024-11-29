/*
  File Name: ctrl.h
  Author: Mohamed Newir
  File Description: header file for the control unit
*/

#ifndef  myCtrl
#define  myCtrl

#include <Arduino.h>
#include <ros.h>
#include <util/atomic.h>
#include <std_msgs/Float32.h>

//set RTOS parameters
#define interval 600.0 //set rtos intervals

//encoder pins
#define encoderA 2 //WHITE
#define encoderB 3 //GREEN
#define PPR 440.0

enum motorDriver { CYTRON, L298N };
#define MOTOR_DRIVER  L298N /* CYTRON or L298N */

//motor driver pins
#define pinDir 8// dir motor = Pin 8
#define pinPwm 6 // PWM motor = Pin 9
extern ros::NodeHandle nh; // Declare the nh object as external

//ctrl func prototypes
void mororCtrl(void); //set motors speed
void ctrlInit(void); //encoder & motor init
void updateEncoder(void);
void setSpeed(int pwm, int dirPin, int PwmPin);//set motor speed

#endif