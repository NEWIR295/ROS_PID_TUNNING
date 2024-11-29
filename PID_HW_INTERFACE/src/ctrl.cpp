/*
  File Name: ctrl.cpp
  Author: Mohamed Newir
  File Description: source file for the control unit
*/

/* used libraries */
#include "ctrl.h"
#include "std_msgs/Int64.h"

/// @//set RTOS parameters
unsigned long prevTime = 0; // init prev time with 0

// configure encoder
volatile long encoderCount = 0;

// glopal var to take pwm for motors
std_msgs::Float32 inputPwm;

// subscriber cb for motor speed
void callback(const std_msgs::Float32 &velMsg)
{
    inputPwm = velMsg;
}
// sub to "/velWpid" topic for pwm data
ros::Subscriber<std_msgs::Float32> sub("/velWpid", &callback);

// global var to take encoder values
std_msgs::Float32 msg;
// pub for angular velocity for motors
ros::Publisher pub("/encoder", &msg);
std_msgs::Int64 msgenc;
// pub for angular velocity for motors
ros::Publisher pubenc("/encodercount", &msgenc);

// init for motor & encoder
void ctrlInit(void)
{

    // sub & pub
    nh.advertise(pub);

    // nh.advertise(current_pub);
    nh.subscribe(sub);
    nh.advertise(pubenc);

    // motor init
    pinMode(pinDir, OUTPUT);
    pinMode(pinPwm, OUTPUT);
    // init motor with speed = 0
    analogWrite(pinPwm, 0);

    // encoder pin setups
    pinMode(encoderA, INPUT_PULLUP);
    pinMode(encoderB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, RISING);
}

void mororCtrl(void)
{

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {                                         // Read the position in an atomic block to avoid a potential misread if the interrupt coincides with this code running
        unsigned long currentTime = millis(); // get the current time
        if (currentTime - prevTime > interval)
        { // check if enough time has elapsed since the last interrupt
            // meassuring angular velocity for  & right motors
            double motorSpeed = double((encoderCount / PPR) * (2 * PI) * (1000.0 / interval)); // Revs per second to radians per second
            double motorSpeed_rpm = (motorSpeed * 60.0) / (2.0 * PI);

#if (MOTOR_DRIVER == L298N)
            /*
                if L298 motor driver is used
            */
            digitalWrite(pinDir, HIGH);
            analogWrite(pinPwm, abs(inputPwm.data));
            
#elif (MOTOR_DRIVER == CYTRON)
            /*
                if cytron motor driver used
            */

            setSpeed(inputPwm.data, pinDir, pinPwm); // set Motor with input speed.
#endif
            /* publishing encoder angular velocity msgs for pid feedback */
            msg.data = motorSpeed_rpm;
            pub.publish(&msg);

            /*  publishing encoder ticks msg */
            msgenc.data = encoderCount;
            pubenc.publish(&msgenc);

            // reset counts to 0
            encoderCount = 0;
            prevTime = currentTime;
        }
    }
}

/* if cytron motor driver used, set motor speed */
void setSpeed(int pwm, int dirPin, int PwmPin)
{
    digitalWrite(dirPin,HIGH);        
    analogWrite(PwmPin,abs(pwm));
}

// update encoder
void updateEncoder(void)
{
    if (digitalRead(encoderB) > 0)
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }
}
