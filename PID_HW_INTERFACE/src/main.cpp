/*
  File Name: main.cpp
  Author: Mohamed Newir
  File Description: initialize ROS node
*/

#include <Arduino.h>
#include "ctrl.h"

ros::NodeHandle nh;

void setup()
{
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(38400);
  nh.initNode();
  ctrlInit();
}

void loop()
{
  // put your main code here, to run repeatedly:
  mororCtrl();
  nh.spinOnce();
}
