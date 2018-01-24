#ifndef COMMUNICATION_MODULES_ALL_H
#define COMMUNICATION_MODULES_ALL_H


//#define ROS_CONTROL

#ifdef ROS_CONTROL
  #include "mwoibn/communication_modules/custom_controller.h"
  #include "mwoibn/communication_modules/velocity_controller.h"
#endif

#include "mwoibn/communication_modules/ros_controller.h"
#include "mwoibn/communication_modules/ros_feedback.h"
#include "mwoibn/communication_modules/ros_operational_euler.h"

#endif
