#pragma once

#include <iostream>
#include "ros/ros.h"
#include <unistd.h>
#include <math.h>
#include <algorithm>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>

#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
