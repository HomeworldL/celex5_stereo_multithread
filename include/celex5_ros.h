#pragma once

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>
#include <celextypes.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <pthread.h>  
#include <celex5_msgs/Event.h>
#include <celex5_msgs/EventVector.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include <unistd.h>
#include <signal.h>
#include <termio.h>
#include <stdio.h>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// #include <math.h>
// #include <stdlib.h>
// #include <deque>
// #include <string>
// #include <vector>
// #include <chrono>
// #include <fstream>
// #include <iomanip>
// #include <unistd.h>

#define MAT_ROWS 800
#define MAT_COLS 1280