#pragma once

#include <iostream>
#include "ros/ros.h"
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>  
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include <pthread.h>  
#include <thread>



#define SERVER_HOST "192.168.1.100"
#define SERVER_PORT 8899
#define PI 3.1415926

////////////////////////////////////////////////////////////////////////////////////
class checkMacData
{
public:
    checkMacData(uint16 m_macSize)
    {
        receive_robotService = new ServiceInterface();

        int ret = receive_robotService->robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"receive login success"<<std::endl;
        }
        else
        {
            std::cerr<<"receive  login failed."<<std::endl;
            return;
        }

        m_macSize=0;
    }


    ~checkMacData()
    {
        delete receive_robotService;
        pthread_mutex_destroy(&mac_lock);
    }

    uint16 getData()
    {
        pthread_mutex_lock(&mac_lock);
        auto tmp = m_macSize;
        pthread_mutex_unlock(&mac_lock);
        return tmp;
    }

public:

    // Note：interface board buffer is 1000，but you should do that
    // send data when the buffer less than 200 ,otherwise stop send
    // void *checkRobotMacSize(void *arg)
    void checkRobotMacSize()
    {
        aubo_robot_namespace::RobotDiagnosis robotdiagnosis;
        receive_robotService->robotServiceGetRobotDiagnosisInfo(robotdiagnosis);
        while(!robotdiagnosis.macDataInterruptWarning){
            receive_robotService->robotServiceGetRobotDiagnosisInfo(robotdiagnosis);
            // lock
            pthread_mutex_lock(&mac_lock);
            m_macSize = robotdiagnosis.macTargetPosDataSize;
            pthread_mutex_unlock(&mac_lock);

            // std::cout << "macTargetPosDataSize child thread: " << m_macSize  << std::endl;
            usleep(1000);
        }
        std::cout << "macTargetPosDataSize child thread close: " << m_macSize  << std::endl;
    }

private:
    ServiceInterface *receive_robotService;

    uint16 m_macSize;
    pthread_mutex_t mac_lock = PTHREAD_MUTEX_INITIALIZER;
    // pthread_t check_mac_thread_;
};

////////////////////////////////////////////////////////////////////////////////////
class RobotCatch{
public:
    int move_change;

    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;

    // aubo_robot_namespace::wayPoint_S m_waypoint;

    //获取当前关节参数（不是实时的
    // aubo_robot_namespace::wayPoint_S m_waypoint_current;

    //抓捕点参数
    aubo_robot_namespace::wayPoint_S m_target_flange_waypoint;
    aubo_robot_namespace::Pos m_target_flange_position;
    aubo_robot_namespace::Ori m_target_flange_ori;
    std::vector<aubo_robot_namespace::wayPoint_S> m_vec_target_tool_waypoint;
    aubo_robot_namespace::Pos m_target_tool_position;
    aubo_robot_namespace::Ori m_target_tool_ori;

    //轨迹规划出的路点（透传连续
    // aubo_robot_namespace::wayPoint_S m_tcp2can_waypoint;

    //初始化参数
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool m_userCoord;
    aubo_robot_namespace::ToolInEndDesc m_toolDesc;
    aubo_robot_namespace::Pos m_init_tool_position;
    aubo_robot_namespace::Ori m_init_tool_ori;
    aubo_robot_namespace::Pos m_init_flange_position;
    aubo_robot_namespace::Ori m_init_flange_ori;
    aubo_robot_namespace::wayPoint_S m_init_flange_waypoint;

    aubo_robot_namespace::wayPoint_S m_save_flange_waypoint;

    int ret_init;//若初始化运动错误，则直接结束程序

    int ret_end;
    // int ret_IK;

    double startPointJointAngle[6] = {0};//逆运动学的一个参数 ，必须要有

    //轨迹规划用到的矩阵
    // 5次的
    Eigen::Matrix<double, 6, 6> quintic_TM;//时间矩阵
    Eigen::Matrix<double, 6, 1> quintic_A;//多项式系数
    Eigen::Matrix<double, 6, 1> quintic_Q;//位置，速度，加速度
    double T;

    RobotCatch(ros::NodeHandle &nh);
    ~RobotCatch();
    void MoveRightLeft();
    // void MoveToTarget(const aubo_robot_namespace::Pos &target_tool_position,
    //                   const aubo_robot_namespace::Ori &target_tool_ori);
    void MoveToTarget();
    void MoveDown();
    void QuinticPolyPlan(double jointpos_0,double jointpos_1, double Time, double *A);
    void BangBangPlan();
    void MoveUsingTcp2Can(aubo_robot_namespace::wayPoint_S &m_waypoint);
    static void RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg);
    void InvKinetic(const aubo_robot_namespace::Pos &pose, 
                            const aubo_robot_namespace::Ori &ori, 
                            aubo_robot_namespace::wayPoint_S &waypoint);
    void PoseCallback(const geometry_msgs::PoseStamped &msg);

    ros::Subscriber catch_sub_;
private:

    ServiceInterface *send_robotService;
    ros::NodeHandle node_;
    

    checkMacData *m_checkMacSize;
    // pthread_t check_mac_thread_;
    std::thread check_mac_thread_;
    std::thread move_right_left_thread_;
};

