#include "robot_catch.h"

aubo_robot_namespace::wayPoint_S G_waypoint;

////////////// Class ////////////////

 
RobotCatch::RobotCatch(ros::NodeHandle &nh)
{
    node_ = nh;
    move_change = 0;

    // catch_sub_ = node_.subscribe(
    //     "celex5_stereo/catch_pose", 10, &RobotCatch::PoseCallback, this);    

    //堆区开辟ServiceInterface
    send_robotService = new ServiceInterface();
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    //初始化参数
    m_userCoord.coordType = aubo_robot_namespace::BaseCoordinate;
    // 设置工具端参数
    m_toolDesc.toolInEndPosition.x = 0;
    m_toolDesc.toolInEndPosition.y = 0;
    m_toolDesc.toolInEndPosition.z = 0.12;//网兜相对于z（x）轴的偏移量，这个时候取的是进入网的那个平面中心点
    m_toolDesc.toolInEndOrientation.w = 0.9238;//四元数
    m_toolDesc.toolInEndOrientation.x = 0;
    m_toolDesc.toolInEndOrientation.y = 0;
    m_toolDesc.toolInEndOrientation.z = -0.3826;
    // 初始工具目标位置参数
    m_init_tool_position.x = -0.2579;
    m_init_tool_position.y = -0.2375;
    m_init_tool_position.z = 0.5627;
    m_init_tool_ori.w = 0.36114;
    m_init_tool_ori.x = 0.70044;
    m_init_tool_ori.y = -0.53501;
    m_init_tool_ori.z = -0.3045;

    send_robotService->userToBaseCoordinate(m_init_tool_position,    //基于用户座标系的工具末端位置信息
                                     m_init_tool_ori, //基于用户座标系的工具末端姿态信息
                                     m_userCoord,  //用户坐标系
                                     m_toolDesc,                 //工具参数                          
                                     m_init_flange_position,    //基于基座标系的法兰盘中心位置信息，共有
                                     m_init_flange_ori  //基于基座标系的法兰盘中心姿态信息，共有
                                     );
    // 防滑实验                      
    m_init_flange_position.x = 0.1138;
    m_init_flange_position.y = -0.3583;
    m_init_flange_position.z = 0.4298;
    m_init_flange_ori.w = 0;
    m_init_flange_ori.x = -0.9239;
    m_init_flange_ori.y = -0.3827;
    m_init_flange_ori.z = 0;

    std::cout<<"-----------------------------------------------"<<std::endl
      <<"法兰盘参数："<<std::endl
      <<"x"<<m_init_flange_position.x<<std::endl
      <<"y"<<m_init_flange_position.y<<std::endl
      <<"z"<<m_init_flange_position.z<<std::endl
      <<"ow"<<m_init_flange_ori.w<<std::endl
      <<"ox"<<m_init_flange_ori.x<<std::endl
      <<"oy"<<m_init_flange_ori.y<<std::endl
      <<"oz"<<m_init_flange_ori.z<<std::endl;

    std::cout<<"-----------------------------------------------"<<std::endl;

    // double startPointJointAngle[6] = {0};//逆运动学的一个参数 ，必须要有
    send_robotService->robotServiceRobotIk(startPointJointAngle,                                                     
                                           m_init_flange_position,
                                           m_init_flange_ori, 
                                           m_init_flange_waypoint);//得到抓捕点的关节角度
    // InvKinetic(m_init_flange_position, m_init_flange_ori, m_init_flange_waypoint);

    std::cout<<"-----------------------------------------------"<<std::endl
      <<"逆解关节参数："<<std::endl
      <<"第一个关节："<<m_init_flange_waypoint.jointpos[0]*180/PI<<std::endl
      <<"第二个关节："<<m_init_flange_waypoint.jointpos[1]*180/PI<<std::endl
      <<"第三个关节："<<m_init_flange_waypoint.jointpos[2]*180/PI<<std::endl
      <<"第四个关节："<<m_init_flange_waypoint.jointpos[3]*180/PI<<std::endl
      <<"第五个关节："<<m_init_flange_waypoint.jointpos[4]*180/PI<<std::endl
      <<"第六个关节："<<m_init_flange_waypoint.jointpos[5]*180/PI<<std::endl;
    std::cout<<"-----------------------------------------------"<<std::endl;

    //login and robot startup
    /** login ***/
    ret = send_robotService->robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"login success"<<std::endl;
    }
    else
    {
        std::cerr<<"login failed."<<std::endl;
        return;
    }

    /** startup　**/
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = send_robotService->rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"robot startup success."<<std::endl;
    }
    else
    {
        std::cerr<<"robot startup failed."<<std::endl;
        return;
    }

    /** 接口调用: 初始化运动属性 ***/
    send_robotService->robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 150.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 150.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 150.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 150.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 150.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 150.0/180.0*M_PI;   //接口要求单位是弧度
    send_robotService->robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 800.0/180.0*M_PI;   //接口要求单位是弧度
    send_robotService->robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 3;   //单位米每秒
    send_robotService->robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
    send_robotService->robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 1.2;   //单位米每秒
    send_robotService->robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    send_robotService->robotServiceGetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

    send_robotService->robotServiceRegisterRealTimeRoadPointCallback(RobotCatch::RealTimeWaypointCallback, NULL);
    usleep(1000);//稍微等一下

    // check macsize
    m_checkMacSize = new checkMacData(0);
    // ///////pthread_create(&check_mac_thread_, NULL, &checkMacData::checkRobotMacSize, NULL);
    check_mac_thread_ = std::thread(&checkMacData::checkRobotMacSize, m_checkMacSize);
    check_mac_thread_.detach();
    
    int tmp_macsize = m_checkMacSize->getData();
    std::cout << "macTargetPosDataSize child thread(in main): " << tmp_macsize<<std::endl;


    //在构造函数中运动到初始位置
    // ret_init = send_robotService->robotMoveLineToTargetPosition(m_userCoord, m_init_tool_position, m_toolDesc);   //关节运动至准备点
    ret_init = send_robotService->robotServiceJointMove(m_init_flange_waypoint,  true);

    usleep(1000*1000);//稍微等一下
    // std::cerr<<"休息好了"<<std::endl;

    if(ret_init != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      std::cerr<<"运动初始化失败.　ret:"<<ret_init<<std::endl;
    }
    else
    {
      std::cerr<<"运动初始化成功.　ret:"<<ret_init<<std::endl;
    }

    //enter Tcp2Can Mode
    // send_robotService->robotServiceEnterTcp2CanbusMode();
    usleep(1000*1000);//稍微等一下

    // 到这都没有问题

    // move_right_left_thread_ = std::thread(&RobotCatch::MoveRightLeft, this);
    // check_mac_thread_.detach();
}

  RobotCatch::~RobotCatch()
{
    // LEAVE Tcp2Can Mode
    usleep(1000*1000);
    send_robotService->robotServiceLeaveTcp2CanbusMode();
    usleep(2000*1000);

    /** 接口调用: 初始化运动属性 ***/
    send_robotService->robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 110.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 110.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 110.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 110.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 110.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 110.0/180.0*M_PI;   //接口要求单位是弧度
    send_robotService->robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 800.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 800.0/180.0*M_PI;   //接口要求单位是弧度
    send_robotService->robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //在析构函数中运动到初始位置
    // ret_end = send_robotService->robotMoveLineToTargetPosition(m_userCoord, m_init_tool_position, m_toolDesc, true);   //关节运动至准备点
    // ret_end = send_robotService->robotServiceJointMove(m_init_flange_waypoint,  true);
    if(ret_end != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"析构时回到初始化构型失败.　ret:"<<ret_end<<std::endl;
    }
    else
    {
        std::cerr<<"析构时回到初始化构型成功.　ret:"<<ret_end<<std::endl;
    }

    /** loginout　**/
    send_robotService->robotServiceLogout();

    delete send_robotService;
}

// 路点实时监测
void RobotCatch::RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    aubo_robot_namespace::wayPoint_S waypoint = *wayPointPtr;

    G_waypoint = waypoint;
}

void RobotCatch::PoseCallback(const geometry_msgs::PoseStamped &msg)
{
  std::cout<<"callback ready!"<<std::endl;

  if (msg.pose.position.x && msg.pose.position.y && msg.pose.position.z)
  {
    aubo_robot_namespace::wayPoint_S tmp_wayPoint;
    tmp_wayPoint.cartPos.position.x = msg.pose.position.x;
    tmp_wayPoint.cartPos.position.y = msg.pose.position.y;
    tmp_wayPoint.cartPos.position.z = msg.pose.position.z;
    tmp_wayPoint.orientation.w = msg.pose.orientation.w;
    tmp_wayPoint.orientation.x = msg.pose.orientation.x;
    tmp_wayPoint.orientation.y = msg.pose.orientation.y;
    tmp_wayPoint.orientation.z = msg.pose.orientation.z;
    m_vec_target_tool_waypoint.emplace_back(tmp_wayPoint);
    move_change = 1;
    std::cout<<"There is a catching pose!"<<std::endl;
    std::cout<<msg<<std::endl;

    MoveDown();
  }
  else
  {
    std::cout<<"no catching pose!"<<std::endl;
  }

}

// 利用透传模式实现左右两点来回小幅度运动，然后要写在一个线程函数里，随时给他上锁
void RobotCatch::MoveRightLeft()
{
  Eigen::VectorXd time;
  Eigen::VectorXd angle_sin;
  int size = 200;
  int duration = 1;
  int deltat = duration/size;
  int anglerange = 4;
  time = Eigen::VectorXd::LinSpaced(size, 0, duration-deltat);
  angle_sin = 2*PI*time.array()/duration;
  angle_sin = angle_sin.array().sin();
  angle_sin = angle_sin.array()*anglerange*PI/180;

  aubo_robot_namespace::wayPoint_S m_waypoint;
  aubo_robot_namespace::wayPoint_S m_waypoint_init;
  m_waypoint_init = G_waypoint;
  m_waypoint = G_waypoint;
  // std::cout<<angle_sin<<std::endl;
  std::cout<<"-----------------------------------------------"<<std::endl
      <<"初始关节参数："<<std::endl
      <<"第一个关节："<<m_waypoint_init.jointpos[0]*180/PI<<std::endl
      <<"第二个关节："<<m_waypoint_init.jointpos[1]*180/PI<<std::endl
      <<"第三个关节："<<m_waypoint_init.jointpos[2]*180/PI<<std::endl
      <<"第四个关节："<<m_waypoint_init.jointpos[3]*180/PI<<std::endl
      <<"第五个关节："<<m_waypoint_init.jointpos[4]*180/PI<<std::endl
      <<"第六个关节："<<m_waypoint_init.jointpos[5]*180/PI<<std::endl;
  std::cout<<"-----------------------------------------------"<<std::endl;
  
  int i = 0;
  int turn = 0;
  while (1)
  {
    

    // 左右运动
    m_waypoint.jointpos[0] = m_waypoint_init.jointpos[0] + angle_sin(i);
    // m_waypoint.jointpos[1] = m_waypoint_init.jointpos[1] + angle_sin(i)/2;
    // m_waypoint.jointpos[2] = m_waypoint_init.jointpos[2] + angle_sin(i)/2;
    m_waypoint.jointpos[3] = m_waypoint_init.jointpos[3] + angle_sin(i);
    m_waypoint.jointpos[4] = m_waypoint_init.jointpos[4] + angle_sin(i);

    if (move_change)
    {
      // 等待标志位改变，记录发送的最后一个点
      m_save_flange_waypoint = m_waypoint;
      std::cout << "move right and left thread child thread close: " << std::endl;
      // move_change = 1;
      //唤醒线程

      break;
    }
    // std::cout<<"-----------------------------------------------"<<std::endl
    //   <<"第一个关节："<<m_waypoint.jointpos[0]*180/PI<<std::endl
    //   <<"第二个关节："<<m_waypoint.jointpos[1]*180/PI<<std::endl
    //   <<"第三个关节："<<m_waypoint.jointpos[2]*180/PI<<std::endl
    //   <<"第四个关节："<<m_waypoint.jointpos[3]*180/PI<<std::endl
    //   <<"第五个关节："<<m_waypoint.jointpos[4]*180/PI<<std::endl
    //   <<"第六个关节："<<m_waypoint.jointpos[5]*180/PI<<std::endl;
    // std::cout<<"-----------------------------------------------"<<std::endl;
    MoveUsingTcp2Can(m_waypoint);
    // usleep(2000);
    std::cout<<"right and left move once"<<std::endl;

    i++;
    if (i == size)
    {

      i = 0;
      turn++;
      if (turn == 15)
      {
        std::cout << "move right and left thread child thread close: " << std::endl;
        move_change = 1;
        break;
      }
      
    }
  }

}
// test
void RobotCatch::MoveDown()
{
  aubo_robot_namespace::MoveRelative rel;
  rel.relativePosition[2] = -0.006;
  send_robotService->robotMoveLineToTargetPositionByRelative(m_userCoord,rel,true);
}

// 当收到要运动的目标点位姿后，改变全局标志位，左右运动被停止，并唤醒移动到目标点位置的程序
void RobotCatch::MoveToTarget()
{
  while (!move_change)
  {
    // 判断目标点是否非空，空则不改变标志位，非空则改变。标志位是要上锁的
    // std::cout<<"move_change: "<<move_change<<std::endl;
    usleep(100);

  }
  std::cout<<"move_change: "<<move_change<<std::endl;

  // if (1 !xxx.size())
  // {

  // }
  
  // // 线程等待/阻塞在这里，不然有可能左右运动还在动，条件变量

  // 快速运动到目标位置
  struct timeval t0, t1;
  double deltaT;
  gettimeofday(&t0, NULL);
  send_robotService->userToBaseCoordinate(m_vec_target_tool_waypoint[0].cartPos.position,    //基于用户座标系的工具末端位置信息
                                          m_vec_target_tool_waypoint[0].orientation, //基于用户座标系的工具末端姿态信息
                                          m_userCoord,  //用户坐标系
                                          m_toolDesc,                 //工具参数                          
                                          m_target_flange_position,    //基于基座标系的法兰盘中心位置信息，共有
                                          m_target_flange_ori  //基于基座标系的法兰盘中心姿态信息，共有
                                          );
  send_robotService->robotServiceRobotIk(startPointJointAngle,                                                     
                                         m_target_flange_position,
                                         m_target_flange_ori, 
                                         m_target_flange_waypoint);//得到抓捕点的关节角度
  aubo_robot_namespace::wayPoint_S m_waypoint_init;
  // usleep(1000);// 10 ms还没有动完
  // send_robotService->robotServiceGetCurrentWaypointInfo(m_waypoint_init);
  m_waypoint_init = G_waypoint;

  // m_waypoint_init = m_save_flange_waypoint;
  double param[6][6] = {0}; 
  std::cout<<"-----------------------------------------------"<<std::endl
      <<"初始关节参数："<<std::endl
      <<"第一个关节："<<m_waypoint_init.jointpos[0]*180/PI<<std::endl
      <<"第二个关节："<<m_waypoint_init.jointpos[1]*180/PI<<std::endl
      <<"第三个关节："<<m_waypoint_init.jointpos[2]*180/PI<<std::endl
      <<"第四个关节："<<m_waypoint_init.jointpos[3]*180/PI<<std::endl
      <<"第五个关节："<<m_waypoint_init.jointpos[4]*180/PI<<std::endl
      <<"第六个关节："<<m_waypoint_init.jointpos[5]*180/PI<<std::endl;
  std::cout<<"-----------------------------------------------"<<std::endl
      <<"抓捕点关节参数："<<std::endl
      <<"第一个关节："<<m_target_flange_waypoint.jointpos[0]*180/PI<<std::endl
      <<"第二个关节："<<m_target_flange_waypoint.jointpos[1]*180/PI<<std::endl
      <<"第三个关节："<<m_target_flange_waypoint.jointpos[2]*180/PI<<std::endl
      <<"第四个关节："<<m_target_flange_waypoint.jointpos[3]*180/PI<<std::endl
      <<"第五个关节："<<m_target_flange_waypoint.jointpos[4]*180/PI<<std::endl
      <<"第六个关节："<<m_target_flange_waypoint.jointpos[5]*180/PI<<std::endl;
  double duration = 0.4;
  for (int i=0; i<6; i++)
  {
    QuinticPolyPlan(m_waypoint_init.jointpos[i], m_target_flange_waypoint.jointpos[i],duration,param[i]);

    std::cout<<"-----------------------------------------------"<<std::endl
      <<"多项式系数："<<std::endl
      <<param[i][0]<<","<<param[i][1]<<","<<param[i][2]<<","<<param[i][3]<<","<<param[i][4]<<","<<param[i][5]<<std::endl;
  }
  gettimeofday(&t1, NULL);//计时器
  deltaT = (t1.tv_sec-t0.tv_sec)*1000000 + (t1.tv_usec-t0.tv_usec);//判断微秒
  std::cout<<"deltaT:"<<t1.tv_usec-t0.tv_usec<<std::endl;

                                         
  // 运动
  aubo_robot_namespace::wayPoint_S m_waypoint;
  int size = 800;// 请注意！ 这个size必须满足点的时间间隔靠近5ms，最好等于5ms
  for (int i = 1; i <= size; i++)
  {
    double t_move = double(i)/(size/duration);
    m_waypoint.jointpos[0] = param[0][0] + param[0][1]*t_move + param[0][2]*pow(t_move,2) +
                             param[0][3]*pow(t_move,3) + param[0][4]*pow(t_move,4) + param[0][5]*pow(t_move,5);
    m_waypoint.jointpos[1] = param[1][0] + param[1][1]*t_move + param[1][2]*pow(t_move,2) +
                             param[1][3]*pow(t_move,3) + param[1][4]*pow(t_move,4) + param[1][5]*pow(t_move,5);
    m_waypoint.jointpos[2] = param[2][0] + param[2][1]*t_move + param[2][2]*pow(t_move,2) +
                             param[2][3]*pow(t_move,3) + param[2][4]*pow(t_move,4) + param[2][5]*pow(t_move,5);
    m_waypoint.jointpos[3] = param[3][0] + param[3][1]*t_move + param[3][2]*pow(t_move,2) +
                             param[3][3]*pow(t_move,3) + param[3][4]*pow(t_move,4) + param[3][5]*pow(t_move,5);
    m_waypoint.jointpos[4] = param[4][0] + param[4][1]*t_move + param[4][2]*pow(t_move,2) +
                             param[4][3]*pow(t_move,3) + param[4][4]*pow(t_move,4) + param[4][5]*pow(t_move,5);
    m_waypoint.jointpos[5] = param[5][0] + param[5][1]*t_move + param[5][2]*pow(t_move,2) +
                             param[5][3]*pow(t_move,3) + param[5][4]*pow(t_move,4) + param[5][5]*pow(t_move,5);
    MoveUsingTcp2Can(m_waypoint);
    std::cout<<"-----------------------------------------------"<<std::endl
      <<"第一个关节："<<m_waypoint.jointpos[0]*180/PI<<std::endl
      <<"第二个关节："<<m_waypoint.jointpos[1]*180/PI<<std::endl
      <<"第三个关节："<<m_waypoint.jointpos[2]*180/PI<<std::endl
      <<"第四个关节："<<m_waypoint.jointpos[3]*180/PI<<std::endl
      <<"第五个关节："<<m_waypoint.jointpos[4]*180/PI<<std::endl
      <<"第六个关节："<<m_waypoint.jointpos[5]*180/PI<<std::endl
      <<"路点个数："<<i<<std::endl
      <<"路点时间："<<t_move<<std::endl;
    // usleep(1000);                                                                              

  }
  

}

// 透传模式
void RobotCatch::MoveUsingTcp2Can(aubo_robot_namespace::wayPoint_S &waypoint)
{
  //m_waypoint.jointpos[0] = m_waypoint.jointpos[0] + 0.00008*i;   // increase 0.11° at a time(you must adjust that use your trajectory planning data ,that only a demo)
  // note: The Joint Angle you send, the corresponding velocity and acc can`t more than rate speed and acc ,otherwise the robot shutdown and poweroff
  // setup3:check Mac Size , stop setRobotPosData2Canbus if more than 200
  
  //i think the reason why he uses 200 is 200*5=1s, so we can change the value of the tmp_macsize
   //send your translate data by this API(set it less than 5ms at a time)
  int tmp_macsize = m_checkMacSize->getData();
  std::cout << "macTargetPosDataSize move thread: " << tmp_macsize  << std::endl;
  if (tmp_macsize < 200)
  {
    send_robotService->robotServiceSetRobotPosData2Canbus(waypoint.jointpos);
  }
  
}

// 


///////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 规划算法2：关节空间轨迹规划——五次多项式
 * 适用：
 * 这里固定了初末加速度和速度
 * @param	waypoint_1
 * @param waypoint_2
 * @param Time
 * @param A
 * @outpara   A
*/
void RobotCatch::QuinticPolyPlan(double jointpos_0,
                                 double jointpos_1,
                                 double Time,
                                 double *A)
{
    T = Time;
    //轨迹规划参数
    quintic_TM << 1, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0,
                  0, 0, 2, 0, 0, 0,
                  1, T, pow(T,2), pow(T,3), pow(T,4), pow(T,5),
                  0, 1, 2*T, 3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
                  0, 0, 2, 6*T, 12*pow(T,2), 20*pow(T,3);
    quintic_Q << jointpos_0, 0, 0, jointpos_1, 0, 0;//列向量

    //可以用稀疏矩阵lu(),但不太会
    quintic_A = quintic_TM.inverse()*quintic_Q;

    A[0] = quintic_A(0);
    A[1] = quintic_A(1);
    A[2] = quintic_A(2);
    A[3] = quintic_A(3);
    A[4] = quintic_A(4);
    A[5] = quintic_A(5);

}

/**
 * @brief   根据关节位置和姿态求解逆运动学(使用与末端为R91的圆环，有45°翻转)
 * @param   pose    目标位置
 * @param   R	   目标姿态矩阵
 * @return  m_jointpos 关节角度
 * @use 	solve_inv_kin(&pbar, &RR, &end_joint);
 */
// void RobotCatch::InvKinetic(const Pose &pose, const Eigen::Matrix3d &R, aubo_robot_namespace::JointParam &m_jointparam_used)
void RobotCatch::InvKinetic(const aubo_robot_namespace::Pos &pose, 
                            const aubo_robot_namespace::Ori &ori, 
                            aubo_robot_namespace::wayPoint_S &waypoint)
{
  // 四元数转旋转矩阵
  Eigen::Quaterniond quat(ori.x,ori.y,ori.z,ori.w);
  Eigen::Matrix3d R = quat.matrix();

  //旋转矩阵数据提取,nx\ny\nz是旋转矩阵的第一列，三列按照noa排列，其中nz oz计算过程中没用到
  double nx;
  double ny;
  double ox;
  double oy;
  double ax;
  double ay;
  double az;

  nx = (double)R(0, 0);
  ny = (double)R(1, 0);
  ox = (double)R(0, 1);
  oy = (double)R(1, 1);
  ax = (double)R(0, 2);
  ay = (double)R(1, 2);
  az = (double)R(2, 2);

  //位置点数据提取
  double px;
  double py;
  double pz;
  px = pose.x;
  py = pose.y;
  pz = pose.z;

  //其他初值量
  double a2;
  double a3;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;


  a2 = 0.408;
  a3 = 0.376;
  d1 = 0.0985+0.51;	//机械臂底座0.51
  d2 = 0.1405;
  d3 = 0.1215;
  d4 = 0.1025;
  d5 = 0.1025;
  d6 = 0.091+0.15;	//网兜+0.094，robotiq+0.15

  //6个theta角及中间变量
  double theta1;
  double theta2;
  double theta3;
  double theta4;
  double theta5;
  double theta6;
  double theta234;
  double theta23;
  double N1;
  double M1;
  double C1;
  double A;
  double B;


  //按公式解算部分
  A = px - (d6 * ax);
  B = py - (d6 * ay);

  theta1 = 2 * atan2((A + (sqrt( pow(A,2) + pow(B,2) - pow((d2 - d3 + d4),2)))), (-1*B + d2 - d3 + d4));
  theta5 = -acos(ax * sin(theta1) - ay * cos(theta1));
  theta6 = atan2((ox * sin(theta1) - oy * cos(theta1)) / sin(theta5), (ny * cos(theta1) - nx * sin(theta1)) / sin(theta5)) + M_PI / 4;
  theta234 = atan2(az / sin(theta5), (ax * cos(theta1) + ay * sin(theta1)) / sin(theta5));
  M1 = px * cos(theta1) + py * sin(theta1) - d6 * cos(theta234) * sin(theta5) + d5 * sin(theta234);
  N1 = pz - d1 - d6 * sin(theta234) * sin(theta5) - d5 * cos(theta234);
  //ROS_INFO_STREAM("pz"<<pz<<"d1:"<<d1<<"d6:"<<d6<<"theta234:"<<theta234<<"s234:"<<sin(theta234)<<"s5"<<sin(theta5)<<"d5"<<d5<<"c234:"<<cos(theta234));
  C1 = (pow(M1,2) + pow(N1,2) + pow(a2,2) - pow(a3,2)) / (2 * a2);
  theta2 = 2 * atan2(N1 - sqrt(pow(N1,2) + pow(M1,2) - pow(C1,2)), (C1 + M1)) - M_PI / 2;
  theta23 = atan2((N1 - a2 * sin(theta2 + M_PI / 2)), (M1 - a2 * cos(theta2 + M_PI / 2)));
  theta4 = theta234 - theta23 + M_PI / 2;
  theta3 = theta2 + M_PI / 2 - theta23;

  //ROS_INFO_STREAM("A:"<<A<<"/n B:"<<B<<"/n N1:"<<N1<<"/n M1"<<M1<<"/n C1"<<C1<<"/n theta23"<<theta23<<"/n theta234:"<<theta234);

  //将输出的角度限制在±π之间
  if (theta1 >M_PI)
    theta1 = theta1 - 2*M_PI;
  else if (theta1 <(-1*M_PI))
    theta1 = theta1 + 2*M_PI;

  if ((theta2) > M_PI)
    theta2 = theta2 - 2 * M_PI;
  else if ((theta2) < (-1 * M_PI))
    theta2 = theta2 + 2 * M_PI;

  if ((theta3) > M_PI)
    theta3 = theta3 - 2 * M_PI;
  else if ((theta3) < (-1 * M_PI))
    theta3 = theta3 + 2 * M_PI;

  if ((theta4) > M_PI)
    theta4 = theta4 - 2 * M_PI;
  else if (theta4 < (-1 * M_PI))
    theta4 = theta4 + 2 * M_PI;

  if ((theta5) > M_PI)
    theta5 = theta5 - 2 * M_PI;
  else if ((theta5) < (-1 * M_PI))
    waypoint.jointpos[4] = theta5 + 2 * M_PI;

  if ((theta6) > M_PI)
    theta6 = theta6 - 2 * M_PI;
  else if ((theta6) < (-1 * M_PI))
    theta6 = theta6 + 2 * M_PI;

  waypoint.jointpos[0] = theta1 / M_PI * 180;
  waypoint.jointpos[1] = theta2 / M_PI * 180;
  waypoint.jointpos[2] = theta3 / M_PI * 180;
  waypoint.jointpos[3] = theta4 / M_PI * 180;
  waypoint.jointpos[4] = theta5 / M_PI * 180;
  waypoint.jointpos[5] = theta6 / M_PI * 180;

}
//////////////////////////////////////////////////////////////////////////////////////////
// RobotCatch *rc =
//       new RobotCatch();


int main(int argc, char **argv) {
  ros::init(argc, argv, "celex5_stereo_multithread_robot_catch");
  ros::NodeHandle node_;

  // RobotCatch *rc =
  //     new RobotCatch(node_);

  RobotCatch rc(node_);
  rc.catch_sub_ = node_.subscribe(
        "celex5_stereo/catch_pose", 10, &RobotCatch::PoseCallback, &rc);
  // std::cout<<"我的回调函数去哪了？？？"<<std::endl;
  struct timeval t0, t1;
  double deltaT;

  pthread_t move_right_left_thread_;
  pthread_t move_to_target_thread_;
  int ret;
  void *RobotMoveRightLeft(void *arg);
  void *RobotMoveToTarget(void *arg);

  // std::cout << "main() : 创建机械臂往复运动线程" << std::endl;      
  // ret = pthread_create(&move_right_left_thread_, NULL, RobotMoveRightLeft, (void *)&rc);
  // if (ret){
  //     std::cout << "Error:无法创建机械臂往复运动线程" << std::endl;
  //     exit(-1);
  // }
  // pthread_detach(move_right_left_thread_);

  // std::cout << "main() : 创建机械臂等待抓捕线程" << std::endl;      
  // ret = pthread_create(&move_to_target_thread_, NULL, RobotMoveToTarget, (void *)&rc);
  // if (ret){
  //     std::cout << "Error:无法创建机械臂等待抓捕线程" << std::endl;
  //     exit(-1);
  // }
  // pthread_detach(move_to_target_thread_);

  // rc.MoveRightLeft();
  gettimeofday(&t0, NULL);//总计时器，超过一定数目，暂停
  ros::MultiThreadedSpinner s(2);

  ros::Rate loop_rate(1000);
  while (node_.ok()) {
    ros::spinOnce();
    // std::cout<<"spining!"<<std::endl;
    loop_rate.sleep();
    gettimeofday(&t1, NULL);//计时器
    deltaT = (t1.tv_sec-t0.tv_sec);//只看s
    if (deltaT > 30)//0s
    {
      // delete rc;
      // rc.MoveDown();
      sleep(1);
      ROS_INFO("Program break down: time too long!");
      
      ros::shutdown();
      return 0;
    }
  }

  // delete rc;

  return EXIT_SUCCESS;
}


void *RobotMoveRightLeft(void *arg){
    RobotCatch *rc;
    rc = (RobotCatch *) arg;
    rc->MoveRightLeft();
    pthread_exit(NULL);
}

void *RobotMoveToTarget(void *arg){
    RobotCatch *rc;
    rc = (RobotCatch *) arg;
    // 最后都是要去掉的，是根据内部的来。
    // aubo_robot_namespace::Pos target_tool_position;
    // aubo_robot_namespace::Ori target_tool_ori;

    // target_tool_position.x = -0.3651;
    // target_tool_position.y = 0.2611;
    // target_tool_position.z = 0.5779;
    // target_tool_ori.w = 0.1889;
    // target_tool_ori.x = -0.8043;
    // target_tool_ori.y = 0.0784;
    // target_tool_ori.z = 0.5579;

    rc->MoveToTarget();
    pthread_exit(NULL);
}