#include "visual_servoing.h"

aubo_robot_namespace::wayPoint_S G_waypoint;

////////////// Class ////////////////

 
VisualServoing::VisualServoing(ros::NodeHandle &nh)
{
    node_ = nh;
    move_change = 0;
    flag = 0;

    srand(time(NULL));

    ////////////////////////////////////////////////////////////////

    aubo.link1.DH.a = 0;
    aubo.link1.DH.alpha = 0*PI/180;
    aubo.link1.DH.d = 121.5-23;
    aubo.link1.DH.theta = 0*PI/180;

    aubo.link2.DH.a = 0;
    aubo.link2.DH.alpha = 90*PI/180;
    aubo.link2.DH.d = 0;       
    // aubo.link2.DH.theta = theta1;

    aubo.link3.DH.a = 408;
    aubo.link3.DH.alpha = 180*PI/180;
    aubo.link3.DH.d = 140.5;   
    // aubo.link3.DH.theta = theta2;

    aubo.link4.DH.a = 376;
    aubo.link4.DH.alpha = 180*PI/180;
    aubo.link4.DH.d = 121.5;
    // aubo.link4.DH.theta = theta3;

    aubo.link5.DH.a = 0;   //0
    aubo.link5.DH.alpha = -90*PI/180;
    aubo.link5.DH.d = 102.5;
    // aubo.link5.DH.theta = theta4;

    aubo.link6.DH.a = 0;
    aubo.link6.DH.alpha = 90*PI/180;
    aubo.link6.DH.d = 102.5;
    // aubo.link6.DH.theta = theta5;

    aubo.link7.DH.a = 0;
    aubo.link7.DH.alpha = 0*PI/180;
    aubo.link7.DH.d = 94;   //根据实际基座再改
    // aubo.link7.DH.theta = theta6;

    J_tool << -0.9994,    0.0356,   -0.0033,   -0.0038,   -0.0985,    0.0811,
              -0.0356,   -0.9994,    0.0001,    0.0985,   -0.0035,    0.0030,
              -0.0033,    0.0002,    1.0000,    0.0811,   -0.0003,    0.0003,
               0,         0,         0,        -0.9994,    0.0356,   -0.0033,
               0,         0,         0,        -0.0356,   -0.9994,    0.0001,
               0,         0,         0,        -0.0033,    0.0002,    1.0000;

    feature_star[0] = 640;
    feature_star[1] = 400;
    feature_save[0] = 640;
    feature_save[1] = 400;
    feature_velocity_star(0,0) = 0.2;
    feature_velocity_star(1,0) = 0.2;

    double theta[6]; 
    theta[0] = -53.523*PI/180;
    theta[1] = 17.257*PI/180;
    theta[2] = 101.814*PI/180;
    theta[3] = -5.443*PI/180;
    theta[4] = 90*PI/180;
    theta[5] = -8.524*PI/180;
    Eigen::Matrix<double, 6, 6> manipulator_jocab;
    manipulator_jocab = getJacobian(theta); // rad
    std::cout<<manipulator_jocab<<std::endl;

    // std::cout<<"full_jocab"<<std::endl;
    // std::cout<<J_tool * manipulator_jocab<<std::endl;

    // std::cout<<J_tool<<std::endl;

    // sleep(50);

    outfile.open("track_delay.txt", std::ios::out);   
    joint_outfile.open("joint_outfile.txt", std::ios::out);   



    ////////////////////////////////////////////////////////////////

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
    m_toolDesc.toolInEndPosition.z = 0;//事件相机
    m_toolDesc.toolInEndOrientation.w = 1;//事件相机 四元数
    m_toolDesc.toolInEndOrientation.x = 0;
    m_toolDesc.toolInEndOrientation.y = 0;
    m_toolDesc.toolInEndOrientation.z = 0;
    // 初始工具目标位置参数
    m_init_tool_position.x = 0.0847;
    m_init_tool_position.y = -0.5402;
    m_init_tool_position.z = 0.5602;
    m_init_tool_ori.w = 0.0;
    m_init_tool_ori.x = -0.9239;
    m_init_tool_ori.y = -0.3827;
    m_init_tool_ori.z = 0.0;

    send_robotService->userToBaseCoordinate(m_init_tool_position,    //基于用户座标系的工具末端位置信息
                                     m_init_tool_ori, //基于用户座标系的工具末端姿态信息
                                     m_userCoord,  //用户坐标系
                                     m_toolDesc,                 //工具参数                          
                                     m_init_flange_position,    //基于基座标系的法兰盘中心位置信息，共有
                                     m_init_flange_ori  //基于基座标系的法兰盘中心姿态信息，共有
                                     );

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

    // m_init_flange_waypoint.jointpos[0] = -53.523*PI/180;
    // m_init_flange_waypoint.jointpos[1] = 17.257*PI/180;
    // m_init_flange_waypoint.jointpos[2] = 101.814*PI/180;
    // m_init_flange_waypoint.jointpos[3] = -5.443*PI/180;
    // m_init_flange_waypoint.jointpos[4] = 90*PI/180;
    // m_init_flange_waypoint.jointpos[5] = -8.524*PI/180;

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
                                               3        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               2000,    /*保留默认为1000 */
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

    send_robotService->robotServiceRegisterRealTimeRoadPointCallback(VisualServoing::realTimeWaypointCallback, NULL);
    usleep(1000);//稍微等一下

    // check macsize
    m_checkMacSize = new checkMacData(0);
    // ///////pthread_create(&check_mac_thread_, NULL, &checkMacData::checkRobotMacSize, NULL);
    check_mac_thread_ = std::thread(&checkMacData::checkRobotMacSize, m_checkMacSize);
    check_mac_thread_.detach();
    
    int tmp_macsize = m_checkMacSize->getData();
    std::cout << "macTargetPosDataSize child thread(in main): " << tmp_macsize<<std::endl;


    send_robotService->robotServiceEnterTcp2CanbusMode();
    usleep(1000*1000);
    send_robotService->robotServiceLeaveTcp2CanbusMode();


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

    tmp_wayPoint = G_waypoint;

    //enter Tcp2Can Mode
    send_robotService->robotServiceEnterTcp2CanbusMode();
    usleep(1000*1000);//稍微等一下

    // 到这都没有问题

    // move_right_left_thread_ = std::thread(&RobotCatch::MoveRightLeft, this);
    // check_mac_thread_.detach();
}

  VisualServoing::~VisualServoing()
{
    std::cout<<"average time delay"<<time_delay_total/num<<std::endl;



    plt::figure_size(1280, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(vec_time_delay,".");
    // plt::xlim(0,1280);
    plt::ylim(0,5000);
    plt::title("time delay");
    plt::xlabel("x");
    plt::ylabel("y");  
    // plt::show();
    // save figure
    //const char* filename = "./basic" + to_string(mcelex_index) + ".png";
    std::string filename = "./time_delay.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);

    // LEAVE Tcp2Can Mode
    usleep(1000*1000);
    send_robotService->robotServiceLeaveTcp2CanbusMode();
    usleep(2000*1000);

    /** 接口调用: 初始化运动属性 ***/
    send_robotService->robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 130.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 130.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 130.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 160.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 160.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 160.0/180.0*M_PI;   //接口要求单位是弧度
    send_robotService->robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 900.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 900.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 900.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 900.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 900.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 900.0/180.0*M_PI;   //接口要求单位是弧度
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
void VisualServoing::realTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    aubo_robot_namespace::wayPoint_S waypoint = *wayPointPtr;

    G_waypoint = waypoint;
}

void VisualServoing::imagePointCallback(const geometry_msgs::Point &msg)
{
  std::cout<<"callback once!"<<std::endl;

  move_change = 1;

  num++;

  // if (!isMoveDone()) // 初始运动是否完成，没完成就等待，直到完成，如果是小球进入视野就不用这步，因为不需要机械臂初始运动
  // {
  //   return;
  // }

  // 第一次动时，根据当前路点开始

  // 之后动的时候，根据第一次动的路点进行累加


  // 如果是5ms为时间间隔，那应该上一次发的点差不多走完了，那就可以直接用当前路点吧


  // 错了，还是得累积

  // aubo_robot_namespace::wayPoint_S tmp_wayPoint; // 累积

  // 机械臂雅可比矩阵
  // aubo_robot_namespace::wayPoint_S waypoint;
  // waypoint = G_waypoint;
  double theta[6]; 
  theta[0] = G_waypoint.jointpos[0];
  theta[1] = G_waypoint.jointpos[1];
  theta[2] = G_waypoint.jointpos[2];
  theta[3] = G_waypoint.jointpos[3];
  theta[4] = G_waypoint.jointpos[4];
  theta[5] = G_waypoint.jointpos[5];
  // theta[0] = tmp_wayPoint.jointpos[0];
  // theta[1] = tmp_wayPoint.jointpos[1];
  // theta[2] = tmp_wayPoint.jointpos[2];
  // theta[3] = tmp_wayPoint.jointpos[3];
  // theta[4] = tmp_wayPoint.jointpos[4];
  // theta[5] = tmp_wayPoint.jointpos[5];

  Eigen::Matrix<double, 6, 6> manipulator_jocab;
  manipulator_jocab = getJacobian(theta); // rad

  manipulator_jocab = J_tool * manipulator_jocab;// q_dot to camera_vw_dot

  // 图像雅可比矩阵
  double feature_now[2];
  feature_now[0] = msg.x;
  feature_now[1] = 800 - msg.y - 1;
  
  

  Eigen::Matrix<double, 2, 2> image_jocab;
  image_jocab = getInteraction(feature_now);
  // std::cout<<"image_jocab:"<<image_jocab<<std::endl;
  // std::cout<<"image_jocab_inv:"<<image_jocab.inverse()<<std::endl;

  // 相机速度
  Eigen::Matrix<double, 2, 1> feature_error;


  // // 直接以最大速度平移
  // double z = 0.45;
  // double fx = 772.886;
  // double fy = 774.153;
  // feature_error << feature_now[0]-feature_star[0], feature_now[1]-feature_star[1];

  // aubo_robot_namespace::MoveRelative rel;
  // // rel.relativePosition[0] = -0.3*feature_error(0,0)*z/fx; // x
  // // rel.relativePosition[1] = 0.1*feature_error(1,0)*z/fy; // y
  // rel.relativePosition[0] = -1*feature_error(0,0)*z/fx; // x
  // rel.relativePosition[1] = 0.5*feature_error(1,0)*z/fy; // y
  // rel.relativePosition[2] = 0;

  // send_robotService->robotMoveLineToTargetPositionByRelative(m_userCoord,rel,true);


 // /*
  Eigen::Matrix<double, 2, 1> camera_velocity_2d;
  feature_error << feature_now[0]-feature_star[0], feature_now[1]-feature_star[1];

  double dt = 0.008;// 步长

  // analysis

  double time_delay;

  time_delay = 1000*dt*(abs(feature_error(0,0)/(feature_save[0]-feature_now[0])) + abs(feature_error(1,0)/(feature_save[1]-feature_now[1])))/2;
  if (isnan(time_delay) || time_delay > 5000)
  {

  }
  else
  {
    std::cout<<"time_delay:"<<time_delay<<"(ms)"<<std::endl;
    time_delay_total += time_delay;

    feature_save[0] = feature_now[0];
    feature_save[1] = feature_now[1];

    outfile << time_delay << std::endl;
    vec_time_delay.emplace_back(time_delay);
  }

  //
  
  // if (num > 3)
  // {
  //   feature_error = (vec_feature_error_save[vec_feature_error_save.size()-3]+
  //                    vec_feature_error_save[vec_feature_error_save.size()-2]+
  //                    vec_feature_error_save[vec_feature_error_save.size()-1])/3;
  // }
   vec_feature_error_save.push_back(feature_error);

  // lamda从小到大增加，不然初始时刻不能到达
  if (num <10)
    lamda = double(num)/10;
  else
    lamda = 1;
 
  camera_velocity_2d = (image_jocab.inverse() * feature_error).array()*(-lamda);
  
  // Eigen::Matrix<double, 2, 1> fv;
  // fv = ((image_jocab.inverse() * feature_error - image_jocab.inverse() * feature_error_save)).array()/dt;
  // std::cout<<fv<<std::endl;
  
  // Eigen::Matrix<double, 2, 1> emmm;
  // emmm = (fv - feature_velocity_star).array()*0.01;
  // std::cout<<"emmm"<<emmm<<std::endl;

  // camera_velocity_2d += emmm;
  // feature_error_save = feature_error;



    
  // camera_velocity_2d = camera_velocity_2d.array()*(-lamda) + ;
  // std::cout<<"camera_velocity_2d:"<<camera_velocity_2d<<std::endl;
  // std::cout<<"feature_error:"<<feature_error<<std::endl;


  if (!chechCameraVelocity(camera_velocity_2d))
  {
    return;
  }

  Eigen::Matrix<double, 6, 1> camera_velocity;
  camera_velocity << camera_velocity_2d(0,0), camera_velocity_2d(1,0),0,0,0,0;

  // std::cout<<"camera_velocity:"<<camera_velocity<<std::endl;

  // 关节速度
  Eigen::Matrix<double, 6, 1> joint_velocity;
  joint_velocity = manipulator_jocab.inverse() * camera_velocity;

  // double l1,l2;
  // l1 = 0.5;
  // l2 = 0.1;
  // if (num > 1)
  // {
  //   // joint_velocity(0,0)= l1*joint_velocity(0,0) + (1-l1)*joint_velocity_save(0,0);
  //   // joint_velocity(1,0)= l2*joint_velocity(1,0) + (1-l2)*joint_velocity_save(1,0);
  //   // joint_velocity(2,0)= l2*joint_velocity(2,0) + (1-l2)*joint_velocity_save(2,0);
  //   // joint_velocity(3,0)= l2*joint_velocity(3,0) + (1-l2)*joint_velocity_save(3,0);
  //   // joint_velocity(4,0)= l2*joint_velocity(4,0) + (1-l2)*joint_velocity_save(4,0);
  //   // joint_velocity(5,0)= l1*joint_velocity(5,0) + (1-l1)*joint_velocity_save(5,0);
  //   // joint_velocity= 0.9*joint_velocity + (1-0.9)*joint_velocity_save;
  // }
  // joint_velocity_save = joint_velocity;

  // if (num > 3)
  // {
  //   joint_velocity = (vec_joint_velocity_save[vec_joint_velocity_save.size()-3]+
  //                     vec_joint_velocity_save[vec_joint_velocity_save.size()-2]+
  //                     vec_joint_velocity_save[vec_joint_velocity_save.size()-1])/3;
  // }
  //  vec_joint_velocity_save.push_back(joint_velocity);



  // // std::cout<<"joint_velocity:"<<joint_velocity<<std::endl;


  if (!chechJointState(joint_velocity))// 速度和位置
  {
    return;
  }

  
  
  // tmp_wayPoint.jointpos[0] = tmp_wayPoint.jointpos[0] + dt*joint_velocity(0,0);
  // tmp_wayPoint.jointpos[1] = tmp_wayPoint.jointpos[1] + dt*joint_velocity(1,0);
  // tmp_wayPoint.jointpos[2] = tmp_wayPoint.jointpos[2] + dt*joint_velocity(2,0);
  // tmp_wayPoint.jointpos[3] = tmp_wayPoint.jointpos[3] + dt*joint_velocity(3,0);
  // tmp_wayPoint.jointpos[4] = tmp_wayPoint.jointpos[4] + dt*joint_velocity(4,0);
  // tmp_wayPoint.jointpos[5] = tmp_wayPoint.jointpos[5] + dt*joint_velocity(5,0);
  tmp_wayPoint.jointpos[0] += dt*joint_velocity(0,0);
  tmp_wayPoint.jointpos[1] += dt*joint_velocity(1,0);
  tmp_wayPoint.jointpos[2] += dt*joint_velocity(2,0);
  tmp_wayPoint.jointpos[3] += dt*joint_velocity(3,0);
  tmp_wayPoint.jointpos[4] += dt*joint_velocity(4,0);
  tmp_wayPoint.jointpos[5] += dt*joint_velocity(5,0);


  // std::cout<<"-----------------------------------------------"<<std::endl
  //     <<"第一个关节："<<tmp_wayPoint.jointpos[0]*180/PI<<std::endl
  //     <<"第二个关节："<<tmp_wayPoint.jointpos[1]*180/PI<<std::endl
  //     <<"第三个关节："<<tmp_wayPoint.jointpos[2]*180/PI<<std::endl
  //     <<"第四个关节："<<tmp_wayPoint.jointpos[3]*180/PI<<std::endl
  //     <<"第五个关节："<<tmp_wayPoint.jointpos[4]*180/PI<<std::endl
  //     <<"第六个关节："<<tmp_wayPoint.jointpos[5]*180/PI<<std::endl;
  // std::cout<<"-----------------------------------------------"<<std::endl;
  joint_outfile<<tmp_wayPoint.jointpos[0]*180/PI<<","
               <<tmp_wayPoint.jointpos[1]*180/PI<<","
               <<tmp_wayPoint.jointpos[2]*180/PI<<","
               <<tmp_wayPoint.jointpos[3]*180/PI<<","
               <<tmp_wayPoint.jointpos[4]*180/PI<<","
               <<tmp_wayPoint.jointpos[5]*180/PI<<","<<std::endl;

  MoveUsingTcp2Can(tmp_wayPoint);
  // usleep(1500);
  //*/
  


}


int VisualServoing::isMoveDone()
{
  // return flag;
  return flag;
}


int VisualServoing::chechCameraVelocity(Eigen::Matrix<double, 2, 1> &v)
{
  if (v(0,0) < 2.5 && v(1,0) < 2.5)
    return 1;
  else
  {
    std::cout<<"over camera velocity range"<<std::endl;
    return 0;

  }


}

int VisualServoing::chechJointState(Eigen::Matrix<double, 6, 1> &v)
{
  if (v(0,0) < 150*PI/180 && 
      v(1,0) < 150*PI/180 && 
      v(2,0) < 150*PI/180 && 
      v(3,0) < PI && 
      v(4,0) < PI &&
      v(5,0) < PI )
    return 1;
  else
  {
    std::cout<<"over joint velocity range"<<std::endl;
    return 0;
  }
}

Eigen::Matrix<double, 6, 6> VisualServoing::getJacobian(double *th)
{
  Eigen::Matrix<double, 6, 6> jacob;
  //////////////////// 1 //////////////////////
  a = aubo.link1.DH.a; alpha = aubo.link1.DH.alpha; d = aubo.link1.DH.d; theta = 0;
  H1 << cos(theta)  , -sin(theta)*cos(alpha) ,  sin(theta)*sin(alpha), a*cos(theta),
        sin(theta)  ,  cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta),
        0           ,  sin(alpha)            ,  cos(alpha)           , d           ,
        0           ,  0                     ,  0                    , 1           ;
  // std::cout<<H1<<std::endl;
  //////////////////// 2 //////////////////////
  a = aubo.link2.DH.a; alpha = aubo.link2.DH.alpha; d = aubo.link2.DH.d; theta = th[0];
  H2 << cos(theta)  , -sin(theta)*cos(alpha) ,  sin(theta)*sin(alpha), a*cos(theta),
        sin(theta)  ,  cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta),
        0           ,  sin(alpha)            ,  cos(alpha)           , d           ,
        0           ,  0                     ,  0                    , 1           ;
  // std::cout<<H2<<std::endl;
  //////////////////// 3 //////////////////////
  a = aubo.link3.DH.a; alpha = aubo.link3.DH.alpha; d = aubo.link3.DH.d; theta = th[1] + 90*PI/180;
  H3 << cos(theta)  , -sin(theta)*cos(alpha) ,  sin(theta)*sin(alpha), a*cos(theta),
        sin(theta)  ,  cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta),
        0           ,  sin(alpha)            ,  cos(alpha)           , d           ,
        0           ,  0                     ,  0                    , 1           ;
  // std::cout<<H3<<std::endl;
  //////////////////// 4 //////////////////////
  a = aubo.link4.DH.a; alpha = aubo.link4.DH.alpha; d = aubo.link4.DH.d; theta = th[2];
  H4 << cos(theta)  , -sin(theta)*cos(alpha) ,  sin(theta)*sin(alpha), a*cos(theta),
        sin(theta)  ,  cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta),
        0           ,  sin(alpha)            ,  cos(alpha)           , d           ,
        0           ,  0                     ,  0                    , 1           ;
  // std::cout<<H4<<std::endl;
  //////////////////// 5 //////////////////////
  a = aubo.link5.DH.a; alpha = aubo.link5.DH.alpha; d = aubo.link5.DH.d; theta = th[3] - 90*PI/180;
  H5 << cos(theta)  , -sin(theta)*cos(alpha) ,  sin(theta)*sin(alpha), a*cos(theta),
        sin(theta)  ,  cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta),
        0           ,  sin(alpha)            ,  cos(alpha)           , d           ,
        0           ,  0                     ,  0                    , 1           ;
  // std::cout<<H5<<std::endl;
  //////////////////// 6 //////////////////////
  a = aubo.link6.DH.a; alpha = aubo.link6.DH.alpha; d = aubo.link6.DH.d; theta = th[4];
  H6 << cos(theta)  , -sin(theta)*cos(alpha) ,  sin(theta)*sin(alpha), a*cos(theta),
        sin(theta)  ,  cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta),
        0           ,  sin(alpha)            ,  cos(alpha)           , d           ,
        0           ,  0                     ,  0                    , 1           ;
  // std::cout<<H6<<std::endl;
  //////////////////// 7 //////////////////////
  a = aubo.link7.DH.a; alpha = aubo.link7.DH.alpha; d = aubo.link7.DH.d; theta = th[5];
  H7 << cos(theta)  , -sin(theta)*cos(alpha) ,  sin(theta)*sin(alpha), a*cos(theta),
        sin(theta)  ,  cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta),
        0           ,  sin(alpha)            ,  cos(alpha)           , d           ,
        0           ,  0                     ,  0                    , 1           ;
  // std::cout<<H7<<std::endl;
  ///////////////////////////////////////////////////////////
  H01  =H1;
  H02  =H1*H2;
  H03  =H1*H2*H3;
  H04  =H1*H2*H3*H4;
  H05  =H1*H2*H3*H4*H5;
  H06  =H1*H2*H3*H4*H5*H6;
  H_end=H1*H2*H3*H4*H5*H6*H7;
  // std::cout<<H_end<<std::endl;

  p1 = H01.block<3,1>(0,3);
  p2 = H02.block<3,1>(0,3);
  p3 = H03.block<3,1>(0,3);
  p4 = H04.block<3,1>(0,3);
  p5 = H05.block<3,1>(0,3);
  p6 = H06.block<3,1>(0,3);
  p7 = H_end.block<3,1>(0,3);
  // std::cout<<p1<<std::endl;
  // std::cout<<p7<<std::endl;

  z1 = H01.block<3,1>(0,2);
  z2 = H02.block<3,1>(0,2);
  z3 = H03.block<3,1>(0,2);
  z4 = H04.block<3,1>(0,2);
  z5 = H05.block<3,1>(0,2);
  z6 = H06.block<3,1>(0,2);
  // z7 = H_end.block<3,1>(0,2);

  Jv1=z1.cross(p7-p1);
  Jv2=z2.cross(p7-p2);
  Jv3=z3.cross(p7-p3);
  Jv4=z4.cross(p7-p4);
  Jv5=z5.cross(p7-p5);
  Jv6=z6.cross(p7-p6);
  Jw1=z1;
  Jw2=z2;
  Jw3=z3;
  Jw4=z4;
  Jw5=z5;
  Jw6=z6;

  Jv << Jv1,Jv2,Jv3,Jv4,Jv5,Jv6;
  Jw << Jw1,Jw2,Jw3,Jw4,Jw5,Jw6;
  Jv = Jv.array() / 1000;

  jacob<<H_end.block<3,3>(0,0).transpose()*Jv,
         H_end.block<3,3>(0,0).transpose()*Jw;

  // jacob = J_tool * jacob;

  return jacob;
}

Eigen::Matrix<double, 2, 2> VisualServoing::getInteraction(double *feature)
{

  Eigen::Matrix<double, 2, 2> jacob;
  /////////////////////////////////////////////
  double Z = 0.4; // 固定
  double fx = 772.886;
  double fy = 774.153;
  jacob << -fx/Z, 0,
            0,  -fy/Z;
  return jacob;
}


// 利用透传模式实现左右两点来回小幅度运动，然后要写在一个线程函数里，随时给他上锁
void VisualServoing::MoveRightLeft()
{
  Eigen::VectorXd time;
  Eigen::VectorXd angle_sin;
  int size = 1000;
  int duration = 1;
  int deltat = duration/size;
  double anglerange = 0.5;
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
    m_waypoint.jointpos[1] = m_waypoint_init.jointpos[1] + angle_sin(i)/4;
    m_waypoint.jointpos[2] = m_waypoint_init.jointpos[2] + angle_sin(i)/4;
    m_waypoint.jointpos[3] = m_waypoint_init.jointpos[3] + angle_sin(i)/4;
    m_waypoint.jointpos[4] = m_waypoint_init.jointpos[4] + angle_sin(i)/4;
    m_waypoint.jointpos[5] = m_waypoint_init.jointpos[5] + angle_sin(i);

    if (move_change)
    {
      // 等待标志位改变，记录发送的最后一个点
      // m_save_flange_waypoint = m_waypoint;
      tmp_wayPoint = m_waypoint;
      MoveUsingTcp2Can(m_waypoint);
      std::cout << "move right and left thread child thread close: " << std::endl;
      // move_change = 1;
      //唤醒线程
      flag = 1;

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
      // if (turn == 15)
      // {
      //   std::cout << "move right and left thread child thread close: " << std::endl;
      //   move_change = 1;
      //   break;
      // }
      
    }
  }

}
// test
void VisualServoing::MoveDown()
{
  aubo_robot_namespace::MoveRelative rel;
  rel.relativePosition[2] = -0.006;
  send_robotService->robotMoveLineToTargetPositionByRelative(m_userCoord,rel,true);
}

// 当收到要运动的目标点位姿后，改变全局标志位，左右运动被停止，并唤醒移动到目标点位置的程序
void VisualServoing::MoveToTarget()
{
  // while (!move_change)
  // {
  //   // 判断目标点是否非空，空则不改变标志位，非空则改变。标志位是要上锁的
  //   // std::cout<<"move_change: "<<move_change<<std::endl;
  //   usleep(100);

  // }
  // std::cout<<"move_change: "<<move_change<<std::endl;

  m_target_flange_position.x = -0.1651;
  m_target_flange_position.y = 0.2611;
  m_target_flange_position.z = 0.4079;
  m_target_flange_ori.w = 1;
  m_target_flange_ori.x = 0;
  m_target_flange_ori.y = 0;
  m_target_flange_ori.z = 0;

  // if (1 !xxx.size())
  // {

  // }
  
  // // 线程等待/阻塞在这里，不然有可能左右运动还在动，条件变量

  // 快速运动到目标位置
  struct timeval t0, t1;
  double deltaT;
  gettimeofday(&t0, NULL);
  // send_robotService->userToBaseCoordinate(m_vec_target_tool_waypoint[0].cartPos.position,    //基于用户座标系的工具末端位置信息
  //                                         m_vec_target_tool_waypoint[0].orientation, //基于用户座标系的工具末端姿态信息
  //                                         m_userCoord,  //用户坐标系
  //                                         m_toolDesc,                 //工具参数                          
  //                                         m_target_flange_position,    //基于基座标系的法兰盘中心位置信息，共有
  //                                         m_target_flange_ori  //基于基座标系的法兰盘中心姿态信息，共有
  //                                         );
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
void VisualServoing::MoveUsingTcp2Can(aubo_robot_namespace::wayPoint_S &waypoint)
{
  //m_waypoint.jointpos[0] = m_waypoint.jointpos[0] + 0.00008*i;   // increase 0.11° at a time(you must adjust that use your trajectory planning data ,that only a demo)
  // note: The Joint Angle you send, the corresponding velocity and acc can`t more than rate speed and acc ,otherwise the robot shutdown and poweroff
  // setup3:check Mac Size , stop setRobotPosData2Canbus if more than 200
  
  //i think the reason why he uses 200 is 200*5=1s, so we can change the value of the tmp_macsize
   //send your translate data by this API(set it less than 5ms at a time)
  int tmp_macsize = m_checkMacSize->getData();
  std::cout << "macTargetPosDataSize move thread: " << tmp_macsize  << std::endl;
  if (tmp_macsize < 1000)
  {
    send_robotService->robotServiceSetRobotPosData2Canbus(waypoint.jointpos);

    // // 太少了会卡顿，下下策
    // if (tmp_macsize <= 24)
    // {
    //   for (int i = 0; i<2; i++)
    //   {
    //     if (rand()%2 == 0)
    //       waypoint.jointpos[0] += 0.00001;
    //     else 
    //       waypoint.jointpos[0] -= 0.00001;
    //     send_robotService->robotServiceSetRobotPosData2Canbus(waypoint.jointpos);
    //   }

    // }
    // else if(tmp_macsize <= 24)
    // {
    //   for (int i = 0; i<2; i++)
    //   {
    //     if (rand()%2 == 0)
    //       waypoint.jointpos[0] += 0.00001;
    //     else 
    //       waypoint.jointpos[0] -= 0.00001;
    //     send_robotService->robotServiceSetRobotPosData2Canbus(waypoint.jointpos);
    //   }
      
    // }
    // else if(tmp_macsize <= 48)
    // {
    //   if (rand()%2 == 0)
    //     waypoint.jointpos[0] += 0.00001;
    //   else 
    //     waypoint.jointpos[0] -= 0.00001;
    //   send_robotService->robotServiceSetRobotPosData2Canbus(waypoint.jointpos);
    // }
  }
}

// 
void VisualServoing::QuinticPolyPlan(double jointpos_0,
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



int main(int argc, char **argv) {
  ros::init(argc, argv, "celex5_stereo_multithread_robot_catch");
  ros::NodeHandle node_;


  VisualServoing vs(node_);
  vs.catch_sub_ = node_.subscribe(
        "/celex5_monocular/point", 1, &VisualServoing::imagePointCallback, &vs);
  // std::cout<<"我的回调函数去哪了？？？"<<std::endl;
  struct timeval t0, t1;
  double deltaT;
  pthread_t move_right_left_thread_;
  pthread_t move_to_target_thread_;
  int ret;
  void *RobotMoveRightLeft(void *arg);
  void *RobotMoveToTarget(void *arg);

  // std::cout << "main() : 创建机械臂往复运动线程" << std::endl;      
  // ret = pthread_create(&move_right_left_thread_, NULL, RobotMoveRightLeft, (void *)&vs);
  // if (ret){
  //     std::cout << "Error:无法创建机械臂往复运动线程" << std::endl;
  //     exit(-1);
  // }
  // pthread_detach(move_right_left_thread_);

  // std::cout << "main() : 创建机械臂等待抓捕线程" << std::endl;      
  // ret = pthread_create(&move_to_target_thread_, NULL, RobotMoveToTarget, (void *)&vs);
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
    if (deltaT > 80)//0s
    {
      // delete vs;
      // vs.MoveDown();
      sleep(1);
      ROS_INFO("Program break down: time too long!");
      
      ros::shutdown();
      return 0;
    }
  }

  // delete vs;

  return EXIT_SUCCESS;
}


void *RobotMoveRightLeft(void *arg){
    VisualServoing *vs;
    vs = (VisualServoing *) arg;
    vs->MoveRightLeft();
    pthread_exit(NULL);
}

void *RobotMoveToTarget(void *arg){
    VisualServoing *vs;
    vs = (VisualServoing *) arg;
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

    vs->MoveToTarget();
    pthread_exit(NULL);
}