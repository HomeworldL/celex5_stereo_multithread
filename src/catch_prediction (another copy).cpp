#include "catch_prediction.h"

////////////// Class ////////////////
class CatchPrediction{
public:


  CatchPrediction(ros::NodeHandle &nh);
  ~CatchPrediction();

  void CalculateCallback(const geometry_msgs::PoseStamped &msg);
  void KarmanFilting();
  void CatchPointPub(geometry_msgs::PoseStamped &pose);
  ////////////////////////////////////////////////////////////////


private:
  // private ROS node handle
  ros::NodeHandle node_;
  ros::Subscriber pose_sub_;
  ros::Publisher catch_pub_;
    
  std::vector<geometry_msgs::PoseStamped> m_vec_ball_pose;
  // geometry_msgs::PoseStamped m_init_ball_pose;// 绝对时间，而不是相对
  // geometry_msgs::TwistStamped m_init_ball_twist;
  // geometry_msgs::PoseStamped m_capture_ball_pose;// 绝对时间，而不是相对
  // geometry_msgs::TwistStamped m_capture_ball_twist;
  //////////////////////////////////////////////////////////////////////////////////
  int count_num_;                         //接收数据计数
  float x_start_;
  double gravityacceleration_;            //重力加速度

  ////////////////////////////////////////////////////////////////////////////////
  // 异步累积
  double t_init;
  double t_sum;
  double x_sum;
  double xt_sum;
  double y_sum;
  double yt_sum;
  double z_sum;
  double t2_sum;
  double zt_sum;
  double t3_sum;

  Eigen::Matrix<double, 6, 6> M;         //系数矩阵
  Eigen::Matrix<double, 6, 1> N;         
  Eigen::Matrix<double, 6, 1> theta_;     //得到的表达式参数，需要存储并输出,x,vx,y,vy,z,vz
  Eigen::Matrix<double, 6, 1> m_now_ball_theta;

  std::vector<Eigen::Matrix<double, 6, 1>> m_vec_now_ball_theta;

  ////////////////////////////////////////////////////////////////////////////////
  int CalculateNearestCatch(geometry_msgs::PoseStamped &capture_ball_pose);
  double time_to_target;
  Eigen::Matrix<double, 3, 1> m_init_robot_pos;//把m_init_robot_pos换成矩阵，计算更快
  Eigen::MatrixXd trajectory_pos;//每个时间对应的小球位置
  double t_deltat;  //时间间隔5ms，对应机械臂透传模式,0.005
  double t_period;  //从当前时刻计算1000ms以内的位置
  int t_num;
  Eigen::VectorXd t_vec; //时间数列

  double distance;//末端到轨迹离散点的距离
  double distance_min;
  int distance_min_index;

  double A;
  double B;
  double C;
  Eigen::Matrix<double, 3, 1> ball_rotm_z;
  Eigen::Matrix<double, 3, 1> ball_rotm_y;
  Eigen::Matrix<double, 3, 1> ball_rotm_x;
  Eigen::Matrix3d  ball_rotm;
  Eigen::Quaterniond ball_quaternion;
  Eigen::Vector4d q;

  ///////////////////////////////////////////////////////////////////////////////
  int CalculatePlaneCatch(geometry_msgs::PoseStamped &capture_ball_pose);

  ///////////////////////////////////////////////////////////////////////////////
  void PlotTrajectory3D(std::vector<geometry_msgs::PoseStamped> &trajectory_point);
  std::vector<double> m_plot_x;
  std::vector<double> m_plot_y;
  std::vector<double> m_plot_z;

  std::ofstream ball_point_outfile;  

  

};

CatchPrediction::CatchPrediction(ros::NodeHandle &nh)
{
  // 参数初始化
  node_ = nh;

  count_num_ = 0;
  gravityacceleration_ = 9.81;
  x_start_ = -1.8;

  // 机械臂初始工具的坐标
  m_init_robot_pos[0] = -0.2519;
  m_init_robot_pos[1] = -0.2375;
  m_init_robot_pos[2] = 0.5657;

    // 异步累积
  double t_sum = 0;
  double x_sum = 0;
  double xt_sum = 0;
  double y_sum = 0;
  double yt_sum = 0;
  double z_sum = 0;
  double t2_sum = 0;
  double zt_sum = 0;
  double t3_sum = 0;

  t_deltat = 0.005;
  t_period = 1; // 飞一秒内的轨迹
  t_num = t_period/t_deltat;
  t_vec = Eigen::VectorXd::LinSpaced(t_num + 1, 0, t_period);//要换算为s
  trajectory_pos = Eigen::MatrixXd::Zero(3,t_num + 1);

  ball_point_outfile.open("ball_point_3d.txt", std::ios::out);  
  ball_point_outfile.setf(std::ios::fixed, std::ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
  ball_point_outfile.precision(10);  // 设置精度


  int event_camera_open = 1 ;
  if (event_camera_open)
  {
    pose_sub_ = node_.subscribe("/celex5_stereo/ball_pose", 0, &CatchPrediction::CalculateCallback, this);    //订阅小球位置信息，收到就执行回调函数
  }


  // pub the pose
  catch_pub_ = node_.advertise<geometry_msgs::PoseStamped>("celex5_stereo/catch_pose", 10);
  // sleep(20);
  // while (1)
  // {
  //   CatchPointPub();
  //   std::cout<<"pub running!"<<std::endl;
  //   sleep(1);

  // }
  
}

CatchPrediction::~CatchPrediction()
{
  PlotTrajectory3D(m_vec_ball_pose);

  ball_point_outfile.close();
}


/***************************************
核心函数，通过接收到的小球位置数量进行分类计算
前两次收到执行最小二乘
从第三次开始进行迭代，更新catch_point_并发布
****************************************/
void CatchPrediction::CalculateCallback(const geometry_msgs::PoseStamped &msg) // 新函数，拟合
{
    // ROS_INFO("666");
    // std::cout<<"pose"<<msg<<std::endl;
    if (1 /*msg.pose.position.x > x_start_*/)//如果x坐标大于则开始预测
    {
      count_num_++;

      //////////////////////////////////////
      if (count_num_ < 81) // 前10误差太大
      {
        return;
      }
      m_vec_ball_pose.push_back(msg);

      if (count_num_ == 81) // 初始时刻，不然矩阵病态了
      {
        
        t_init = msg.header.stamp.sec + double(msg.header.stamp.nsec)/1e9;
        for (int kk=0; kk<20;kk++)
        {std::cout<<"!"<<std::endl;}
      }

      if (count_num_ < 100)
      {
        // 异步累积
        double t = msg.header.stamp.sec + double(msg.header.stamp.nsec)/1e9 - t_init;
        // std::cout<<"t:"<<t<<std::endl;
        
        t_sum += t;
        t2_sum+= pow(t,2);
        t3_sum+= pow(t,3);
        x_sum += msg.pose.position.x;
        xt_sum+= msg.pose.position.x * t;
        y_sum += msg.pose.position.y;
        yt_sum+= msg.pose.position.y * t;
        z_sum += msg.pose.position.z;
        zt_sum+= msg.pose.position.z * t;
        // ROS_INFO("received once");
      }
      else
      {
        // ROS_INFO("received once");
        // std::cout<<"count_num_:"<<count_num_<<std::endl;
        // 异步累积
        double t = msg.header.stamp.sec + double(msg.header.stamp.nsec)/1e9 - t_init;
        // std::cout<<"t:"<<t<<std::endl;
        // std::cout<<"t_init:"<<t_init<<std::endl;
        // t_sum += t;
        t2_sum+= pow(t,2);
        t3_sum+= pow(t,3);
        x_sum += msg.pose.position.x;
        xt_sum+= msg.pose.position.x * t;
        y_sum += msg.pose.position.y;
        yt_sum+= msg.pose.position.y * t;
        z_sum += msg.pose.position.z;
        zt_sum+= msg.pose.position.z * t;
        if (count_num_%50 == 0)
        {
          std::cout<<"count_num_:"<<count_num_<<std::endl;
          int real_num = count_num_ - 80;
          // 拟合
          M << real_num,t_sum,0,0,0,0,
               t_sum,t2_sum,0,0,0,0,
               0,0,real_num,t_sum,0,0,
               0,0,t_sum,t2_sum,0,0,
               0,0,0,0,real_num,t_sum,
               0,0,0,0,t_sum,t2_sum;
          N << x_sum,xt_sum,y_sum,yt_sum,z_sum+0.5*gravityacceleration_*t2_sum,zt_sum+0.5*gravityacceleration_*t3_sum;
          theta_ = M.inverse()*N;

          // std::cout<<"/////////////////// M ///////////////////"<<M<<std::endl;
          // std::cout<<"/////////////////// N ///////////////////"<<N<<std::endl;

          ///////////////////////////
          // double time_to_now = ((msg.pose.position.x - theta_(1,0))/theta_(0,0) + (msg.pose.position.y - theta_(1,1))/theta_(0,1))/2;//x,y都是匀速
          // 算当前位置
          m_now_ball_theta(0,0) = theta_(0,0) + theta_(1,0) * t;
          m_now_ball_theta(1,0) = theta_(1,0);
          m_now_ball_theta(2,0) = theta_(2,0) + theta_(3,0) * t;
          m_now_ball_theta(3,0) = theta_(3,0);
          m_now_ball_theta(4,0) = theta_(4,0) + theta_(5,0) * t - gravityacceleration_ * t * t/2;
          m_now_ball_theta(5,0) = theta_(5,0) - gravityacceleration_ * t;

          m_vec_now_ball_theta.push_back(m_now_ball_theta);

          geometry_msgs::PoseStamped pose_msg;
          int ret = CalculateNearestCatch(pose_msg);
          
          if (ret == 1)
          {
            CatchPointPub(pose_msg);
            std::cout<<"///////////////////theta_///////////////////"<<theta_<<std::endl;
            std::cout<<"///////////////////m_now_ball_theta///////////////////"<<m_now_ball_theta<<std::endl;
            std::cout<<"///////////////////catch pose///////////////////"<<pose_msg<<std::endl;
          }
        }
      }
    }
}


void CatchPrediction::CatchPointPub(geometry_msgs::PoseStamped &msg)
{
  // geometry_msgs::PoseStamped msg;
  // msg.pose.position.x = -0.5159;
  // msg.pose.position.y = 0.0861;
  // msg.pose.position.z = 0.6079;
  // msg.pose.orientation.w = 0.1889;
  // msg.pose.orientation.x = -0.8043;
  // msg.pose.orientation.y = 0.0784;
  // msg.pose.orientation.z = 0.5579;
  
  catch_pub_.publish(msg);
  std::cout<<"pub running!"<<std::endl;
  // catch_pub_.publish(m_capture_ball_pose);
}


int CatchPrediction::CalculatePlaneCatch(geometry_msgs::PoseStamped &capture_ball_pose)
{
  double x_catch = -0.2;
  if (m_now_ball_theta(0,0) >= x_catch)
    return 0;

  double t_move  = (x_catch-m_now_ball_theta(0,0))/m_now_ball_theta(1,0) ;
  double y_catch = t_move * m_now_ball_theta(3,0) + m_now_ball_theta(2,0);
  double z_catch = t_move * m_now_ball_theta(5,0) + m_now_ball_theta(4,0) - 0.5*gravityacceleration_* pow(t_move,2);

  if ( (pow(x_catch,2)+pow(y_catch,2)+pow(z_catch,2)) > pow(0.7, 2) )
    return 0;

  /////////////////////////////////// 计算最短抓捕点的位置和姿态 /////////////////////////////////////
  capture_ball_pose.pose.position.x = x_catch;
  capture_ball_pose.pose.position.y = y_catch;
  capture_ball_pose.pose.position.z = z_catch;

  //求抓捕点姿态
  /*
  目前的想法：
  小球的本体坐标系：z为v的方向，x在轨迹平面内，即铅垂面内，这里令B>0（可以改）
  */

  //求飞行轨迹平面的法向量(A, B, C)
  Eigen::Matrix<double, 3, 1> trajectory_near_velocity;
  trajectory_near_velocity << m_now_ball_theta(1,0),
                              m_now_ball_theta(3,0),
                              m_now_ball_theta(5,0) - t_move * gravityacceleration_;
  std::cout<<"trajectory_near_velocity:"<<trajectory_near_velocity<<std::endl;
  

  B = pow(trajectory_near_velocity(0),2)/(pow(trajectory_near_velocity(0),2) + pow(trajectory_near_velocity(1),2));
  B = -sqrt(B);
  A = -B*trajectory_near_velocity(1)/trajectory_near_velocity(0);
  C = 0;//铅垂

  double trajectory_vel_norm = trajectory_near_velocity.norm();
  ball_rotm_y = trajectory_near_velocity.array() / (trajectory_vel_norm);
  ball_rotm_x << A, B, C;
  ball_rotm_z = ball_rotm_x.cross(ball_rotm_y);//默认为列向量(?)

  // std::cout<<"ball_rotm_y(v)："<<ball_rotm_y(0,0)<<","<<ball_rotm_y(1,0)<<","<<ball_rotm_y(2,0)<<std::endl
  //          <<"ball_rotm_x："<<ball_rotm_x(0,0)<<","<<ball_rotm_x(1,0)<<","<<ball_rotm_x(2,0)<<std::endl
  //          <<"ball_rotm_z："<<ball_rotm_z(0,0)<<","<<ball_rotm_z(1,0)<<","<<ball_rotm_z(2,0)<<std::endl
  // <<"-----------------------------------------------"<<std::endl;

  //求坐标转换矩阵
  ball_rotm << ball_rotm_x(0), -ball_rotm_y(0), -ball_rotm_z(0),
                ball_rotm_x(1), -ball_rotm_y(1), -ball_rotm_z(1),
                ball_rotm_x(2), -ball_rotm_y(2), -ball_rotm_z(2);


  std::cout<<"ball_rotm旋转矩阵："<<std::endl
  <<ball_rotm(0,0)<<","<<ball_rotm(0,1)<<","<<ball_rotm(0,2)<<std::endl
  <<ball_rotm(1,0)<<","<<ball_rotm(1,1)<<","<<ball_rotm(1,2)<<std::endl
  <<ball_rotm(2,0)<<","<<ball_rotm(2,1)<<","<<ball_rotm(2,2)<<std::endl
  <<"-----------------------------------------------"<<std::endl;
  //求四元数
  ball_quaternion = ball_rotm;
  ball_quaternion.normalize();
  q = ball_quaternion.coeffs();

  capture_ball_pose.pose.orientation.w = q(3);//在最后一个
  capture_ball_pose.pose.orientation.x = q(0);
  capture_ball_pose.pose.orientation.y = q(1);
  capture_ball_pose.pose.orientation.z = q(2);


  return 1;

}

int CatchPrediction::CalculateNearestCatch(geometry_msgs::PoseStamped &capture_ball_pose)
{
    //抓捕点和运动时间放在共有成员
    //cout<<"进入了CalculateNearest"<<endl;

    //轨迹离散化
    trajectory_pos.row(0) = t_vec.array() * m_now_ball_theta(1,0) + m_now_ball_theta(0,0);//数组和矩阵的转换
    trajectory_pos.row(1) = t_vec.array() * m_now_ball_theta(3,0) + m_now_ball_theta(2,0);
    trajectory_pos.row(2) = t_vec.array() * m_now_ball_theta(5,0) + m_now_ball_theta(4,0) - 0.5*gravityacceleration_*(t_vec.array().square());
    // trajectory_pos = m_now_ball_theta.row(0).transpose() * t_vec.array() + m_now_ball_theta.row(1).transpose(); // 玩骚的失败了...
    // std::cout<<"轨迹离散化结束"<<std::endl;

    // std::cout<<"t_vec"<<t_vec<<std::endl;

    distance_min = 10;
    // aubo工作空间半径886.5mm（估计值）
    for (int i = 0; i < t_num + 1; i++)
    {
        if (trajectory_pos.col(i).squaredNorm()> pow(0.7, 2) || trajectory_pos(2,i) < 0.2)//应该用矩阵加速
        {
            continue;
        }
        else//在工作空间内，开始排序
        {
            distance = (m_init_robot_pos - trajectory_pos.col(i)).norm();
            // std::cout<<"distance: "<<distance<<std::endl;
            if (distance < distance_min)
            {
                distance_min_index = i;
                distance_min = distance;
            }
        }
    }//

    if (distance_min_index == 0)//没有点在工作空间中
    {
      ROS_INFO("Program error: no point in workspace");
      return 0;
    }
    else
    {
      std::cout<<"distance_min: "<<distance_min<<std::endl;
      std::cout<<"distance_min_index: "<<distance_min_index<<std::endl;

      ROS_INFO("Find a nearest point in workspace");
    }

    /////////////////////////////////// 计算最短抓捕点的位置和姿态 /////////////////////////////////////
    capture_ball_pose.pose.position.x = trajectory_pos(0,distance_min_index);
    capture_ball_pose.pose.position.y = trajectory_pos(1,distance_min_index);
    capture_ball_pose.pose.position.z = trajectory_pos(2,distance_min_index);

    //求抓捕点姿态
    /*
    目前的想法：
    小球的本体坐标系：z为v的方向，x在轨迹平面内，即铅垂面内，这里令B>0（可以改）
    */

    //求飞行轨迹平面的法向量(A, B, C)
    Eigen::Matrix<double, 3, 1> trajectory_near_velocity;
    trajectory_near_velocity << m_now_ball_theta(1,0),
                                m_now_ball_theta(3,0),
                                m_now_ball_theta(5,0) - t_vec(distance_min_index) * gravityacceleration_;
    std::cout<<"trajectory_near_velocity:"<<trajectory_near_velocity<<std::endl;
    

    B = pow(trajectory_near_velocity(0),2)/(pow(trajectory_near_velocity(0),2) + pow(trajectory_near_velocity(1),2));
    B = -sqrt(B);
    A = -B*trajectory_near_velocity(1)/trajectory_near_velocity(0);
    C = 0;//铅垂

    double trajectory_vel_norm = trajectory_near_velocity.norm();
    ball_rotm_y = trajectory_near_velocity.array() / (trajectory_vel_norm);
    ball_rotm_x << A, B, C;
    ball_rotm_z = ball_rotm_x.cross(ball_rotm_y);//默认为列向量(?)

    // std::cout<<"ball_rotm_y(v)："<<ball_rotm_y(0,0)<<","<<ball_rotm_y(1,0)<<","<<ball_rotm_y(2,0)<<std::endl
    //          <<"ball_rotm_x："<<ball_rotm_x(0,0)<<","<<ball_rotm_x(1,0)<<","<<ball_rotm_x(2,0)<<std::endl
    //          <<"ball_rotm_z："<<ball_rotm_z(0,0)<<","<<ball_rotm_z(1,0)<<","<<ball_rotm_z(2,0)<<std::endl
    // <<"-----------------------------------------------"<<std::endl;

    //求坐标转换矩阵
    ball_rotm << ball_rotm_x(0), -ball_rotm_y(0), -ball_rotm_z(0),
                 ball_rotm_x(1), -ball_rotm_y(1), -ball_rotm_z(1),
                 ball_rotm_x(2), -ball_rotm_y(2), -ball_rotm_z(2);


    std::cout<<"ball_rotm旋转矩阵："<<std::endl
    <<ball_rotm(0,0)<<","<<ball_rotm(0,1)<<","<<ball_rotm(0,2)<<std::endl
    <<ball_rotm(1,0)<<","<<ball_rotm(1,1)<<","<<ball_rotm(1,2)<<std::endl
    <<ball_rotm(2,0)<<","<<ball_rotm(2,1)<<","<<ball_rotm(2,2)<<std::endl
    <<"-----------------------------------------------"<<std::endl;
    //求四元数
    ball_quaternion = ball_rotm;
    ball_quaternion.normalize();
    q = ball_quaternion.coeffs();

    capture_ball_pose.pose.orientation.w = q(3);//在最后一个
    capture_ball_pose.pose.orientation.x = q(0);
    capture_ball_pose.pose.orientation.y = q(1);
    capture_ball_pose.pose.orientation.z = q(2);
    
    return 1;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void CatchPrediction::PlotTrajectory3D(std::vector<geometry_msgs::PoseStamped> &trajectory_point)
{
  for (int i = 0; i<trajectory_point.size()-200; i++)
  {
    m_plot_x.push_back(trajectory_point[i].pose.position.x);
    m_plot_y.push_back(trajectory_point[i].pose.position.y);
    m_plot_z.push_back(trajectory_point[i].pose.position.z);
    ball_point_outfile << trajectory_point[i].pose.position.x <<","
                       << trajectory_point[i].pose.position.y <<","
                       << trajectory_point[i].pose.position.z <<","
                       << trajectory_point[i].header.stamp.sec + double(trajectory_point[i].header.stamp.nsec)/1e9 << std::endl;  //在txt中写入结果
  }
  
  

  plt::figure_size(800, 800);
  // Plot line from given x and y data. Color is selected automatically.
  // plt::plot3(m_plot_x, m_plot_y, m_plot_z);
  
  std::vector<double> robot_x;
  std::vector<double> robot_y;
  std::vector<double> robot_z;
  std::vector<double> zero;
  robot_x.push_back(-0.2519);
  robot_y.push_back(-0.2375);
  robot_z.push_back(0.5657);
  zero.push_back(0);
  plt::plot(m_plot_x, m_plot_y);
  plt::plot(robot_x, robot_y,"r.");
  plt::plot(zero, zero,"k.");
  for (int i = 0; i<m_vec_now_ball_theta.size();i++)
  {
    std::cout<< m_vec_now_ball_theta[i] << std::endl<<"/////////////"<<std::endl;

    std::vector<double> fit_x;
    std::vector<double> fit_y;
    std::vector<double> fit_z;
    int pren = 20;

    for (int j=0; j<pren;j++)
    {
      fit_x.push_back( (j/pren)* m_vec_now_ball_theta[i](1,0) + m_vec_now_ball_theta[i](0,0) );
      fit_y.push_back( (j/pren)* m_vec_now_ball_theta[i](3,0) + m_vec_now_ball_theta[i](2,0) );

      // std::cout<< fit_x[i] << std::endl;
      // std::cout<< fit_y[i] << std::endl;
    }
    //  plt::plot(fit_x, fit_y,"g.");

  }
  plt::title("Plot-Trajectory-xy");
  plt::xlabel("x");
  plt::ylabel("y");
  std::string filename = "./ball_pose_save_xy.png";
  // plt::show();
  std::cout << "Saving result to " << filename << std::endl;;
  plt::save(filename);




  plt::figure_size(800, 800);
  // Plot line from given x and y data. Color is selected automatically.
  plt::plot3(m_plot_x, m_plot_y, m_plot_z);
  plt::title("Plot-Trajectory-3D");
  plt::xlabel("x");
  plt::ylabel("y");

  filename = "./ball_pose_save.png";
  plt::show();
  std::cout << "Saving result to " << filename << std::endl;;
  plt::save(filename);







}//plot3d



/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
  ros::init(argc, argv, "celex5_stereo_multithread_catch_prediction");
  ros::NodeHandle node_;

  CatchPrediction *cp =
      new CatchPrediction(node_);

  struct timeval t0, t1;
  double deltaT;
  gettimeofday(&t0, NULL);//总计时器，超过一定数目，暂停
  // ros::MultiThreadedSpinner s(2);
  ros::Rate loop_rate(2e6/5000);
  while (node_.ok()) {
    ros::spinOnce();
    gettimeofday(&t1, NULL);//计时器
    deltaT = (t1.tv_sec-t0.tv_sec);//只看s
    if (deltaT > 30)//0s
    {
      // delete rc;
      sleep(1);
      ROS_INFO("Program break down: time too long!");

      delete cp;
      ros::shutdown();
      return 0;
    }
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}