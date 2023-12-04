#include "catch_prediction.h"

////////////// Class ////////////////
class CatchPrediction{
public:


  CatchPrediction(ros::NodeHandle &nh);
  ~CatchPrediction();

  void CalculateCallback(const geometry_msgs::PoseStamped &msg);
  void KarmanFilting();
  void TrajectoryPrediction();  
  void CalculatePlaneCatch();
  void CatchPointPub();
  ////////////////////////////////////////////////////////////////


private:
  // private ROS node handle
  ros::NodeHandle node_;
  ros::Subscriber pose_sub_;
  ros::Publisher catch_pub_;
    
  std::vector<geometry_msgs::PoseStamped> m_vec_ball_pose;
  // geometry_msgs::PoseStamped m_init_ball_pose;// 绝对时间，而不是相对
  // geometry_msgs::TwistStamped m_init_ball_twist;
  geometry_msgs::PoseStamped m_capture_ball_pose;// 绝对时间，而不是相对
  // geometry_msgs::TwistStamped m_capture_ball_twist;
  //////////////////////////////////////////////////////////////////////////////////
  int count_num_;                         //接收数据计数
  float x_start_;
  double gravityacceleration_;            //重力加速度
  //////////////////////////////////////////////////////////////////////////////////
  //最小二乘
  void LeastSquare();                 //两个球位置算最小二乘，使用m_vec_ball_pose
  Eigen::Matrix<double, 2, 1> time;
  Eigen::Matrix<double, 2, 3> yk;
  Eigen::Matrix<double, 2, 2> phi;
  Eigen::Matrix<double, 2, 2> P_inv;

  //递推最小二乘
  void RecursiveLeastSquare(const geometry_msgs::PoseStamped &pn);        //三个以上递推最小二乘
  double time_r;
  Eigen::Matrix<double, 1, 3> yk_r;
  Eigen::Matrix<double, 2, 1> phi_r;
  Eigen::Matrix<double, 1, 3> eps_r;
  Eigen::Matrix<double, 2, 1> K_r;

  // 递推参数
  Eigen::Matrix<double, 2, 2> P_;         //迭代中的P矩阵
  Eigen::Matrix<double, 2, 3> theta_;     //得到的表达式参数，需要存储并输出
  Eigen::Matrix<double, 2, 3> m_now_ball_theta;

  ////////////////////////////////////////////////////////////////////////////////
  int CalculateNearestCatch();
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
  void PlotTrajectory3D(std::vector<geometry_msgs::PoseStamped> &trajectory_point);
  std::vector<double> m_plot_x;
  std::vector<double> m_plot_y;
  std::vector<double> m_plot_z;

};

CatchPrediction::CatchPrediction(ros::NodeHandle &nh)
{
  // 参数初始化
  node_ = nh;

  count_num_ = 0;
  gravityacceleration_ = 9.81;
  x_start_ = -1.8;

  // 机械臂初始工具的坐标
  m_init_robot_pos[0] = -0.4651;
  m_init_robot_pos[1] = 0.1611;
  m_init_robot_pos[2] = 0.6779;

  t_deltat = 0.005;
  t_period = 1; // 飞一秒内的轨迹
  t_num = t_period/t_deltat;
  t_vec = Eigen::VectorXd::LinSpaced(t_num + 1, 0, t_period);//要换算为s
  trajectory_pos = Eigen::MatrixXd::Zero(3,t_num + 1);


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


/***************************************
核心函数，通过接收到的小球位置数量进行分类计算
前两次收到执行最小二乘
从第三次开始进行迭代，更新catch_point_并发布
****************************************/
void CatchPrediction::CalculateCallback(const geometry_msgs::PoseStamped &msg) // 新函数，拟合
{
    ROS_INFO("666");
    if (1 /*msg.pose.position.x > x_start_*/)//如果x坐标大于则开始预测
    {
      count_num_++;
      if (count_num_ < 200)
      {
        m_vec_ball_pose.push_back(msg);
        ROS_INFO("received once");
      }
      else
      {
        // 拟合
        theta = TrajectoryFit()


      }
    }
}

void CatchPrediction::TrajectoryFit()
{
    yk(0,0) = m_vec_ball_pose[0].pose.position.x;
    yk(0,1) = m_vec_ball_pose[0].pose.position.y;
    yk(0,2) = m_vec_ball_pose[0].pose.position.z;

}
// void CatchPrediction::CalculateCallback(const geometry_msgs::PoseStamped &msg)
// {
//     ROS_INFO("666");
//     if (1 /*msg.pose.position.x > x_start_*/)//如果x坐标大于则开始预测
//     {
//       count_num_++;
//       if (count_num_ == 1)
//       {
//         // ball_pose_front_ = msg;   //第一次收到点，先将其存到pose1中
//         m_vec_ball_pose.push_back(msg);
//         ROS_INFO("first ball pose received");
//       }
//       else if(count_num_ == 2)
//       {
//         m_vec_ball_pose.push_back(msg);
//         ROS_INFO("second ball pose received");//num=2执行 leastsquare
//         LeastSquare();
//       }
//       else
//       {
//         //先进行计算
//         RecursiveLeastSquare(msg);
//         ROS_INFO("ball pose called");
//         m_vec_ball_pose.push_back(msg);
//       }

//       // plot 
//       if(count_num_ == 80)
//       {
//         // matplotlib
//         // m_vec_ball_pose
//         // PlotTrajectory3D(m_vec_ball_pose);
//         CatchPointPub();
//       }

//       if(count_num_ > 80) // event camera 
//       {
//         //此时已经得到三个方向的函数了,直接调用轨迹生成和最近点预测
//         double time_to_now = ((msg.pose.position.x - theta_(1,0))/theta_(0,0) + (msg.pose.position.y - theta_(1,1))/theta_(0,1))/2;//x,y都是匀速

//         m_now_ball_theta(1,0) = theta_(1,0) + theta_(0,0) * time_to_now;
//         m_now_ball_theta(1,1) = theta_(1,1) + theta_(0,1) * time_to_now;
//         m_now_ball_theta(1,2) = theta_(1,2) + theta_(0,2) * time_to_now - gravityacceleration_ * time_to_now * time_to_now/2;
//         m_now_ball_theta(0,0) = theta_(0,0);
//         m_now_ball_theta(0,2) = theta_(0,1);
//         m_now_ball_theta(0,2) = theta_(0,2);
//         int ret = CalculateNearestCatch();
//         if (1) // ret test
//         {
//           CatchPointPub();
//           ROS_INFO("prediction done and pub once ");
//         }
//         else
//         {
//           ROS_INFO("prediction erroe once ");
//         }
                
//       }
//     }
// }

void CatchPrediction::CatchPointPub()
{
  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = -0.5159;
  msg.pose.position.y = 0.0861;
  msg.pose.position.z = 0.6079;
  msg.pose.orientation.w = 0.1889;
  msg.pose.orientation.x = -0.8043;
  msg.pose.orientation.y = 0.0784;
  msg.pose.orientation.z = 0.5579;
  
  catch_pub_.publish(msg);
  std::cout<<"pub running!"<<std::endl;
  // catch_pub_.publish(m_capture_ball_pose);
}


/**
 * @brief   输入两个坐标与对应的时间周期，并且更新类内的私有变量P_与theta_
 * @param   p1,p2两个ball_pose输入
 * @param   Tk      两点对应的 时间点！！！！注意不是时间间隔！！！！！！！！！
 * @param   yk      两个坐标
 * @return  更新P_与theta_
**/
void CatchPrediction::LeastSquare()
{
    time(0,0) = m_vec_ball_pose[0].header.stamp.sec + double(m_vec_ball_pose[0].header.stamp.nsec)/1e9;
    time(1,0) = m_vec_ball_pose[1].header.stamp.sec + double(m_vec_ball_pose[1].header.stamp.nsec)/1e9;

    yk(0,0) = m_vec_ball_pose[0].pose.position.x;
    yk(0,1) = m_vec_ball_pose[0].pose.position.y;
    yk(0,2) = m_vec_ball_pose[0].pose.position.z;
    yk(1,0) = m_vec_ball_pose[1].pose.position.x;
    yk(1,1) = m_vec_ball_pose[1].pose.position.y;
    yk(1,2) = m_vec_ball_pose[1].pose.position.z;

    yk.col(2) = yk.col(2).array() + time.array().square() * 0.5 * gravityacceleration_;
    
    //phi << 0,0,1,1;
    phi.row(0) = time.transpose();
    phi.row(1) = Eigen::MatrixXd::Ones(1,2);
    P_inv = phi * phi.transpose();
    // 通过对inv(P)求逆来得到P
    P_ = P_inv.inverse();
    // 求theta
    theta_ = P_ * phi * yk;
}
/**结合P_与theta_，输入最新的点，更新参数theta_
 * @param   Tk      球坐标点xyz
 * @return  更新私有变量theta_
**/
void CatchPrediction::RecursiveLeastSquare(const geometry_msgs::PoseStamped &pn)
{
    time_r = pn.header.stamp.sec + double(pn.header.stamp.nsec)/1e9;
    // yk_r :1*3
    yk_r(0,0) = pn.pose.position.x;
    yk_r(0,1) = pn.pose.position.y;
    yk_r(0,2) = pn.pose.position.z;
    // zk = zk + g * t^2 / 2
    yk_r(2) = yk_r(2) + 0.5 * gravityacceleration_ * time_r * time_r;
    // phi_r :2*1
    phi_r(0) = time_r; phi_r(1) = 1;
    // eps :1*3, 求eps(k) 
    eps_r = yk_r - phi_r.transpose() * theta_;
    // 输入为P(k-1), 求P(k)
    P_ = P_ - P_ * phi_r * phi_r.transpose() * P_ / (1 + (phi_r.transpose() * P_ * phi_r));
    // 求K(k)
    K_r = P_ * phi_r;
    // 输入为theta(k-1), 求theta(k)
    theta_ = theta_ + K_r * eps_r;
}
//////////////////////////////////////////////

void CatchPrediction::CalculatePlaneCatch()
{

}

int CatchPrediction::CalculateNearestCatch()
{
    //抓捕点和运动时间放在共有成员
    //cout<<"进入了CalculateNearest"<<endl;

    //轨迹离散化
    trajectory_pos.row(0) = t_vec.array() * m_now_ball_theta(0,1) + m_now_ball_theta(1,1);//数组和矩阵的转换
    trajectory_pos.row(1) = t_vec.array() * m_now_ball_theta(0,2) + m_now_ball_theta(1,2);
    trajectory_pos.row(2) = t_vec.array() * m_now_ball_theta(0,3) + m_now_ball_theta(1,3) - 0.5*gravityacceleration_*(t_vec.array().square());
    // trajectory_pos = m_now_ball_theta.row(0).transpose() * t_vec.array() + m_now_ball_theta.row(1).transpose(); // 玩骚的失败了...
    std::cout<<"轨迹离散化结束"<<std::endl;


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
            std::cout<<"distance: "<<distance<<std::endl;
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

    /////////////////////////////////// 计算最短抓捕点的位置和姿态 /////////////////////////////////////
    m_capture_ball_pose.pose.position.x = trajectory_pos(0,distance_min_index);
    m_capture_ball_pose.pose.position.y = trajectory_pos(1,distance_min_index);
    m_capture_ball_pose.pose.position.z = trajectory_pos(2,distance_min_index);

    //求抓捕点姿态
    /*
    目前的想法：
    小球的本体坐标系：z为v的方向，x在轨迹平面内，即铅垂面内，这里令B>0（可以改）
    */

    //求飞行轨迹平面的法向量(A, B, C)
    Eigen::Matrix<double, 3, 1> trajectory_near_velocity;
    trajectory_near_velocity << m_now_ball_theta(0,0),
                                m_now_ball_theta(0,1),
                                m_now_ball_theta(0,2) - distance_min_index * t_deltat * 0.5 * gravityacceleration_;

    B = pow(trajectory_near_velocity(0),2)/(pow(trajectory_near_velocity(0),2) + pow(trajectory_near_velocity(1),2));
    B = -sqrt(B);
    A = -B*trajectory_near_velocity(1)/trajectory_near_velocity(0);
    C = 0;//铅垂

    double trajectory_vel_norm = trajectory_near_velocity.norm();
    ball_rotm_y = trajectory_near_velocity.array() / (-trajectory_vel_norm);
    ball_rotm_x << A, B, C;
    ball_rotm_z = ball_rotm_x.cross(ball_rotm_y);//默认为列向量(?)

    std::cout<<"ball_rotm_y(v)："<<ball_rotm_y(0,0)<<","<<ball_rotm_y(1,0)<<","<<ball_rotm_y(2,0)<<std::endl
             <<"ball_rotm_x："<<ball_rotm_x(0,0)<<","<<ball_rotm_x(1,0)<<","<<ball_rotm_x(2,0)<<std::endl
             <<"ball_rotm_z："<<ball_rotm_z(0,0)<<","<<ball_rotm_z(1,0)<<","<<ball_rotm_z(2,0)<<std::endl
    <<"-----------------------------------------------"<<std::endl;

    //求坐标转换矩阵
    ball_rotm << ball_rotm_x(0), ball_rotm_y(0), ball_rotm_z(0),
                 ball_rotm_x(1), ball_rotm_y(1), ball_rotm_z(1),
                 ball_rotm_x(2), ball_rotm_y(2), ball_rotm_z(2);


    std::cout<<"ball_rotm旋转矩阵："<<std::endl
    <<ball_rotm(0,0)<<","<<ball_rotm(0,1)<<","<<ball_rotm(0,2)<<std::endl
    <<ball_rotm(1,0)<<","<<ball_rotm(1,1)<<","<<ball_rotm(1,2)<<std::endl
    <<ball_rotm(2,0)<<","<<ball_rotm(2,1)<<","<<ball_rotm(2,2)<<std::endl
    <<"-----------------------------------------------"<<std::endl;
    //求四元数
    ball_quaternion = ball_rotm;
    ball_quaternion.normalize();
    q = ball_quaternion.coeffs();

    m_capture_ball_pose.pose.orientation.w = q(3);//在最后一个
    m_capture_ball_pose.pose.orientation.x = q(0);
    m_capture_ball_pose.pose.orientation.y = q(1);
    m_capture_ball_pose.pose.orientation.z = q(2);
    
    return 1;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void CatchPrediction::PlotTrajectory3D(std::vector<geometry_msgs::PoseStamped> &trajectory_point)
{
  for (int i = 0; i<trajectory_point.size(); i++)
  {
    m_plot_x.push_back(trajectory_point[i].pose.position.x);
    m_plot_y.push_back(trajectory_point[i].pose.position.y);
    m_plot_z.push_back(trajectory_point[i].pose.position.z);
  }
  

  plt::figure();
  // Plot line from given x and y data. Color is selected automatically.
  plt::plot3(m_plot_x, m_plot_y, m_plot_z);
  plt::title("Plot-Trajectory-3D");
  plt::xlabel("x");
  plt::ylabel("y");

  // save figure
  //const char* filename = "./basic" + to_string(mcelex_index) + ".png";
  std::string filename = "./ball_pose_save.png";
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

  // ros::MultiThreadedSpinner s(2);
  ros::Rate loop_rate(2e6/5000);
  while (node_.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}