#include "celex5_ros.h"
static const std::string OPENCV_WINDOW_0 = "Master";
static const std::string OPENCV_WINDOW_1 = "Slave";


#define N_disp  1000

namespace celex5_stereo_multithread { 

typedef struct region_of_interest
{
  double     x;
  double     y;
  double     r;
  uint64_t    time; 
}region_of_interest;

typedef struct Circle2D
{
	double     x; // 是不是该用int16_t
	double     y;
  double     r;
	uint64_t    time; 
  // only for hough
  int     vote;
} Circle2D;

typedef struct CircleStereo
{
  Circle2D    circle_m;
  Circle2D    circle_s;
	uint64_t    time; 
  double      disp;

} CircleStereo;

//自定义排序函数
bool SortRow(const Circle2D &c1, const Circle2D &c2)
{
	return c1.y > c2.y;//降序排列  
}

Eigen::MatrixXd pinv(Eigen::MatrixXd  &A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) 
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());
 
    return X;
}

////////////// Class ////////////////
class Celex5StereoMTEventsCircle{
public:
  // private ROS node handle
  ros::NodeHandle node_[2];
  ros::Subscriber data_sub_[2];
  ros::Publisher image_pub_[2];
  ros::CallbackQueue queue_[2];
  ros::Publisher pose_pub_;

  int event_frame_time = 3000;

  uint32_t master_timestamp_last;
  uint32_t slave_timestamp_last;

  region_of_interest master_roi;
  region_of_interest slave_roi;
  region_of_interest master_roi_last;
  region_of_interest slave_roi_last;

  double master_datasize_ave;
  double slave_datasize_ave;
  int master_dichotomy_flag;
  int slave_dichotomy_flag;

  Circle2D master_circle;
  Circle2D slave_circle;

  cv::Mat master_mat;
  cv::Mat slave_mat;

  std::ofstream master_outfile;   //输出流
  std::ofstream slave_outfile; 

  CircleStereo stereo_pair;
  std::vector<CircleStereo> m_vec_stereo_pair;

  std::ofstream z_camera_left_outfile;

  std::ifstream trajectory_infile;
  double trajectory_test[2001][4];
  int pub_num;

  Circle2D circle_default;

  cv::Point2f master_point;
  cv::Point2f slave_point;

  int master_row_last;
  int master_event_num_frame;
  int slave_row_last;
  int slave_event_num_frame;

  std::vector<double> master_point_save_x;
  std::vector<double> slave_point_save_x;
  std::vector<double> master_point_save_y;
  std::vector<double> slave_point_save_y;

  std::vector<double> master_point_fit_save_x;
  std::vector<double> slave_point_fit_save_x;
  std::vector<double> master_point_fit_save_y;
  std::vector<double> slave_point_fit_save_y;


  int near_index_front;
  int near_index_back;

  int match_num;

  // std::vector<std::vector<EventData>> master_event_queue; 
  std::vector<Circle2D> master_circle_queue; // based on master to match
  std::vector<Circle2D> slave_circle_queue;

  std::vector<double> m_vec_z_camera_left;
  std::vector<double> m_vec_z_camera_left_fit;


  Eigen::MatrixXd A_disp;
  Eigen::MatrixXd B_disp;
  Eigen::Matrix<double, 2, 1> P_disp;


  pthread_mutex_t callback_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t index_lock = PTHREAD_MUTEX_INITIALIZER;

  double LPF_alpha;

  //////////////////////////////////////////kalman////////////////////////////////////////////////////////
  double Q = 0.00010; //Q�󣬲����࣬QС��ƫֱ��
	double R = 1;
	double N = 1;

	double X = 45, P = 1, AA = 1, H = 1;
  double X_k, P_k, Kg, z_k;


  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  void celexDataCallback_0(const celex5_msgs::EventVector &msg);
  void celexDataCallback_1(const celex5_msgs::EventVector &msg);
  void CircleDetect(const celex5_msgs::EventVector &msg, int device_index);
  void ROICheck(region_of_interest &roi, region_of_interest &roi_last, int datasize);
  void CircleFit(std::vector<EventData> &roiEvent, Circle2D &circle);

  void DistortionCorrection(cv::Point2f &uvLeft,cv::Point2f &uvRight);
  
  void Display(const celex5_msgs::EventVector &msg, int device_index);
  void PlotTrajectory2D();
  void Dichotomy(const celex5_msgs::EventVector &msg, region_of_interest &roi, int index);
  int  SpatioTemporalMatch(Circle2D &circle);
  void StereoDispFitDeep(std::vector<CircleStereo> &circlestereo);

private:
  // 圆拟合参数
  double c_E,c_F,c_G,c_H,c_I,c_J,c_K;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd x2;
  Eigen::VectorXd y2;
  Eigen::Matrix<double, 3, 3> c_T1;
  Eigen::Matrix<double, 3, 1> c_T2;
  Eigen::Matrix<double, 3, 1> c_T;

  // 外参/内参,mat形式,还有M矩阵
  cv::Mat mLeftRotation;
  cv::Mat mLeftTranslation;
  cv::Mat mLeftRT;//左相机M矩阵
  cv::Mat mLeftIntrinsic;
  cv::Mat mLeftM;

  cv::Mat mRightRotation;
  cv::Mat mRightTranslation;
  cv::Mat mRightRT;//右相机M矩阵
  cv::Mat mRightIntrinsic;
  cv::Mat mRightM;
  // 畸变参数
  std::vector<float> left_Distort_Coefficients;
  std::vector<float> right_Distort_Coefficients;
  //最小二乘法A,B矩阵
  cv::Mat A;
  cv::Mat B;
  cv::Mat A_p;
  //坐标矩阵
  cv::Mat XYZ;

  double b = 136.1067;
  double disp;
  double disp_tmp;
  double z_camera_left_tmp;
  double x_camera_left_tmp;
  double y_camera_left_tmp;

  int disp_num;
  double fx = 738.0303;
  double u0 = 661.7933;
  double v0 = 401.6159;
  Eigen::Matrix<double, 3, 3> matleftRotation;
  //左相机平移向量
  Eigen::Matrix<double, 3, 1> matleftTranslation;

  double world_x_last;
  double world_y_last;

  // 标定函数
  void SetCalibration();

  // 排序
  // bool SortRow(const Circle2D &c1, const Circle2D &c2);
  void FindTrajectoryHighest();
  void ParabolaFit(std::vector<Circle2D> &queue, Eigen::Matrix<double, 3, 1> &pa) ;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
  Celex5StereoMTEventsCircle(ros::NodeHandle& nh0, ros::NodeHandle& nh1)
  {
    node_[0] = nh0;
    node_[1] = nh1;
    node_[0].setCallbackQueue(&queue_[0]);
    node_[1].setCallbackQueue(&queue_[1]);

    SetCalibration();

    matleftRotation << -0.98009,       0.010089,      -0.19828, 
                        0.19828,      -0.001429,      -0.98015, 
                       -0.01017,      -0.99995,       -0.00059981;
    matleftTranslation << -672.9838, 1739.80244, 834.5714;//771.7037;
    std::cout<<"左相机旋转矩阵 = "<<std::endl<<matleftRotation<<std::endl;
    std::cout<<"左相机平移矢量 = "<<std::endl<<matleftTranslation<<std::endl;

    //
    master_timestamp_last = 0;
    slave_timestamp_last  = 0;
    near_index_front = 0;
    match_num = 0;

    //
    master_datasize_ave = 5000;
    slave_datasize_ave = 6500;
    master_dichotomy_flag = 0;
    slave_dichotomy_flag = 0;

    //
    master_roi.x = 640;
    master_roi.y = 400;
    master_roi.r = 800;
    master_roi_last.x = 0;
    master_roi_last.y = 0;
    master_roi_last.r = 0;

    slave_roi = master_roi;
    slave_roi_last = master_roi_last;

    circle_default.x = 640;
    circle_default.y = 400;
    circle_default.r = 700;

    master_row_last = 0;
    master_event_num_frame = 0;
    slave_row_last = 0;
    slave_event_num_frame = 0;

    LPF_alpha = 0.5;

    // world_x_last = 0;
    world_y_last = 0; // 基本上在0附近

    //最小二乘法A矩阵
    A = cv::Mat(4,3,CV_32F);
    B = cv::Mat(4,1,CV_32F);
    A_p = cv::Mat(3,4,CV_32F);//广义逆
    //坐标矩阵
    XYZ = cv::Mat(3,1,CV_32F);

    disp_num = 0;

    master_mat = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
    slave_mat  = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);

    A_disp = Eigen::MatrixXd::Zero(N_disp,2);
    B_disp = Eigen::MatrixXd::Zero(N_disp,1);

    master_outfile.open("master_point_2d.txt", ios::out);   
    slave_outfile.open("slave_point_2d.txt", ios::out);  
    z_camera_left_outfile.open("disp.txt", ios::out);  

    if(!master_outfile.is_open () || !slave_outfile.is_open () || !z_camera_left_outfile.is_open () )
    {
        std::cout << "Open file failure" << std::endl;
    }        
    else
    {
        std::cout << "Open file succesed" << std::endl;
    }

    trajectory_infile.open("/home/guoyeye/celex_ws/src/celex5_stereo_multithread/src/trajectory_test.txt");//定义读取文件流
    if(!trajectory_infile.is_open ())
    {
        std::cout << "Open file trajectory_test.txt failure" << std::endl;
    }   
    for (int i = 0; i < 2001; i++)//定义行循环
    {
          for (int j = 0; j < 4; j++)//定义列循环
          {
              trajectory_infile >> trajectory_test[i][j];//读取一个值（空格、制表符、换行隔开）就写入到矩阵中，行列不断循环进行
              // std::cout<<trajectory_test[i][j]<<" ";
          }
          // std::cout<<std::endl;
    }
    trajectory_infile.close();
    

    pub_num = 0;

    ///////////////////////////////////////////////////////////////////////

    // pub the pose
    pose_pub_ = node_[0].advertise<geometry_msgs::PoseStamped>("celex5_stereo/ball_pose", 100);

    // sleep(10);
    // ROS_INFO("Start detection");

    // subscribe the data
    data_sub_[0] = node_[0].subscribe(
        "celex5_stereo/0/events", 0, &Celex5StereoMTEventsCircle::celexDataCallback_0, this);
    // sleep(0.5); // 让从相机进入的时间一定比主相机第一个大
    data_sub_[1] = node_[1].subscribe(
        "celex5_stereo/1/events", 0, &Celex5StereoMTEventsCircle::celexDataCallback_1, this);// 0 很关键，就是保留所有数据的意思

    struct timeval t0, t1;
    double deltaT;



    gettimeofday(&t0, NULL);//总计时器，超过一定数目，暂停
    int deal_0_num = 0;
      
    while(node_[0].ok() && node_[1].ok())
    {
      // std::cout<<1<<std::endl;
      // queue_[0].callOne(ros::WallDuration(1.0)); // can also be callAvailable()
      queue_[0].callAvailable(ros::WallDuration(1.0)); // 为了保证0中有很多数，不然1到了但0里根本没有
      deal_0_num++;
      
      if (deal_0_num > 2)
      {
        queue_[1].callOne(ros::WallDuration(1.0)); // can also be callAvailable()
      }
      
      // queue_[0].callAvailable(ros::WallDuration(1.0));
      // queue_[1].callAvailable(ros::WallDuration(1.0));

      gettimeofday(&t1, NULL);//计时器
      deltaT = (t1.tv_sec-t0.tv_sec);//只看s
      if (deltaT > 28)//0s
      {
        ROS_INFO("Program break down: time too long!");
        
        // ros::shutdown();
        // return 0;
        break;
      }
    }
    // 这个地方可以写成双线程，两个回调队列用两个线程的spin，这个时候就需要上锁。
    
  }

  ~Celex5StereoMTEventsCircle()
  {
    // 求最高点时间及时间差
    // FindTrajectoryHighest();
    // cv::destroyWindow(OPENCV_WINDOW_0);
    // cv::destroyWindow(OPENCV_WINDOW_1);
    PlotTrajectory2D();
    master_outfile.close();//关闭文件
    slave_outfile.close();

    usleep(5000);
    pthread_mutex_destroy(&callback_lock);
    pthread_mutex_destroy(&index_lock);

  }


};


void Celex5StereoMTEventsCircle::FindTrajectoryHighest() 
{
  std::sort(master_circle_queue.begin(), master_circle_queue.end(), SortRow);
  std::sort(slave_circle_queue.begin(), slave_circle_queue.end(), SortRow);

  double master_highest_time_add;
  double slave_highest_time_add;

  for(int i = 0; i<40; i++)
  {
    master_highest_time_add += master_circle_queue[i].time;
    std::cout<<"[0]"<<master_circle_queue[i].time<<std::endl;

    slave_highest_time_add += slave_circle_queue[i].time;
    std::cout<<"[1]"<<slave_circle_queue[i].time<<std::endl;

  }

  std::cout<<"stereo time diff: "<<(master_highest_time_add - slave_highest_time_add)/40<<std::endl;

  for(int i = 0; i<master_circle_queue.size(); i++)
  {
    master_circle_queue[i].time -=  master_circle_queue[0].time;// 反正是洗沟
    slave_circle_queue[i].time -=  master_circle_queue[0].time;// 反正是洗沟
  }


  Eigen::Matrix<double, 3, 1> pa0;
  Eigen::Matrix<double, 3, 1> pa1;

  ParabolaFit(master_circle_queue,pa0);
  ParabolaFit(slave_circle_queue,pa1);

  double master_max_time = -pa0(1,0)/(2*pa0(0,0));
  double slave_max_time = -pa1(1,0)/(2*pa1(0,0));

  std::cout<<"stereo time diff fitted: "<<master_max_time - slave_max_time<<std::endl;
}

void Celex5StereoMTEventsCircle::ParabolaFit(std::vector<Circle2D> &queue, Eigen::Matrix<double, 3, 1> &pa) 
{
  int N = queue.size();

  if (N < 510)
  {
    return;
  }
  
  // Eigen::VectorXd center_circle_x = VectorXd::Map(&master_circle_queue[0].x,master_circle_queue.size());//转化成向量x
  // Eigen::VectorXd center_circle_y = VectorXd::Map(&master_circle_queue[0].y,master_circle_queue.size());//转化成向量y
  Eigen::VectorXd center_circle_time = Eigen::VectorXd::LinSpaced(N-500, 0, 0);;
  Eigen::VectorXd center_circle_y = Eigen::VectorXd::LinSpaced(N-500, 0, 0);;
  
  for (int i = 0; i<N-500; i++)
  {
    center_circle_time(i) = queue[i].time;
    center_circle_y(i) = queue[i].y;
  }

  // std::cout<<111<<std::endl;

  double d = center_circle_time.array().pow(4).sum();
  double e = center_circle_time.array().pow(3).sum();
  double f = center_circle_time.array().pow(2).sum();
  double g = center_circle_time.sum();
  Eigen::VectorXd R = center_circle_time.array().pow(2);
  double h = center_circle_y.dot(R);
  double i = center_circle_time.dot(center_circle_y);
  double j = center_circle_y.sum();
  Eigen::Matrix<double, 3, 3> P;
  P << d,e,f,
      e,f,g,
      f,g,N;
  Eigen::Matrix<double, 3, 1> Q;
  Q << h,j,j;
  pa = P.inverse()*Q;

}

//////////////////////////////////////////////////////////////////////
void Celex5StereoMTEventsCircle::Dichotomy(
    const celex5_msgs::EventVector &msg, region_of_interest &roi,int index) 
{
  if (index == 0)
  {
    for (int kk=0; kk<20;kk++)
    {std::cout<<"[0]ball come!"<<std::endl;}
  }
  else
  {
    for (int kk=0; kk<20;kk++)
    {std::cout<<"[1]ball come!"<<std::endl;}
  }

  int block;// 直接用整数取整
  int N = 20;
  int block_vote[N] = {0};// 之前是20
  // 小球一定从右边来（图像平面，不是镜像的），所以先分y，分成份
  for (int i = 0; i < msg.vector_length; i++)
  {
    
    if(msg.events[i].x>(1280-(800/N)*3) )
    {
      block = msg.events[i].y/(800/N); 
      block_vote[block] = block_vote[block]+1;
    }
  }
  // 找到最大block
  int max_block_index = 0;
  int max_block_num = 0;
  for (int j=0; j<N; j++)
  {
    if(block_vote[j]>max_block_num)
    {
      max_block_num = block_vote[j];
      max_block_index = j;
    }
  }
  roi.x = 1180;
  roi.r = (800/N)*3/2 ;
  roi.y = max_block_index*(800/N)+(800/N)/2;

  std::cout<<"max_block_num"<<max_block_num<<std::endl;
  std::cout<<"max_block_index"<<max_block_index<<std::endl;



}
/////////////////////////// callback for data_sub_[0]/////////////////////////////
void Celex5StereoMTEventsCircle::celexDataCallback_0(
    const celex5_msgs::EventVector &msg) 
{

  // if (msg.events[0].off_pixel_timestamp - master_timestamp_last > 11000)
  // {
  //   std::cout<<"/////////////////////////////////// 0 ERROR! ////////////////////////////////////////////"<<std::endl;
  // }
  // master_timestamp_last = msg.events[0].off_pixel_timestamp;

  // 判断是不是第一次事件激增，代表小球进入，采用二分法找初始ROI
  if (master_dichotomy_flag == 0)
  {
    if(msg.vector_length > (master_datasize_ave + 1500)) // 可以改
    {
      
      Dichotomy(msg, master_roi, 0);
      // 得到一个大概率的ROI

      // 标志位置1
      master_dichotomy_flag = 1; // 即第一次出现小球

    }
    else
    {
      // 平均计算
      master_datasize_ave = 0.95 * master_datasize_ave + 0.05 * msg.vector_length; // double
    }

  }


  std::vector<EventData> roiEvent;
  for (int i = 0; i < msg.vector_length; i++)
  {
    if(msg.events[i].y < master_row_last )//如果小于上一次的行数，则说明发生下一次行循环,开始对上一次循环处理，理论上可以不用等循环完，但我写不来程序
    {
        if (master_event_num_frame <= 300)//不画图，不拟合
        {
        }
        else
        //当roi内的事件大于10才拟合，每拟合一次更新一次ROI
        {
            if (roiEvent.size() <= 15)
            {
              master_event_num_frame = 0;
              roiEvent.clear();
            }
            else//这个时候才拟合
            {
              CircleFit(roiEvent,master_circle);
              uint64 time_sum = 0;
              for(int t_i = 0; t_i<roiEvent.size(); t_i++)
              {
                time_sum += roiEvent[t_i].t_increasing;
              }
              master_circle.time = time_sum/roiEvent.size();
              
              master_roi.x = master_circle.x;
              master_roi.y = master_circle.y;
              master_roi.r = 1.5*(master_circle.r+7);
              ROICheck(master_roi, master_roi_last, 0);
              master_roi_last = master_roi;

              
              
              if(master_circle.r > 10 && master_circle.r < 28) // 之前是80
              {
                master_outfile << master_circle.time <<","<< master_circle.x <<","<< master_circle.y<<","<<master_circle.r<< std::endl;  //在txt中写入结果
                std::cout<<"[0]:"<<"circle's t_increasing:"<<master_circle.time<<","
                      <<"x:"<<master_circle.x<<","
                      <<"y:"<<master_circle.y<<","
                      <<"r:"<<master_circle.r
                      <<std::endl;  
                master_point_save_x.push_back(master_circle.x);
                master_point_save_y.push_back(master_circle.y);

                  
                // // 保存检测到的所有mster_circle，先滤波还是先改roi-------------------暂时不开滤波，可能会影响匹配
                // if (master_circle_queue.empty())
                // {
                //   master_circle_queue.push_back(master_circle); // empl
                // }
                // else
                // {
                //   master_circle.x = LPF_alpha*master_circle.x + (1-LPF_alpha)*master_circle_queue[master_circle_queue.size()-1].x;
                //   master_circle.y = LPF_alpha*master_circle.y + (1-LPF_alpha)*master_circle_queue[master_circle_queue.size()-1].y;
                  master_circle_queue.push_back(master_circle); 
                // }

                master_point_fit_save_x.push_back(master_circle.x);
                master_point_fit_save_y.push_back(master_circle.y); 
              }
              
              master_event_num_frame = 0;
              roiEvent.clear();
            }          
        }
  
    }

    master_row_last = msg.events[i].y; //记录上一次的行数
    master_event_num_frame++;

    if (pow((msg.events[i].x-master_roi.x),2) + pow((msg.events[i].y-master_roi.y),2) < pow(master_roi.r,2) )
    {
      EventData Event;
      Event.col = msg.events[i].x;
      Event.row = msg.events[i].y;
      Event.t_increasing = msg.events[i].off_pixel_timestamp;
      roiEvent.emplace_back(Event);
    }
  }
}

/////////////////////////// callback for data_sub_[1]/////////////////////////////
void Celex5StereoMTEventsCircle::celexDataCallback_1(
    const celex5_msgs::EventVector &msg) 
{
  if (slave_dichotomy_flag == 0)
  {
    if(msg.vector_length > (slave_datasize_ave + 1500)) // 可以改
    {
      
      Dichotomy(msg, slave_roi,1);
      // 得到一个大概率的ROI

      // 标志位置1
      slave_dichotomy_flag = 1; // 即第一次出现小球

    }
    else
    {
      // 平均计算
      slave_datasize_ave = 0.95 * slave_datasize_ave + 0.05 * msg.vector_length; // double
    }

  }

  std::vector<EventData> roiEvent;
  for (int i = 0; i < msg.vector_length; i++)
  {
    if(msg.events[i].y < slave_row_last )//如果小于上一次的行数，则说明发生下一次行循环,开始对上一次循环处理，理论上可以不用等循环完，但我写不来程序
    {
        if (slave_event_num_frame <= 300)//不画图，不拟合
        {
        }
        else
        //当roi内的事件大于10才拟合，每拟合一次更新一次ROI
        {
            if (roiEvent.size() <= 15)
            {
              slave_event_num_frame = 0;
              roiEvent.clear();
            }
            else//这个时候才拟合
            {
              CircleFit(roiEvent,slave_circle);
              uint64 time_sum = 0;
              for(int t_i = 0; t_i<roiEvent.size(); t_i++)
              {
                time_sum += roiEvent[t_i].t_increasing;
              }
              slave_circle.time = time_sum/roiEvent.size();

              slave_roi.x = slave_circle.x;
              slave_roi.y = slave_circle.y;
              slave_roi.r = 1.5*(slave_circle.r+7);
              ROICheck(slave_roi, slave_roi_last, 0);
              slave_roi_last = slave_roi;
              
              if(slave_circle.r > 10  && slave_circle.r < 28) // 之前是80
              {
                slave_outfile << slave_circle.time <<","<< slave_circle.x <<","<< slave_circle.y<<","<<slave_circle.r<< std::endl;  //在txt中写入结果
                std::cout<<"[1]:"<<"circle's t_increasing:"<<slave_circle.time<<","
                      <<"x:"<<slave_circle.x<<","
                      <<"y:"<<slave_circle.y<<","
                      <<"r:"<<slave_circle.r
                      <<std::endl;
                slave_point_save_x.push_back(slave_circle.x);
                slave_point_save_y.push_back(slave_circle.y);
                
                // // 保存检测到的所有slave_circle，先滤波
                // if (slave_circle_queue.empty())
                // {
                //   slave_circle_queue.push_back(slave_circle); 
                // }
                // else
                // {
                //   slave_circle.x = LPF_alpha*slave_circle.x + (1-LPF_alpha)*slave_circle_queue[slave_circle_queue.size()-1].x;
                //   slave_circle.y = LPF_alpha*slave_circle.y + (1-LPF_alpha)*slave_circle_queue[slave_circle_queue.size()-1].y;
                  slave_circle_queue.push_back(slave_circle); 

                  // 在这个地方写匹配函数
                  // geometry_msgs::PoseStamped pose_msg;
                  int ret = SpatioTemporalMatch(slave_circle);

                  if (m_vec_stereo_pair.size() == N_disp)
                  {
                    // 拟合，然后发600个
                    StereoDispFitDeep(m_vec_stereo_pair);

                  }
                  else if(m_vec_stereo_pair.size() > N_disp)
                  {
                    if (ret)
                    {
                      m_vec_z_camera_left.push_back(fx*b/m_vec_stereo_pair[m_vec_stereo_pair.size()-1].disp);
                      m_vec_z_camera_left_fit.push_back((m_vec_stereo_pair[m_vec_stereo_pair.size()-1].time - m_vec_stereo_pair[0].time)*P_disp(0,0) + P_disp(1,0));
                      // z_camera_left_outfile << fx*b/m_vec_stereo_pair[m_vec_stereo_pair.size()-1].disp <<","
                      //                       << m_vec_stereo_pair[m_vec_stereo_pair.size()-1].time*P_disp(0,0) + P_disp(1,0)) << std::endl;
                      std::cout<<m_vec_stereo_pair[m_vec_stereo_pair.size()-1].disp<<std::endl;

                    }
                  }

                  // // 先pub一个人工生成的轨迹    
                  // ret = 1;    

                  // if (ret == 1)
                  // {        
                  // // pose_msg.pose.position.x = trajectory_test[pub_num][0] - 0.2;
                  // // pose_msg.pose.position.y = trajectory_test[pub_num][1] - 0.1;
                  // // pose_msg.pose.position.z = trajectory_test[pub_num][2] - 0.2 ; // 修正，不然机械臂逆解没有值
                  // // pose_msg.header.stamp.sec = 0;
                  // // if (trajectory_test[pub_num][3]<0)
                  // //   pose_msg.header.stamp.nsec = 0;
                  // // else
                  // //   pose_msg.header.stamp.nsec = trajectory_test[pub_num][3]*1e9;

                  // // std::cout<<"pose_msg.header.stamp:"<<pose_msg.header.stamp<<std::endl;
                  // // std::cout<<"pose_msg.header.stamp.sec:"<<pose_msg.header.stamp.sec<<std::endl;
                  // // std::cout<<"pose_msg.header.stamp.nsec:"<<pose_msg.header.stamp.nsec<<std::endl;
                  // // std::cout<<"pose_msg.header.stamp.nsec:"<<trajectory_test[pub_num][3]<<std::endl;
                  // // std::cout<<"pose_msg.header.stamp.nsec:"<<trajectory_test[pub_num][3]*1e9<<std::endl;

                  //   pose_pub_.publish(pose_msg);
                  //   pub_num++;
                  //   std::cout<<"pub!"<<std::endl;
                  // }
                  
                // }
                slave_point_fit_save_x.push_back(slave_circle.x);
                slave_point_fit_save_y.push_back(slave_circle.y);
                
              }
              
              slave_event_num_frame = 0;
              roiEvent.clear();
            }          
        }
    }
    slave_row_last = msg.events[i].y; //记录上一次的行数
    slave_event_num_frame++;

    if (pow((msg.events[i].x-slave_roi.x),2) + pow((msg.events[i].y-slave_roi.y),2) < pow(slave_roi.r,2) )
    {
      EventData Event;
      Event.col = msg.events[i].x;
      Event.row = msg.events[i].y;
      Event.t_increasing = msg.events[i].off_pixel_timestamp;
      roiEvent.emplace_back(Event);
    }
  }
}
//////////////////////////// CircleDetect ////////////////////////////////
void Celex5StereoMTEventsCircle::CircleDetect(const celex5_msgs::EventVector &msg, int device_index)
{

}
//
void Celex5StereoMTEventsCircle::ROICheck(region_of_interest &roi, region_of_interest &roi_last, int datasize)
{
  // if(datasize < 4200)
  // {
  //   roi.x = 1000;
  //   roi.y = 400;
  //   roi.r = 500;
  // }
  // else
  // {
    if(roi.r <= 10)
    {
      roi.r = 700;
    }
    if(roi.x < 5 || roi.x >= MAT_COLS-5 ||
       roi.y < 5 || roi.y >= MAT_ROWS-5)
    {
      roi.x = 640;
      roi.y = 400;
    }
    // if (roi.r == roi_last.r && roi.x == roi_last.x)
    // {
    //   roi.r = 2*roi.r;
    // }
  // }
  
}
//
void Celex5StereoMTEventsCircle::CircleFit(std::vector<EventData> &roiEvent,
                                           Circle2D &circle)
{
  int dataSize = roiEvent.size();
  x = Eigen::VectorXd::Zero(dataSize);
  y = Eigen::VectorXd::Zero(dataSize);
    
  for (int i = 0; i < dataSize; i++) 
	{  
        x[i] = roiEvent[i].col;
        y[i] = roiEvent[i].row;
  }
    c_E = x.array().pow(2).sum();
    c_F = x.dot(y);
    c_G = y.array().pow(2).sum();
    c_H = x.sum();
    c_I = y.sum();
    x2 = x.array().pow(2);
    y2 = y.array().pow(2);
    c_J = x.array().pow(3).sum() + x.dot(y2);
    c_K = y.array().pow(3).sum() + y.dot(x2);
    c_T1 << c_E, c_F, c_H,
                     c_F, c_G, c_I,
                     c_H, c_I, dataSize;
    c_T2 << c_J, c_K,c_E + c_G;
    c_T = -c_T1.inverse()*c_T2;
    //
    circle.x = -c_T(0,0)/2;
    circle.y = -c_T(1,0)/2;
    circle.r = sqrt(c_T(0,0)*c_T(0,0) + c_T(1,0)*c_T(1,0) - 4*c_T(2,0))/2;
   
}
//
int Celex5StereoMTEventsCircle::SpatioTemporalMatch(Circle2D &circle)
{
  if(master_circle_queue.empty())
  {
    return 0;
  }

  int ridge_flag = 0;
  int find_index = near_index_front;
  // std::cout<<"进入时刻的near_index_front： "<<near_index_front<<std::endl;
  std::cout<<"进入时刻的master_circle_queue长度： "<<master_circle_queue.size()<<std::endl;

  while(1)
  {
    if (find_index > master_circle_queue.size()-1)
    {
      std::cout<<"一直比master最后一个时间大！"<<master_circle_queue.size()<<std::endl;
      return 0;
    }

    if(circle.time > master_circle_queue[find_index].time) 
    {
      if(ridge_flag == 0)
      {
        ridge_flag = 1;
      }
    }
    else if(circle.time < master_circle_queue[find_index].time)
    {
      if(ridge_flag == 0)
      {
        break;
      }
      else
      {
        near_index_front = find_index-1;
        near_index_back = find_index;
        // std::cout<<"near_index_front： "<<near_index_front<<std::endl;
        // std::cout<<"near_index_back "<<near_index_back<<std::endl;

        // double master_y_insert = master_circle_queue[near_index_front].y + 
        //                       (master_circle_queue[near_index_back].y - master_circle_queue[near_index_front].y) * 
        //                       ((circle.time - master_circle_queue[near_index_front].time)/
        //                       (master_circle_queue[near_index_back].time - master_circle_queue[near_index_front].time));   
        // 我不知道这个地方要不要800 - y - 1;
        double master_x_insert = master_circle_queue[near_index_front].x + 
                              (master_circle_queue[near_index_back].x - master_circle_queue[near_index_front].x) * 
                              ((circle.time - master_circle_queue[near_index_front].time)/
                              (master_circle_queue[near_index_back].time - master_circle_queue[near_index_front].time));

        disp = circle.x - master_x_insert - 7; // slave in left
        stereo_pair.disp = disp;

        // stereo_pair.circle_m.x = master_x_insert;
        // stereo_pair.circle_m.y = 800 - master_y_insert - 1;

        stereo_pair.circle_s.x = circle.x;
        stereo_pair.circle_s.y = 800 - circle.y - 1;

        stereo_pair.time = circle.time;

        m_vec_stereo_pair.push_back(stereo_pair);

        std::cout<<"matched row: "<<circle.y<<"----"<<master_circle_queue[near_index_front].y<<"//"<<master_circle_queue[near_index_back].y<<std::endl;

        // StereoMatch(slave_point,master_point, pose_msg);// 感觉立体匹配更偏向于用视差的那个方法，可以重载
        // StereoDispDeep(slave_point,master_point, pose_msg);

        break;
      }
    }
    else // time == time
    {
      near_index_front = find_index;

      // DistortionCorrection();
      // StereoCorrection();

      disp = circle.x - master_circle_queue[near_index_front].x - 7; // slave in left
      stereo_pair.disp = disp;

      // stereo_pair.circle_m.x = master_circle_queue[near_index_front].x;
      // stereo_pair.circle_m.y = 800 - master_circle_queue[near_index_front].y - 1;

      stereo_pair.circle_s.x = circle.x;
      stereo_pair.circle_s.y = 800 - circle.y - 1;

      stereo_pair.time = circle.time;

      m_vec_stereo_pair.push_back(stereo_pair);

      std::cout<<"matched row: "<<circle.y<<"----"<<master_circle_queue[near_index_front].y<<std::endl;


      // StereoMatch(slave_point, master_point, pose_msg); 
      // StereoDispDeep(slave_point, master_point, pose_msg); 
      break;

    }
    find_index++;
  }

    return 1;
}
//
void Celex5StereoMTEventsCircle::Display(
    const celex5_msgs::EventVector &msg, int device_index) 
{
  if (device_index == 0)
  {
    for (int i = 0; i < msg.vector_length; i++)
    {
      master_mat.at<uchar>(MAT_ROWS - msg.events[i].y - 1,
                      MAT_COLS - msg.events[i].x - 1) = 255;	
    }
    //roi
    cv::circle(master_mat, 
              cv::Point(MAT_COLS - master_roi.x -1, 
                    MAT_ROWS - master_roi.y - 1), 
              master_roi.r, 
              cv::Scalar::all(100));
    //circle
    cv::circle(master_mat, 
              cv::Point(MAT_COLS - master_circle.x -1, 
                    MAT_ROWS - master_circle.y - 1), 
              master_circle.r, 
              cv::Scalar::all(100));
    cv::imshow(OPENCV_WINDOW_0, master_mat);
    cv::waitKey(1);
    master_mat = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
  }
  else
  {
    for (int i = 0; i < msg.vector_length; i++)
    {
      slave_mat.at<uchar>(MAT_ROWS - msg.events[i].y - 1,
                      MAT_COLS - msg.events[i].x - 1) = 255;	
    }
    //roi
    cv::circle(slave_mat, 
              cv::Point(MAT_COLS - slave_roi.x -1, 
                    MAT_ROWS - slave_roi.y - 1), 
              slave_roi.r, 
              cv::Scalar::all(100));
    //circle
    cv::circle(slave_mat, 
              cv::Point(MAT_COLS - slave_circle.x -1, 
                    MAT_ROWS - slave_circle.y - 1), 
              slave_circle.r, 
              cv::Scalar::all(100));
    // cv::imshow(OPENCV_WINDOW_1, slave_mat);
    // cv::waitKey(1);
    slave_mat = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
  }

}
///////////////////////// 标定参数加载 //////////////////////////
void Celex5StereoMTEventsCircle::SetCalibration()
{
  //在构造函数开始时加载标定文件，标定文件储存在公有成员
  //默认值，单位统一一下，mm
  //左相机内参数矩阵
  float leftIntrinsic[3][3] = {738.0303,     1.1254,        661.7933,
                              0,             736.0541,      401.6159,
                              0,             0,             1};
  //左相机畸变系数
  float leftDistortion[5] = {-0.1802, 0.1251, 0.0012, -0.0056, -0.0747};
  left_Distort_Coefficients.push_back(leftDistortion[0]);
  left_Distort_Coefficients.push_back(leftDistortion[1]);
  left_Distort_Coefficients.push_back(leftDistortion[2]);
  left_Distort_Coefficients.push_back(leftDistortion[3]);
  left_Distort_Coefficients.push_back(leftDistortion[4]);

  //左相机旋转矩阵
  float leftRotation[3][3] = {-0.98009,       0.010089,      -0.19828, 
                               0.19828,      -0.001429,      -0.98015, 
                              -0.01017,      -0.99995,       -0.00059981};// 平行放置，标定后要改
  //左相机平移向量
  float leftTranslation[1][3] = {-672.9838, 1739.80244, 834.5714};//771.7037
  
  //右相机内参数矩阵
  float rightIntrinsic[3][3] = {736.5727,     2.2904,       655.0672,
                                  0,          736.2919,     396.4308,
                                  0,          0,            1};                                   
  //右相机畸变系数
  float rightDistortion[5] = {-0.1642,	-0.0701, 0.0018, -0.0063, 0.0147};
  right_Distort_Coefficients.push_back(rightDistortion[0]);
  right_Distort_Coefficients.push_back(rightDistortion[1]);
  right_Distort_Coefficients.push_back(rightDistortion[2]);
  right_Distort_Coefficients.push_back(rightDistortion[3]);
  right_Distort_Coefficients.push_back(rightDistortion[4]);

  //右相机旋转矩阵
  float rightRotation[3][3] = {-0.98016,      0.027805,        -0.19624, 
                                0.19637,      0.001872,        -0.98053, 
                               -0.02691,     -0.99961,         -0.007251};
  //右相机平移向量
  float rightTranslation[1][3] = {-809.6244, 1763.30691, 833.2683};//771.6577

  mLeftRotation = cv::Mat(3,3,CV_32F,leftRotation);
  mLeftTranslation = cv::Mat(3,1,CV_32F,leftTranslation);
  // 求逆
  mLeftRT = cv::Mat(3,4,CV_32F);//左相机RT矩阵
  hconcat(mLeftRotation.t(),-mLeftRotation.t()*mLeftTranslation,mLeftRT);
  mLeftIntrinsic = cv::Mat(3,3,CV_32F,leftIntrinsic);
  mLeftM = mLeftIntrinsic * mLeftRT;
  std::cout<<"左相机M矩阵 = "<<std::endl<<mLeftM<<std::endl;
  // std::cout<<"左相机内参矩阵 = "<<std::endl<<mLeftIntrinsic<<std::endl;
  // std::cout<<"左相机旋转矩阵 = "<<std::endl<<mLeftRotation<<std::endl;
  // std::cout<<"左相机平移矢量 = "<<std::endl<<mLeftTranslation<<std::endl;

  mRightRotation = cv::Mat(3,3,CV_32F,rightRotation);
  mRightTranslation = cv::Mat(3,1,CV_32F,rightTranslation);
  mRightRT = cv::Mat(3,4,CV_32F);//右相机RT矩阵
  hconcat(mRightRotation.t(),-mRightRotation.t()*mRightTranslation,mRightRT);
  mRightIntrinsic = cv::Mat(3,3,CV_32F,rightIntrinsic);
  mRightM = mRightIntrinsic * mRightRT;
  std::cout<<"右相机M矩阵 = "<<std::endl<<mRightM<<std::endl;
}
///////////////////////// 畸变矫正 //////////////////////////
void Celex5StereoMTEventsCircle::DistortionCorrection(cv::Point2f &uvLeft,cv::Point2f &uvRight)
{
    // std::vector<cv::Point2f> CenterPnt ;
    // cv::undistortPoints(CenterPnt, CenterPnt,
    // mLeftIntrinsic, left_Distort_Coefficients,
    // noArray(),mLeftIntrinsic);

}

void Celex5StereoMTEventsCircle::StereoDispFitDeep(std::vector<CircleStereo> &circlestereo)
{
  double z_camera_left;
  double x_camera_left;
  double y_camera_left;

  for (int i=0; i<circlestereo.size(); i++ )
  {
    A_disp(i,0) = circlestereo[i].time - circlestereo[0].time;
    A_disp(i,1) = 1;
    B_disp(i,0) = fx*b/circlestereo[i].disp; // 拟合深度
    m_vec_z_camera_left.push_back(B_disp(i,0));
    // std::cout<<B_disp(i,0)<<std::endl;
    
  }
  P_disp = pinv(A_disp)*B_disp;

  Eigen::Matrix<double, N_disp, 1> F;
  F = A_disp * P_disp;

  for (int i=0; i<F.size(); i++ )
  {
    z_camera_left = F(i,0); 
    x_camera_left = (circlestereo[i].circle_s.x-u0)*F(i,0)/fx;
    y_camera_left = (circlestereo[i].circle_s.y-v0)*F(i,0)/fx;

    m_vec_z_camera_left_fit.push_back(z_camera_left);
    // std::cout<<z_camera_left<<std::endl;
    z_camera_left_outfile << B_disp(i,0) <<","<< z_camera_left << std::endl;

    Eigen::Matrix<double, 3, 1> XYZ_camera;
    Eigen::Matrix<double, 3, 1> XYZ_world;

    XYZ_camera << x_camera_left, y_camera_left, z_camera_left;
    XYZ_world = matleftRotation*XYZ_camera + matleftTranslation;

    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.stamp.sec = floor(circlestereo[i].time/1e6);
    pose_msg.header.stamp.nsec = (circlestereo[i].time%1000000)*1e3;

    //世界坐标系中坐标
    pose_msg.pose.position.x = XYZ_world(0,0)/1000;
    pose_msg.pose.position.y = XYZ_world(1,0)/1000;
    pose_msg.pose.position.z = XYZ_world(2,0)/1000;

    pose_pub_.publish(pose_msg);
    pub_num++;
    std::cout<<"pub!"<<std::endl;
  }

  
}
//////////////////////////////////////////////////////////////////////

void Celex5StereoMTEventsCircle::PlotTrajectory2D()
{
  //plot
  // Set the size of output image =  pixels
  plt::figure_size(1280, 800);
  // Plot line from given x and y data. Color is selected automatically.
  plt::plot(master_point_save_x, master_point_save_y,".");
  plt::plot(master_point_fit_save_x, master_point_fit_save_y,"r");
  plt::xlim(0,1280);
  plt::ylim(0,800);
  plt::title("Plot-Trajectory-2D");
  plt::xlabel("master_x");
  plt::ylabel("master_y");  
  // plt::show();
  // save figure
  //const char* filename = "./basic" + to_string(mcelex_index) + ".png";
  std::string filename = "./master_point.png";
  std::cout << "Saving result to " << filename << std::endl;
  plt::save(filename);

  // plt::figure_size(1280, 800);
  // // Plot line from given x and y data. Color is selected automatically.
  // plt::plot(slave_point_save_x, slave_point_save_y,".");
  // plt::plot(slave_point_fit_save_x, slave_point_fit_save_y,"r");
  // plt::xlim(0,1280);
  // plt::ylim(0,800);
  // plt::title("Plot-Trajectory-2D");
  // plt::xlabel("slave_x");
  // plt::ylabel("slave_y");  
  // // plt::show();
  // // save figure
  // //const char* filename = "./basic" + to_string(mcelex_index) + ".png";
  // filename = "./slave_point.png";
  // std::cout << "Saving result to " << filename << std::endl;
  // plt::save(filename);

  // plt::figure_size(1280, 800);
  // // Plot line from given x and y data. Color is selected automatically.
  // // plt::plot(master_point_save_x, master_point_save_y,".");
  // plt::plot(master_point_fit_save_x, master_point_fit_save_y,"r");
  // // plt::plot(slave_point_save_x, slave_point_save_y,".");
  // plt::plot(slave_point_fit_save_x, slave_point_fit_save_y,"r");

  // plt::xlim(0,1280);
  // plt::ylim(0,800);
  // plt::title("Plot-Trajectory-2D-MS");
  // plt::xlabel("x");
  // plt::ylabel("y");  
  // // plt::show();
  // // save figure
  // //const char* filename = "./basic" + to_string(mcelex_index) + ".png";
  // filename = "./all_point.png";
  // std::cout << "Saving result to " << filename << std::endl;
  // plt::save(filename);

  // plt::figure_size(1280, 800);
  // // Plot line from given x and y data. Color is selected automatically.
  // plt::plot(m_vec_z_camera_left,".");
  // plt::plot(m_vec_z_camera_left_fit,"r");
  // plt::xlim(0,1280);
  // plt::title("z_camera_left");
  // filename = "./z_camera_left.png";
  // std::cout << "Saving result to " << filename << std::endl;
  // plt::save(filename);


}

}// namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "celex5_stereo_multithread_event_circle");
  ros::NodeHandle node_0;
  ros::NodeHandle node_1;

  celex5_stereo_multithread::Celex5StereoMTEventsCircle *csc =
      new celex5_stereo_multithread::Celex5StereoMTEventsCircle(node_0, node_1);

  // ros::MultiThreadedSpinner s(2);
  // // ros::Rate loop_rate(10e6/5000); // 接受端的速度要调整一下，必须比发送要快，不然会掉包，而且要留足够的内存

  // ros::spin(s);

  // while (ros::ok()) {
  //   // ros::spinOnce();
  //   // // loop_rate.sleep();
  // }
  delete csc;
  return EXIT_SUCCESS;
}