#include "celex5_ros.h"
static const std::string OPENCV_WINDOW_0 = "Master";
static const std::string OPENCV_WINDOW_1 = "Slave";

namespace celex5_stereo_multithread { 

typedef struct region_of_interest
{
  int     x;
  int     y;
  int     r;
  uint64_t    time; 
}region_of_interest;

typedef struct Circle2D
{
	int     x;
	int     y;
  int     r;
	uint64_t    time; 
  // only for hough
  int     vote;
} Circle2D;



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
  Circle2D master_circle;
  Circle2D slave_circle;
  cv::Mat master_mat;
  cv::Mat slave_mat;
  std::ofstream master_outfile;   //输出流
  std::ofstream slave_outfile; 

  Circle2D circle_default;

  cv::Point2f master_point;
  cv::Point2f slave_point;

  std::vector<double> master_point_save_x;
  std::vector<double> slave_point_save_x;
  std::vector<double> master_point_save_y;
  std::vector<double> slave_point_save_y;

  int near_index_front;
  int near_index_back;

  int match_num;

  // std::vector<std::vector<EventData>> master_event_queue; 
  std::vector<Circle2D> master_circle_queue; // master first come
  pthread_mutex_t callback_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t index_lock = PTHREAD_MUTEX_INITIALIZER;


  Celex5StereoMTEventsCircle(ros::NodeHandle& nh0, ros::NodeHandle& nh1)
  {
    node_[0] = nh0;
    node_[1] = nh1;
    node_[0].setCallbackQueue(&queue_[0]);
    node_[1].setCallbackQueue(&queue_[1]);

    SetCalibration();
    //
    master_timestamp_last = 0;
    slave_timestamp_last  = 0;
    near_index_front = 0;
    match_num = 0;

    //
    master_roi.x = 640;
    master_roi.y = 400;
    master_roi.r = 800;
    slave_roi = master_roi;
    circle_default.x = 640;
    circle_default.y = 400;
    circle_default.r = 700;

    master_mat = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
    slave_mat  = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);

    master_outfile.open("master_point_2d.txt", ios::out);   
    slave_outfile.open("slave_point_2d.txt", ios::out);  
    if(!master_outfile.is_open () || !slave_outfile.is_open ())
    {
        std::cout << "Open file failure" << std::endl;
    }        
    else
    {
        std::cout << "Open file succesed" << std::endl;
    }

    // pub the pose
    pose_pub_ = node_[0].advertise<geometry_msgs::PoseStamped>("celex5_stereo/ball_pose", 10);

    // subscribe the data
    data_sub_[0] = node_[0].subscribe(
        "celex5_stereo/0/events", 0, &Celex5StereoMTEventsCircle::celexDataCallback_0, this);
    // sleep(0.5); // 让从相机进入的时间一定比主相机第一个大
    data_sub_[1] = node_[1].subscribe(
        "celex5_stereo/1/events", 0, &Celex5StereoMTEventsCircle::celexDataCallback_1, this);// 0 很关键，就是保留所有数据的意思

    struct timeval t0, t1;
    double deltaT;
    gettimeofday(&t0, NULL);//总计时器，超过一定数目，暂停
      
    while(node_[0].ok() && node_[1].ok())
    {
      // std::cout<<1<<std::endl;
      queue_[0].callOne(ros::WallDuration(1.0)); // can also be callAvailable()
      queue_[1].callOne(ros::WallDuration(1.0)); // can also be callAvailable()
      // queue_[0].callAvailable(ros::WallDuration(1.0));
      // queue_[1].callAvailable(ros::WallDuration(1.0));

      gettimeofday(&t1, NULL);//计时器
      deltaT = (t1.tv_sec-t0.tv_sec);//只看s
      if (deltaT > 20)//0s
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
    // cv::destroyWindow(OPENCV_WINDOW_0);
    // cv::destroyWindow(OPENCV_WINDOW_1);
    PlotTrajectory2D();
    master_outfile.close();//关闭文件
    slave_outfile.close();
    pthread_mutex_destroy(&callback_lock);
    pthread_mutex_destroy(&index_lock);

  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  void celexDataCallback_0(const celex5_msgs::EventVector &msg);
  void celexDataCallback_1(const celex5_msgs::EventVector &msg);
  void CircleDetect(const celex5_msgs::EventVector &msg, int device_index);
  void ROICheck(region_of_interest &roi, int datasize);
  void CircleFit(std::vector<EventData> &roiEvent, Circle2D &circle);

  void DistortionCorrection(std::vector<cv::Point2f> &CenterPnt,int index);
  void UV2XYZ(cv::Point2f &uvLeft,cv::Point2f &uvRight, geometry_msgs::PoseStamped &pose);
  void Display(const celex5_msgs::EventVector &msg, int device_index);
  void PlotTrajectory2D();

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

  // 标定函数
  void SetCalibration();
};


/////////////////////////// callback for data_sub_[0]/////////////////////////////
void Celex5StereoMTEventsCircle::celexDataCallback_0(
    const celex5_msgs::EventVector &msg) {
  
  // if (msg.events[0].off_pixel_timestamp == master_timestamp_last)
  // {
  //   return;
  // }
  // std::cout<<"[0]:"<<"celex5's t_increasing:"<<msg.events[0].off_pixel_timestamp<<std::endl;

  // if (msg.events[0].off_pixel_timestamp - master_timestamp_last > 5100)
  // {
  //   std::cout<<"/////////////////////////////////// 0 ERROR! ////////////////////////////////////////////"<<std::endl;
  // }
  // master_timestamp_last = msg.events[0].off_pixel_timestamp;

  // detect
  CircleDetect(msg,0);
  std::cout<<"[0]:"<<"circle's t_increasing:"<<master_circle.time<<","
           <<"x:"<<master_circle.x<<","
           <<"y:"<<master_circle.y<<","
           <<"r:"<<master_circle.r
           <<std::endl;
  // // display the image
  // Display(msg,0);
  // DetectionResultOutput(master_circle);
  master_outfile << master_circle.x <<","<< master_circle.y<<","<<master_circle.r<< std::endl;  //在txt中写入结果
  master_point_save_x.push_back(master_circle.x);
  master_point_save_y.push_back(master_circle.y);
  
  // // pthread_mutex_lock(&callback_lock);
  master_circle_queue.push_back(master_circle); 
  if (master_circle_queue.size() > 200)// && near_index_front>52)
  {
    // save 150, drop first 50
    std::vector<Circle2D> master_circle_queue_tmp;
    master_circle_queue_tmp.assign(master_circle_queue.begin()+51, master_circle_queue.end());
    // std::cout<<"[0]:master_circle_queuee_end:"<<master_circle_queue[master_circle_queue.size()-1].time<<std::endl;
    // // master_circle_queue.clear();
    master_circle_queue = master_circle_queue_tmp;
    // pthread_mutex_lock(&index_lock);
    near_index_front -= 50;
    // pthread_mutex_unlock(&index_lock);
    // std::cout<<"[0]:master_circle_queuee_end*:"<<master_circle_queue[master_circle_queue.size()-1].time<<std::endl;
    // std::cout<<"[0]:master_circle_queuee_size*:"<<master_circle_queue.size()<<std::endl;
    // std::cout<<"[0]:clear,near_index_front:"<<near_index_front<<std::endl;
  }
  // std::cout<<"[0]:"<<"master time:"<<20*master_circle_queue[0].time<<std::endl;
  // std::cout<<"[0]:"<<"master queue size:"<<master_circle_queue.size()<<std::endl;
  // pthread_mutex_unlock(&callback_lock);

}

/////////////////////////// callback for data_sub_[1]/////////////////////////////
void Celex5StereoMTEventsCircle::celexDataCallback_1(
    const celex5_msgs::EventVector &msg) {

  // if (msg.events[0].off_pixel_timestamp == slave_timestamp_last)
  // {
  //   return;
  // }
  // std::cout<<"[1]:"<<"celex5's t_increasing:          "<<msg.events[0].off_pixel_timestamp<<std::endl;

  // if (msg.events[0].off_pixel_timestamp - slave_timestamp_last > 5100)
  // {
  //   std::cout<<"/////////////////////////////////// 1 ERROR! ////////////////////////////////////////////"<<std::endl;
  // }
  // slave_timestamp_last = msg.events[0].off_pixel_timestamp;

  // // detect
  CircleDetect(msg,1);
  std::cout<<"[1]:"<<"circle's t_increasing:"<<slave_circle.time<<","
           <<"x:"<<slave_circle.x<<","
           <<"y:"<<slave_circle.y<<","
           <<"r:"<<slave_circle.r
           <<std::endl;
  // outfile
  // DetectionResultOutput(slave_circle);
  slave_outfile << slave_circle.x <<","<< slave_circle.y<<","<<slave_circle.r<< std::endl;  //在txt中写入结果
  slave_point_save_x.push_back(slave_circle.x);
  slave_point_save_y.push_back(slave_circle.y);

  // Display(msg,1);

  // // 寻找最近时间戳，进行匹配，一定要锁起来，避免多回调出错
  // // pthread_mutex_lock(&index_lock);
  int find_index = near_index_front;
  // // pthread_mutex_unlock(&index_lock);

  int dt;
  // // std::cout<<"[1]:"<<"salve time:"<<20*msg.events[0].off_pixel_timestamp<<std::endl;
  // while(node_[1].ok())
  // {
  //   // lock
  //   // pthread_mutex_lock(&callback_lock);
    
  //   // pthread_mutex_unlock(&callback_lock);
    

  //   if(msg.events[0].off_pixel_timestamp >= master_circle_queue[find_index].time) 
  //   {
  //     dt = msg.events[0].off_pixel_timestamp - master_circle_queue[find_index].time;
  //     if(dt <= event_frame_time)
  //     {
  //       // pthread_mutex_lock(&index_lock);
  //       near_index_front = find_index;
  //       // pthread_mutex_unlock(&index_lock);
  //       near_index_back = find_index+1;

  //       // std::cout<<"find a dt = "<<msg.events[0].off_pixel_timestamp
  //       //          <<" - "
  //       //          <<master_circle_queue[find_index].time
  //       //          <<" = "
  //       //          <<dt
  //       //          <<std::endl;

  //       if (slave_circle.r < 30  ||
  //           slave_circle.r > 200 ||
  //           master_circle_queue[near_index_front].r < 30  ||
  //           master_circle_queue[near_index_front].r > 200 ||
  //           master_circle_queue[near_index_back].r  < 30  ||
  //           master_circle_queue[near_index_back].r  > 200 )
  //       {
  //         break;
  //       }
  //       std::cout<<"match succ!(>>>>>>>>)"<<std::endl;

  //       int master_y_insert = master_circle_queue[near_index_front].y + 
  //                             (master_circle_queue[near_index_back].y - master_circle_queue[near_index_back].y) * (dt/5000);            
  //       int master_x_insert = master_circle_queue[near_index_front].x + 
  //                             (master_circle_queue[near_index_back].x - master_circle_queue[near_index_back].x) * (dt/5000);
  //       // pthread_mutex_unlock(&callback_lock);

  //       // geometry_msgs::PoseStamped circle_poses;
  //       // master_point.x = master_x_insert;
  //       // master_point.y = master_y_insert;
  //       // slave_point.x  = slave_circle.x;
  //       // slave_point.y  = slave_circle.y;
  //       // UV2XYZ(master_point, master_point, circle_poses);
  //       // circle_poses.header.stamp.sec = floor(msg.events[0].off_pixel_timestamp/1e6);
  //       // circle_poses.header.stamp.nsec = (msg.events[0].off_pixel_timestamp%1000000)*1e3;
  //       // pose_pub_.publish(circle_poses);
  //       match_num++;
  //       std::cout<<"///////////////////////////////pub a pose succ! match_num: "
  //                <<match_num<<"/////////////////////////////////////"<<std::endl;
  //       break;
  //     }
  //     else
  //     {
  //       find_index++;
  //       continue;        
  //     }
  //   }//
  //   else
  //   {
  //     dt = master_circle_queue[find_index].time - msg.events[0].off_pixel_timestamp;
  //     if (find_index == 0)
  //     {
  //       break;
  //     }

  //     if(dt <= event_frame_time)
  //     {
  //       near_index_front = find_index-1;
  //       // pthread_mutex_unlock(&index_lock);
  //       near_index_back = find_index;

  //       // std::cout<<"find a dt = "<<msg.events[0].off_pixel_timestamp
  //       //          <<" - "
  //       //          <<master_circle_queue[find_index].time
  //       //          <<" = "
  //       //          <<-dt
  //       //          <<std::endl;

  //       if (slave_circle.r < 30  ||
  //           slave_circle.r > 200 ||
  //           master_circle_queue[near_index_front].r < 30  ||
  //           master_circle_queue[near_index_front].r > 200 ||
  //           master_circle_queue[near_index_back].r  < 30  ||
  //           master_circle_queue[near_index_back].r  > 200 )
  //       {
  //         break;
  //       }
  //       std::cout<<"match succ(<<<<<<<<)!"<<std::endl;

  //       int master_y_insert = master_circle_queue[near_index_front].y + 
  //                             (master_circle_queue[near_index_back].y - master_circle_queue[near_index_back].y) * (dt/5000);            
  //       int master_x_insert = master_circle_queue[near_index_front].x + 
  //                             (master_circle_queue[near_index_back].x - master_circle_queue[near_index_back].x) * (dt/5000);
  //       // pthread_mutex_unlock(&callback_lock);

  //       // geometry_msgs::PoseStamped circle_poses;
  //       // master_point.x = master_x_insert;
  //       // master_point.y = master_y_insert;
  //       // slave_point.x  = slave_circle.x;
  //       // slave_point.y  = slave_circle.y;
  //       // UV2XYZ(master_point, master_point, circle_poses);
  //       // circle_poses.header.stamp.sec = floor(msg.events[0].off_pixel_timestamp/1e6);
  //       // circle_poses.header.stamp.nsec = (msg.events[0].off_pixel_timestamp%1000000)*1e3;
  //       // pose_pub_.publish(circle_poses);
  //       match_num++;
  //       std::cout<<"///////////////////////////////pub a pose succ! match_num: "
  //                <<match_num<<"/////////////////////////////////////"<<std::endl;

  //     }
  //     else
  //     {
  //       find_index--;
  //       continue; 
  //     }  
      
  //   }
    
  // }
}
//////////////////////////// CircleDetect ////////////////////////////////
void Celex5StereoMTEventsCircle::CircleDetect(const celex5_msgs::EventVector &msg, int device_index)
{
  int dataSize = msg.vector_length;
  if (device_index == 0)
  {
    ROICheck(master_roi, msg.vector_length);// 根据数量
    std::vector<EventData> roiEvent;
    for (int i = 0; i < dataSize; i++) 
	  {            
      // ROI
      if (pow((msg.events[i].x-master_roi.x),2) + pow((msg.events[i].y-master_roi.y),2) < pow(master_roi.r,2) )
      {
        EventData Event;
        Event.col = msg.events[i].x;
        Event.row = msg.events[i].y;
        roiEvent.emplace_back(Event);                  //push_back?
      }
    }

    if (roiEvent.size() < 100)
    {
      circle_default.time = msg.events[0].off_pixel_timestamp;
      master_circle = circle_default;
    }
    else
    {
      CircleFit(roiEvent,master_circle);
    }
    master_circle.time = msg.events[0].off_pixel_timestamp;
    master_roi.x = master_circle.x;
    master_roi.y = master_circle.y;
    master_roi.r = 1.5*(master_circle.r+10);
    // master_roi.time = msg.events[0].off_pixel_timestamp;
    
  }
  else
  {
    ROICheck(slave_roi, msg.vector_length);
    std::vector<EventData> roiEvent;
    for (int i = 0; i < dataSize; i++) 
	  {            
      // ROI
      if (pow((msg.events[i].x-slave_roi.x),2) + pow((msg.events[i].y-slave_roi.y),2) < pow(slave_roi.r,2) )
      {
        EventData Event;
        Event.col = msg.events[i].x;
        Event.row = msg.events[i].y;
        roiEvent.emplace_back(Event);
      }
    }

    if (roiEvent.size() < 100)
    {
      circle_default.time = msg.events[0].off_pixel_timestamp;
      slave_circle = circle_default;
    }
    else
    {
      CircleFit(roiEvent,slave_circle);
    }
    slave_circle.time = msg.events[0].off_pixel_timestamp;
    slave_roi.x = slave_circle.x;
    slave_roi.y = slave_circle.y;
    slave_roi.r = 1.5*(slave_circle.r+10);
    // slave_roi.time = msg.events[0].off_pixel_timestamp;
  }
}
//
void Celex5StereoMTEventsCircle::ROICheck(region_of_interest &roi, int datasize)
{
  if(datasize < 4200)
  {
    roi.x = 1000;
    roi.y = 400;
    roi.r = 500;
  }
  else
  {
    if(roi.r < 20)
    {
      roi.r = 700;
    }
    if(roi.x < 0 || roi.x >= MAT_COLS ||
       roi.y < 0 || roi.y >= MAT_ROWS)
    {
      roi.x = 640;
      roi.y = 400;
    }
  }
  
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
  float leftIntrinsic[3][3] = {1718.872858195685,    0,              637.65449,
                              0,             1752.938456154941,      423.48718,
                              0,             0,              1};
  //左相机畸变系数
  float leftDistortion[5] = {0, 0, 0, 0, 0};
  left_Distort_Coefficients.push_back(leftDistortion[0]);
  left_Distort_Coefficients.push_back(leftDistortion[1]);
  left_Distort_Coefficients.push_back(leftDistortion[2]);
  left_Distort_Coefficients.push_back(leftDistortion[3]);
  left_Distort_Coefficients.push_back(leftDistortion[4]);

  //左相机旋转矩阵
  float leftRotation[3][3] = {1,        0,        0, 
                              0,        1,        0, 
                              0,        0,        1};// 平行放置，标定后要改
  //左相机平移向量
  float leftTranslation[1][3] = {100, 0, 0};
  
  //右相机内参数矩阵
  float rightIntrinsic[3][3] = {1718.872858195685,             0,        697.6823113659073,
                                  0,     1752.938456154941,        423.0066761719887,
                                  0,             0,                1};                                   
  //右相机畸变系数
  float rightDistortion[5] = {-0.0835704628721419,	-1.08760521041133, 5.34506579621483, 0.00289692615762865, 0.00815476421350526};
  right_Distort_Coefficients.push_back(rightDistortion[0]);
  right_Distort_Coefficients.push_back(rightDistortion[1]);
  right_Distort_Coefficients.push_back(rightDistortion[2]);
  right_Distort_Coefficients.push_back(rightDistortion[3]);
  right_Distort_Coefficients.push_back(rightDistortion[4]);

  //右相机旋转矩阵
  // float rightRotation[3][3] = {-0.512463107891054,        -0.0920208691438987,        -0.502218873172048, 
  //                        0.597733085511126,        0.100770020678179,        -0.650208873831015, 
  //                        0.0719260622344090,        -0.828601685775270,        0.0604568077110018};
  float rightRotation[3][3] = {1,        0,        0, 
                               0,        1,        0, 
                               0,        0,        1};// 右相机暂时当成世界坐标系
  //右相机平移向量
  float rightTranslation[1][3] = {0, 0, 0};

  mLeftRotation = cv::Mat(3,3,CV_32F,leftRotation);
  mLeftTranslation = cv::Mat(3,1,CV_32F,leftTranslation);
  mLeftRT = cv::Mat(3,4,CV_32F);//左相机RT矩阵
  hconcat(mLeftRotation,mLeftTranslation,mLeftRT);
  mLeftIntrinsic = cv::Mat(3,3,CV_32F,leftIntrinsic);
  mLeftM = mLeftIntrinsic * mLeftRT;
  std::cout<<"左相机M矩阵 = "<<std::endl<<mLeftM<<std::endl;

  mRightRotation = cv::Mat(3,3,CV_32F,rightRotation);
  mRightTranslation = cv::Mat(3,1,CV_32F,rightTranslation);
  mRightRT = cv::Mat(3,4,CV_32F);//右相机RT矩阵
  hconcat(mRightRotation,mRightTranslation,mRightRT);
  mRightIntrinsic = cv::Mat(3,3,CV_32F,rightIntrinsic);
  mRightM = mRightIntrinsic * mRightRT;
  std::cout<<"右相机M矩阵 = "<<std::endl<<mRightM<<std::endl;
}
///////////////////////// 畸变矫正 //////////////////////////
void Celex5StereoMTEventsCircle::DistortionCorrection(vector<cv::Point2f> &CenterPnt,int index)
{
  // if(index == 0)
  // {
  //   cv::undistortPoints(CenterPnt, CenterPnt,
  //   mLeftIntrinsic, left_Distort_Coefficients,
  //   noArray(),mLeftIntrinsic);
  // }
  // else
  // {
  //   cv::undistortPoints(CenterPnt, CenterPnt,
  //   mRightIntrinsic, right_Distort_Coefficients,
  //   noArray(),mRightIntrinsic);
  // }
}//void ParabolaFitting::DistortionCorrection()
///////////////////////// 立体匹配-点 //////////////////////////
//引用了别人的方法，根据左右相机中成像坐标求解空间坐标
//************************************
// 2016/12/2  by 小白
// Method:    uv2xyz
// FullName:  uv2xyz
// Access:    public 
// Returns:   cv::Point3f        世界坐标
// Qualifier: 根据左右相机中成像坐标求解空间坐标
// Parameter: Point2f uvLeft        左相机中成像坐标
// Parameter: Point2f uvRight        右相机中成像坐标
//************************************
void Celex5StereoMTEventsCircle::UV2XYZ(cv::Point2f &uvLeft,cv::Point2f &uvRight, geometry_msgs::PoseStamped &poses)
{
    //  [u1]      |X|                      [u2]      |X|
    //Z*|v1| = Ml*|Y|                    Z*|v2| = Mr*|Y|
    //  [ 1]      |Z|                      [ 1]      |Z|
    //            |1|                                |1|
    //A
    A.at<float>(0,0) = uvLeft.x * mLeftM.at<float>(2,0) - mLeftM.at<float>(0,0);
    A.at<float>(0,1) = uvLeft.x * mLeftM.at<float>(2,1) - mLeftM.at<float>(0,1);
    A.at<float>(0,2) = uvLeft.x * mLeftM.at<float>(2,2) - mLeftM.at<float>(0,2);

    A.at<float>(1,0) = uvLeft.y * mLeftM.at<float>(2,0) - mLeftM.at<float>(1,0);
    A.at<float>(1,1) = uvLeft.y * mLeftM.at<float>(2,1) - mLeftM.at<float>(1,1);
    A.at<float>(1,2) = uvLeft.y * mLeftM.at<float>(2,2) - mLeftM.at<float>(1,2);

    A.at<float>(2,0) = uvRight.x * mRightM.at<float>(2,0) - mRightM.at<float>(0,0);
    A.at<float>(2,1) = uvRight.x * mRightM.at<float>(2,1) - mRightM.at<float>(0,1);
    A.at<float>(2,2) = uvRight.x * mRightM.at<float>(2,2) - mRightM.at<float>(0,2);

    A.at<float>(3,0) = uvRight.y * mRightM.at<float>(2,0) - mRightM.at<float>(1,0);
    A.at<float>(3,1) = uvRight.y * mRightM.at<float>(2,1) - mRightM.at<float>(1,1);
    A.at<float>(3,2) = uvRight.y * mRightM.at<float>(2,2) - mRightM.at<float>(1,2);
    //B
    B.at<float>(0,0) = mLeftM.at<float>(0,3) - uvLeft.x * mLeftM.at<float>(2,3);
    B.at<float>(1,0) = mLeftM.at<float>(1,3) - uvLeft.y * mLeftM.at<float>(2,3);
    B.at<float>(2,0) = mRightM.at<float>(0,3) - uvRight.x * mRightM.at<float>(2,3);
    B.at<float>(3,0) = mRightM.at<float>(1,3) - uvRight.y * mRightM.at<float>(2,3);

    // //采用SVD最小二乘法求解XYZ
    // solve(A,B,XYZ,DECOMP_SVD);

    //求广义逆
    // invert(A, A_p, cv::DECOMP_SVD);
    // XYZ = A_p*B;
    A_p = A.t()*A;
    A_p = A_p.inv()*A.t();
    XYZ = A_p*B;
    
    //cout<<"空间坐标为 = "<<endl<<XYZ<<endl;
    //世界坐标系中坐标
    poses.pose.position.x = XYZ.at<float>(0,0);
    poses.pose.position.y = XYZ.at<float>(1,0);
    poses.pose.position.z = XYZ.at<float>(2,0);
}
void Celex5StereoMTEventsCircle::PlotTrajectory2D()
{
  //plot
  // Set the size of output image =  pixels
  plt::figure_size(1280, 800);
  // Plot line from given x and y data. Color is selected automatically.
  plt::plot(master_point_save_x, master_point_save_y);
  plt::title("Plot-Trajectory-2D");
  plt::xlabel("master_x");
  plt::ylabel("master_y");  
  // plt::show();
  // save figure
  //const char* filename = "./basic" + to_string(mcelex_index) + ".png";
  std::string filename = "./master_point.png";
  std::cout << "Saving result to " << filename << std::endl;
  plt::save(filename);

  plt::figure_size(1280, 800);
  // Plot line from given x and y data. Color is selected automatically.
  plt::plot(slave_point_save_x, slave_point_save_y);
  plt::title("Plot-Trajectory-2D");
  plt::xlabel("slave_x");
  plt::ylabel("slave_y");  
  // plt::show();
  // save figure
  //const char* filename = "./basic" + to_string(mcelex_index) + ".png";
  filename = "./slave_point.png";
  std::cout << "Saving result to " << filename << std::endl;
  plt::save(filename);
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