/*
* Copyright (c) 2017-2018 CelePixel Technology Co. Ltd. All Rights Reserved
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "celex5_ros.h"
#include "robot_catch.h"

static const std::string OPENCV_WINDOW_0 = "Master";
static const std::string OPENCV_WINDOW_1 = "Slave";

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <signal.h>
#endif

#define FPN_PATH_M    "/home/guoyeye/celex_ws/src/celex5_stereo_multithread/celex5dependencies/stereo/config/FPN_M_2.txt"
#define FPN_PATH_S    "/home/guoyeye/celex_ws/src/celex5_stereo_multithread/celex5dependencies/stereo/config/FPN_S_2.txt"

#include "celex5_ros.h"


class Celex5StereoRos : public CeleX5DataManager {
public:
  std::vector<EventData> vecEvent_;
  cv::Mat event_frame_;
  cv::Mat full_picture_;

  pthread_mutex_t event_frame_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t event_vec_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t full_picture_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t  event_vec_cond;

  CX5SensorDataServer *m_pServer_;
  CeleX5 *celex_;

  CeleX5::emEventPicType type_;

  Celex5StereoRos(CX5SensorDataServer *pServer){
    // pthread_mutex_init(&event_frame_lock,NULL);
    // pthread_mutex_init(&event_vec_lock,NULL);
    pthread_cond_init(&event_vec_cond, NULL);
    m_pServer_ = pServer;
    m_pServer_->registerData(this, CeleX5DataManager::CeleX_Frame_Data);
  }

  ~Celex5StereoRos() {
    pthread_mutex_destroy(&event_frame_lock);
    pthread_mutex_destroy(&event_vec_lock);
    pthread_mutex_destroy(&full_picture_lock);
    pthread_cond_destroy(&event_vec_cond);

    m_pServer_->unregisterData(this, CeleX5DataManager::CeleX_Frame_Data);
    delete celex_;
  }

  std::vector<EventData> getEventVector();
  cv::Mat getEventFrame();
  cv::Mat getFullPicture();
  // overrides the update operation
  virtual void onFrameDataUpdated(CeleX5ProcessedData *pSensorData);
};

// the callback update function
void Celex5StereoRos::onFrameDataUpdated(
    CeleX5ProcessedData *pSensorData) {

  // get event_frame
//   pthread_mutex_lock(&event_frame_lock);
//   event_frame_ = cv::Mat(800, 1280, CV_8UC1, pSensorData->getEventPicBuffer(type_));
//   pthread_mutex_unlock(&event_frame_lock);

  // // get raw event data
  // pthread_mutex_lock(&event_vec_lock);
  // vecEvent_ = pSensorData->getEventDataVector();
  // // while(!get_flag)
  // // {
  //   pthread_cond_wait(&event_vec_cond, &event_vec_lock);
  // // }
  // pthread_mutex_unlock(&event_vec_lock);
  // // get_flag = 0;

  // get full_picture
  pthread_mutex_lock(&full_picture_lock);
  full_picture_ = cv::Mat(800, 1280, CV_8UC1, pSensorData->getFullPicBuffer());
  pthread_mutex_unlock(&full_picture_lock);
  }

cv::Mat Celex5StereoRos::getFullPicture(){
  pthread_mutex_lock(&full_picture_lock);
  auto tmp=this->full_picture_.clone(); //clone为深拷贝
  pthread_mutex_unlock(&full_picture_lock);
  return tmp; //mat有自己的数据管理，拷贝代价低
}

cv::Mat Celex5StereoRos::getEventFrame(){
  pthread_mutex_lock(&event_frame_lock);
  auto tmp=this->event_frame_.clone(); //clone为深拷贝
  pthread_mutex_unlock(&event_frame_lock);
  return tmp; //mat有自己的数据管理，拷贝代价低
}

std::vector<EventData> Celex5StereoRos::getEventVector(){
  pthread_mutex_lock(&event_vec_lock);
  auto tmp=this->vecEvent_; //vector默认是深拷贝
  pthread_mutex_unlock(&event_vec_lock);
  ////////////////////////////
  // getflag = 1;
  pthread_cond_broadcast(&event_vec_cond);
  return tmp; //这里可以尝试用std::move 来减少一次拷贝
}


// std::vector<EventData> events[2];
// cv::Mat event_frame[2];
Celex5StereoRos *celex_ros_event[2];

struct publisher_with_id
{
  ros::NodeHandle node;
  ros::Publisher pub;
  int id;
  int event_frame_time;
  // add a thread time controler
  int thread_duration;
};

// parameters
std::string celex_mode, event_pic_type;
int threshold, clock_rate, event_frame_time;
int thread_duration;

// tmp
typedef struct event_vector_temp
{
  std::vector<EventData> tmp_data;
}event_vector_temp;

event_vector_temp event_vec_tmp[2];
pthread_mutex_t event_tmp_lock = PTHREAD_MUTEX_INITIALIZER;

aubo_robot_namespace::wayPoint_S G_waypoint;

// 路点实时监测
void RobotCatch::RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    aubo_robot_namespace::wayPoint_S waypoint = *wayPointPtr;

    G_waypoint = waypoint;
}

//////////////////////// main /////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "celex_ros");
  ros::NodeHandle node_;


  //////////////////////////////////////////////////////////////////////////////////////
  ServiceInterface *send_robotService = new ServiceInterface();

  int ret = aubo_robot_namespace::InterfaceCallSuccCode;
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;

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
        // return 0;
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
        // return 0;
    }

  send_robotService->robotServiceRegisterRealTimeRoadPointCallback(RobotCatch::RealTimeWaypointCallback, NULL);

  CeleX5 *pCelex_;
  pCelex_ = new CeleX5;
  if (NULL == pCelex_)
    return 0;

  pCelex_->openSensor(CeleX5::CeleX5_MIPI);

  publisher_with_id events_pub_0;
  publisher_with_id event_frame_pub_0;
  publisher_with_id full_picture_pub_0;
  publisher_with_id events_pub_1;
  publisher_with_id event_frame_pub_1;
  publisher_with_id full_picture_pub_1;

  // grab the parameters
  node_.param<std::string>("/celex5_stereo_multithread_callback/celex_mode", celex_mode,"Full_Picture_Mode");
  node_.param<std::string>("/celex5_stereo_multithread_callback/event_pic_type", event_pic_type,"EventBinaryPic");
  node_.param<int>("/celex5_stereo_multithread_callback/threshold", threshold, 160);   // 0-1024
  node_.param<int>("/celex5_stereo_multithread_callback/clock_rate", clock_rate, 100); // 0-100
  node_.param<int>("/celex5_stereo_multithread_callback/event_frame_time", event_frame_time, 10000); // 0-100S
  node_.param<int>("/celex5_stereo_multithread_callback/thread_duration", thread_duration, 4e6); // > 1e6,*4 is better

  events_pub_0.pub = node_.advertise<celex5_msgs::EventVector>("celex5_stereo/0/events", 10);
  events_pub_0.id = 0;
  events_pub_0.node = node_;
  events_pub_0.event_frame_time = event_frame_time;
  events_pub_0.thread_duration = thread_duration;
  
  event_frame_pub_0.pub = node_.advertise<sensor_msgs::Image>("celex5_stereo/0/event_frame", 10);
  event_frame_pub_0.id = 0;
  event_frame_pub_0.node = node_;
  event_frame_pub_0.event_frame_time = event_frame_time;
  event_frame_pub_0.thread_duration = thread_duration;

  full_picture_pub_0.pub = node_.advertise<sensor_msgs::Image>("celex5_stereo/0/full_picture", 10);
  full_picture_pub_0.id = 0;
  full_picture_pub_0.node = node_;
  full_picture_pub_0.event_frame_time = event_frame_time;
  full_picture_pub_0.thread_duration = thread_duration;

  events_pub_1.pub  = node_.advertise<celex5_msgs::EventVector>("celex5_stereo/1/events", 10);
  events_pub_1.id = 1;
  events_pub_1.node = node_;
  events_pub_1.event_frame_time = event_frame_time;
  events_pub_1.thread_duration = thread_duration;

  event_frame_pub_1.pub = node_.advertise<sensor_msgs::Image>("celex5_stereo/1/event_frame", 10);
  event_frame_pub_1.id = 1;
  event_frame_pub_1.node = node_;
  event_frame_pub_1.event_frame_time = event_frame_time;
  event_frame_pub_1.thread_duration = thread_duration;

  full_picture_pub_1.pub = node_.advertise<sensor_msgs::Image>("celex5_stereo/1/full_picture", 10);
  full_picture_pub_1.id = 1;
  full_picture_pub_1.node = node_;
  full_picture_pub_1.event_frame_time = event_frame_time;
  full_picture_pub_1.thread_duration = thread_duration;

  pCelex_->setThreshold(threshold, 0);
  pCelex_->setThreshold(threshold, 1);
  pCelex_->setEventFrameTime(event_frame_time, 0);
  pCelex_->setEventFrameTime(event_frame_time, 1);

  CeleX5::CeleX5Mode mode;
  if (celex_mode == "Event_Address_Only_Mode")
    mode = CeleX5::Event_Address_Only_Mode;
  else if (celex_mode == "Event_Optical_Flow_Mode")
    mode = CeleX5::Event_Optical_Flow_Mode;
  else if (celex_mode == "Event_Intensity_Mode")
    mode = CeleX5::Event_Intensity_Mode;
  else if (celex_mode == "Full_Picture_Mode")
    mode = CeleX5::Full_Picture_Mode;
  else if (celex_mode == "Full_Optical_Flow_S_Mode")
    mode = CeleX5::Full_Optical_Flow_S_Mode;
  else if (celex_mode == "Full_Optical_Flow_M_Mode")
    mode = CeleX5::Full_Optical_Flow_M_Mode;
  pCelex_->setSensorFixedMode(mode, 0);
  pCelex_->setSensorFixedMode(mode, 1);

  CeleX5::emEventPicType type;
  if (event_pic_type == "EventBinaryPic")
    type = CeleX5::EventBinaryPic;
  else if (event_pic_type == "EventAccumulatedPic")
    type = CeleX5::EventAccumulatedPic;
  else if (event_pic_type == "EventGrayPic")
    type = CeleX5::EventGrayPic;
  else if (event_pic_type == "EventCountPic")
    type = CeleX5::EventCountPic;
  else if (event_pic_type == "EventDenoisedBinaryPic")
    type = CeleX5::EventDenoisedBinaryPic;
  else if (event_pic_type == "EventSuperimposedPic")
    type = CeleX5::EventSuperimposedPic;
  else if (event_pic_type == "EventDenoisedCountPic")
    type = CeleX5::EventDenoisedCountPic;

  pCelex_->setFpnFile(FPN_PATH_M, 0);
  pCelex_->setFpnFile(FPN_PATH_S, 1);

  // pCelex_->disableFrameModule(-1);
  pCelex_->disableIMUModule(-1);

  celex_ros_event[0] = new Celex5StereoRos(pCelex_->getSensorDataServer(0));
  celex_ros_event[1] = new Celex5StereoRos(pCelex_->getSensorDataServer(1));

  celex_ros_event[0]->type_ = type;
  celex_ros_event[1]->type_ = type;

  // events[0] = celex_ros_0->vecEvent_;
  // events[1] = celex_ros_1->vecEvent_;
  // event_frame[0] = celex_ros_0->event_frame_;
  // event_frame[1] = celex_ros_1->event_frame_;
  
  pthread_t events_pub_thread_0;
  pthread_t events_pub_thread_1;
  pthread_t frame_pub_thread_0;
  pthread_t frame_pub_thread_1;   

  ///////////////////////// test ////////////////////////////
  EventData tmp_event;
  tmp_event.t_increasing = 0;
  for (int i=0; i<10; i++){
    event_vec_tmp[0].tmp_data.push_back(tmp_event);
    event_vec_tmp[1].tmp_data.push_back(tmp_event);
  }
  ///////////////////////// test ////////////////////////////

  int rc;
  void *publish_event_frame(void *arg);
  void *publish_full_picture(void *arg);
  void *publish_events(void *arg);

  // cout << "main() : 创建主相机事件向量发布线程" << endl;      
  // rc = pthread_create(&events_pub_thread_0, NULL, publish_events, (void *)&events_pub_0);
  // if (rc){
  //     cout << "Error:无法创建主相机事件向量发布线程" << endl;
  //     exit(-1);
  // }

  if (mode == CeleX5::Event_Address_Only_Mode){
    cout << "main() : 创建主相机事件帧发布线程" << endl;      
    rc = pthread_create(&frame_pub_thread_0, NULL, publish_event_frame, (void *)&event_frame_pub_0);
    if (rc){
        cout << "Error:无法创建主相机事件帧发布线程" << endl;
        exit(-1);
    }
  }
  else if (mode == CeleX5::Full_Picture_Mode){
    cout << "main() : 创建主相机灰度图发布线程" << endl;      
    rc = pthread_create(&frame_pub_thread_0, NULL, publish_full_picture, (void *)&full_picture_pub_0);
    if (rc){
        cout << "Error:无法创建主相机灰度图发布线程" << endl;
        exit(-1);
    }
  }
  pthread_detach(frame_pub_thread_0);

  // cout << "main() : 创建从相机事件向量发布线程" << endl;      
  // rc = pthread_create(&events_pub_thread_1, NULL, publish_events, (void *)&events_pub_1);
  // if (rc){
  //     cout << "Error:无法创建主相机事件向量发布线程" << endl;
  //     exit(-1);
  // }

  if (mode == CeleX5::Event_Address_Only_Mode){
    cout << "main() : 创建从相机事件帧发布线程" << endl;      
    rc = pthread_create(&frame_pub_thread_1, NULL, publish_event_frame, (void *)&event_frame_pub_1);
    if (rc){
        cout << "Error:无法创建主相机事件帧发布线程" << endl;
        exit(-1);
    }
  }
  else if (mode == CeleX5::Full_Picture_Mode){
    cout << "main() : 创建从相机灰度图发布线程" << endl;      
    rc = pthread_create(&frame_pub_thread_1, NULL, publish_full_picture, (void *)&full_picture_pub_1);
    if (rc){
        cout << "Error:无法创建从相机灰度图发布线程" << endl;
        exit(-1);
    }
  }
  pthread_detach(frame_pub_thread_1);

  ros::Rate loop_rate(thread_duration/event_frame_time);

  int scanKeyboard();

  int i = 1;
  int flag = 0;

  std::ofstream joint_outfile;   //输出流
  joint_outfile.open("/home/guoyeye/celex_ws/calibration/theta16.txt", ios::app);  
  if(!joint_outfile.is_open ())
  {
      std::cout << "Open file failure" << std::endl;
  }        
  else
  {
      std::cout << "Open file succesed" << std::endl;
  }

	while (node_.ok())
	{
    std::cout << "emmmm" << std::endl;
		// if (mode == CeleX5::Full_Picture_Mode)
		// {
		// 	if (!pCelex_->getFullPicMat().empty())
		// 	{
        

		// 		cv::imshow("0", fullPicMatMaster);
		// 		cv::imshow(“1, fullPicMatSlave);

        // char key_board = cv::waitKey(10);

		  // char key_board = getchar();
		    char key_board = scanKeyboard(); // 线程会卡在i这个地方，直到按键，因此图片就算变了也是上一次的图片，所以应该把获取图片写在这个下面
        std::cout << "key_board" << key_board << std::endl;
			
        if (key_board == 's')
        {
          if (flag == 1)
          {
            std::string FilenameMaster;
            std::string FilenameSlave;
            auto fullPicMatMaster = celex_ros_event[0]->getFullPicture();
            auto fullPicMatSlave = celex_ros_event[1]->getFullPicture();

            if (i<10)
            {
              FilenameMaster =  "/home/guoyeye/celex_ws/calibration/master/00" + to_string(i) + ".jpg";
              FilenameSlave =  "/home/guoyeye/celex_ws/calibration/slave/00" + to_string(i) + ".jpg";
            }            
            else if (i<100)
            {
              FilenameMaster =  "/home/guoyeye/celex_ws/calibration//master/0" + to_string(i) + ".jpg";
              FilenameSlave =  "/home/guoyeye/celex_ws/calibration/slave/0" + to_string(i) + ".jpg";
            }
            else
            {
              FilenameMaster =  "/home/guoyeye/celex_ws/calibration//master/" + to_string(i) + ".jpg";
              FilenameSlave =  "/home/guoyeye/celex_ws/calibration/slave/" + to_string(i) + ".jpg";
            }

            std::cout<<FilenameMaster<<std::endl;
            std::cout<<FilenameSlave<<std::endl;

            // if (!fullPicMatMaster.empty() && !fullPicMatSlave.empty())
            if (!fullPicMatMaster.empty())
            {
              cv::imwrite(FilenameMaster, fullPicMatMaster);
              // cv::imwrite(FilenameSlave, fullPicMatSlave);
            
              aubo_robot_namespace::wayPoint_S waypoint;
              waypoint = G_waypoint;
              joint_outfile << waypoint.jointpos[0]*180/PI << " "
                            << waypoint.jointpos[1]*180/PI << " "
                            << waypoint.jointpos[2]*180/PI << " "
                            << waypoint.jointpos[3]*180/PI << " "
                            << waypoint.jointpos[4]*180/PI << " "
                            << waypoint.jointpos[5]*180/PI << " " 
                            << i << " "
                            << std::endl;  //在txt中写入结果
              std::cout << "成功保存图片和关节： 第" << i << "组" << std::endl;
              i++;
            }
            
          }
          else
          {
            flag = 1;
          }
          
          
        }
        else if (key_board == 'q')
        {
          std::cout << "成功退出" << std::endl;
          break;
        }
        loop_rate.sleep();
		// 	}
		// }
		// else if (mode == CeleX5::Event_Address_Only_Mode)
		// {

		// }
	}

  joint_outfile.close();//关闭文件
  delete pCelex_;
  delete send_robotService;
  return EXIT_SUCCESS;
}

void *publish_event_frame(void *arg){
    struct publisher_with_id *publisher;
    publisher = (struct publisher_with_id *) arg;
    ros::Rate loop_rate(1e6/publisher->event_frame_time);
  while (publisher->node.ok()) {
    auto data=celex_ros_event[publisher->id]->getEventFrame();
    sensor_msgs::ImagePtr event_frame_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", data).toImageMsg();
    // sensor_msgs::ImagePtr event_frame_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", event_frame[publisher->id]).toImageMsg();
    publisher->pub.publish(event_frame_ros);
    // ros::spinOnce();
    loop_rate.sleep();
  }
  pthread_exit(NULL);
}

void *publish_full_picture(void *arg){
    struct publisher_with_id *publisher;
    publisher = (struct publisher_with_id *) arg;
    ros::Rate loop_rate(1e6/publisher->event_frame_time);
  while (publisher->node.ok()) {
    auto data=celex_ros_event[publisher->id]->getFullPicture();
    // if (publisher->id == 0)
    // {
    //   cv::imshow("0", data);
    // }
    // else
    // {
    //   cv::imshow("1", data);
    // }
    // cv::waitKey(10);
    sensor_msgs::ImagePtr full_picture_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", data).toImageMsg();
    // sensor_msgs::ImagePtr event_frame_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", event_frame[publisher->id]).toImageMsg();
    publisher->pub.publish(full_picture_ros);
    // ros::spinOnce();
    loop_rate.sleep();
  }
  pthread_exit(NULL);
}

void *publish_events(void *arg){
  struct publisher_with_id *publisher;
  publisher = (struct publisher_with_id *) arg;
  celex5_msgs::EventVector event_vector;
  ros::Rate loop_rate(publisher->thread_duration/publisher->event_frame_time);
  
  while (publisher->node.ok()) {
      auto data =  celex_ros_event[publisher->id]->getEventVector();
      int dataSize = data.size();
      // ///////////////////////// test ////////////////////////////
      pthread_mutex_lock(&event_tmp_lock);
      // int tmp_dataSize = event_vec_tmp[publisher->id].tmp_data.size();//
      if (dataSize == 0 ||
          data[0].t_increasing == event_vec_tmp[publisher->id].tmp_data[0].t_increasing)
      {
        event_vec_tmp[publisher->id].tmp_data = data;// 不加这句要出问题
        pthread_mutex_unlock(&event_tmp_lock);
        // usleep(100);
        continue;
      }
      // if (data[0].t_increasing - event_vec_tmp[publisher->id].tmp_data[0].t_increasing > 5100)
      // {
      //   std::cout<<"///////////////////////////////////ERROR////////////////////////////////////////////"<<std::endl;
      // }
      event_vec_tmp[publisher->id].tmp_data = data;
      pthread_mutex_unlock(&event_tmp_lock);
      // ///////////////////////// test ////////////////////////////
      event_vector.vector_length = dataSize;
      celex5_msgs::Event event_;
      for (int i = 0; i < dataSize; i++) {
        event_.x = data[i].col;
        event_.y = data[i].row;
        event_.brightness = 255;
        // event_.timestamp.sec = floor(data[i].t_increasing/5e4);
        // event_.timestamp.nsec = data[i].t_increasing%50000*2e4;
        event_.off_pixel_timestamp = 20*data[i].t_increasing;
        event_vector.events.emplace_back(event_);
        // if (floor(data[i].t_increasing/5e4) == tmp && publisher->id == 0)
        // {
        //   std::cout << "master: " << data[i].t_increasing/5e4 << std::endl;
        //   tmp++ ;
        // }
        // if (floor(data[i].t_increasing/5e4) == tmp && publisher->id == 1)
        // {
        //   std::cout << "slave: " << data[i].t_increasing/5e4 << std::endl;
        //   tmp++ ;
        // }
      }
      ///////////////////////// test ////////////////////////////
      // if (dataSize>0)
      // {
        if(publisher->id == 0)
          std::cout<<"["<<publisher->id<<"]:"<<"celex5's t_increasing: "<<20*data[dataSize-1].t_increasing<<",            "
                   <<"dataSize: "<<dataSize
                   <<std::endl;
        else
          std::cout<<"["<<publisher->id<<"]:"<<"celex5's t_increasing:             "<<20*data[dataSize-1].t_increasing<<","
                   <<"dataSize:       "<<dataSize
                   <<std::endl;
      // }
      ///////////////////////// test ////////////////////////////
      publisher->pub.publish(event_vector);
      event_vector.events.clear();

    // loop_rate.sleep();      
    }
    pthread_exit(NULL);
}

int scanKeyboard()
{
int in;
struct termios new_settings;
struct termios stored_settings;
tcgetattr(0,&stored_settings);
new_settings = stored_settings;
new_settings.c_lflag &= (~ICANON);
new_settings.c_cc[VTIME] = 0;
tcgetattr(0,&stored_settings);
new_settings.c_cc[VMIN] = 1;
tcsetattr(0,TCSANOW,&new_settings);
 
in = getchar();
 
tcsetattr(0,TCSANOW,&stored_settings);
return in;
}