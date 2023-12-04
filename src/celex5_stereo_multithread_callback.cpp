#include "celex5_ros.h"

CeleX5 *pCelex_ = new CeleX5;

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
  pthread_mutex_lock(&event_frame_lock);
  event_frame_ = cv::Mat(800, 1280, CV_8UC1, pSensorData->getEventPicBuffer(type_));
  pthread_mutex_unlock(&event_frame_lock);

  // get raw event data
  int currDeviceIndex = pSensorData->getDeviceIndex();
  std::time_t time_stamp;
  pthread_mutex_lock(&event_vec_lock);

  // pCelex_->getEventDataVector(vecEvent_, currDeviceIndex);
  pCelex_->getEventDataVectorEx(vecEvent_, time_stamp, currDeviceIndex);
  // vecEvent_ = pSensorData->getEventDataVector();
  // while(!get_flag)
  // {
    pthread_cond_wait(&event_vec_cond, &event_vec_lock);
  // }
  pthread_mutex_unlock(&event_vec_lock);
  std::cout<<"["<<currDeviceIndex<<"]"<<"time_stamp: "<<time_stamp<<std::endl;
  // get_flag = 0;

  // // get full_picture
  // pthread_mutex_lock(&full_picture_lock);
  // full_picture_ = cv::Mat(800, 1280, CV_8UC1, pSensorData->getFullPicBuffer());
  // pthread_mutex_unlock(&full_picture_lock);
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
double stereo_time_difference;

// tmp
typedef struct event_vector_temp
{
  std::vector<EventData> tmp_data;
}event_vector_temp;

event_vector_temp event_vec_tmp[2];
pthread_mutex_t event_tmp_lock = PTHREAD_MUTEX_INITIALIZER;

long add0;
long add1;
long num0;
long num1;
//////////////////////// main /////////////////////////////////
#ifdef _WIN32
bool exit_handler(DWORD fdwctrltype)
{
	switch (fdwctrltype)
	{
		//ctrl-close: confirm that the user wants to exit.
	case CTRL_CLOSE_EVENT:
	case CTRL_C_EVENT:
	case CTRL_SHUTDOWN_EVENT:
		delete pCelex_;
		pCelex_ = NULL;
		return(true);
	default:
		return false;
	}
}
#else
void exit_handler(int sig_num)
{
	printf("SIGNAL received: num =%d\n", sig_num);
	if (sig_num == 1 || sig_num == 2 || sig_num == 3 || sig_num == 9 || sig_num == 15)
	{
    std::cout<< "T_error: " << 2*double(add0-add1)/(num0+num1)<<std::endl;

		delete pCelex_;
		pCelex_ = NULL;
		// cv::destroyWindow(OPENCV_WINDOW_0);
    // cv::destroyWindow(OPENCV_WINDOW_1);
		exit(0);
	}
}
#endif


int main(int argc, char **argv) {
  ros::init(argc, argv, "celex_ros");
  ros::NodeHandle node_;
  // CeleX5 *pCelex_;
  // pCelex_ = new CeleX5;
  
  if (NULL == pCelex_)
    return 0;
  pCelex_->openSensor(CeleX5::CeleX5_MIPI);

  add0 = 0;
  add1 = 0;
  num0 = 0;
  num1 = 0;

  // while(1)
  // {
  //   if (pCelex_->isSensorReady(1) )
  //   {
  //     std::cout << pCelex_->isSensorReady(1) << std::endl;
  //     break;
  //   }
  //   std::cout << pCelex_->isSensorReady(1) << std::endl;
    
  // }
  pCelex_->reset(0); 
  pCelex_->reset(1); 

  publisher_with_id events_pub_0;
  publisher_with_id event_frame_pub_0;
  publisher_with_id full_picture_pub_0;
  publisher_with_id events_pub_1;
  publisher_with_id event_frame_pub_1;
  publisher_with_id full_picture_pub_1;

  // grab the parameters
  node_.param<std::string>("/celex5_stereo_multithread_callback/celex_mode", celex_mode,"Event_Address_Only_Mode");
  node_.param<std::string>("/celex5_stereo_multithread_callback/event_pic_type", event_pic_type,"EventBinaryPic");
  node_.param<int>("/celex5_stereo_multithread_callback/threshold", threshold, 160);   // 0-1024
  node_.param<int>("/celex5_stereo_multithread_callback/clock_rate", clock_rate, 100); // 0-100
  node_.param<int>("/celex5_stereo_multithread_callback/event_frame_time", event_frame_time, 10000); // 0-100S
  node_.param<int>("/celex5_stereo_multithread_callback/thread_duration", thread_duration, 4e6); // > 1e6,*4 is better
  node_.param<double>("/celex5_stereo_multithread_callback/stereo_time_difference", stereo_time_difference, 0); 
  

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

  pCelex_->setFpnFile("../src/celex5_stereo/celex5dependencies/stereo/config/FPN_M_2.txt", 0);
  pCelex_->setFpnFile("../src/celex5_stereo/celex5dependencies/stereo/config/FPN_S_2.txt", 1);

  // pCelex_->disableFrameModule(-1);
  // pCelex_->disableIMUModule(-1);
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
  
#ifdef _WIN32
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)exit_handler, true);
#else
	// install signal use sigaction
	struct sigaction sig_action;
	sigemptyset(&sig_action.sa_mask);
	sig_action.sa_flags = 0;
	sig_action.sa_handler = exit_handler;
	sigaction(SIGHUP, &sig_action, NULL);  // 1
	sigaction(SIGINT, &sig_action, NULL);  // 2
	sigaction(SIGQUIT, &sig_action, NULL); // 3
	sigaction(SIGKILL, &sig_action, NULL); // 9
	sigaction(SIGTERM, &sig_action, NULL); // 15
#endif

  int rc;
  void *publish_event_frame(void *arg);
  void *publish_full_picture(void *arg);
  void *publish_events(void *arg);

  cout << "main() : 创建主相机事件向量发布线程" << endl;      
  rc = pthread_create(&events_pub_thread_0, NULL, publish_events, (void *)&events_pub_0);
  if (rc){
      cout << "Error:无法创建主相机事件向量发布线程" << endl;
      exit(-1);
  }

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

  cout << "main() : 创建从相机事件向量发布线程" << endl;      
  rc = pthread_create(&events_pub_thread_1, NULL, publish_events, (void *)&events_pub_1);
  if (rc){
      cout << "Error:无法创建主相机事件向量发布线程" << endl;
      exit(-1);
  }

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

  ros::Rate loop_rate(thread_duration/event_frame_time);
  while (node_.ok()) {
    // events[0] = celex_ros_0->vecEvent_;
    // events[1] = celex_ros_1->vecEvent_;
    // event_frame[0] = celex_ros_0->event_frame_;
    // event_frame[1] = celex_ros_1->event_frame_;
    // ros::spinOnce();
    loop_rate.sleep(); //理论上主线程可以退出了，具体得看ros那边
  }
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
      publisher->node.getParam("/celex5_stereo_multithread_callback/stereo_time_difference", stereo_time_difference);
      std::cout<<"stereo_time_difference:"<<stereo_time_difference<<std::endl;
      for (int i = 0; i < dataSize; i++) {
        event_.x = data[i].col;
        event_.y = data[i].row;
        event_.brightness = 255;
        // event_.timestamp.sec = floor(data[i].t_increasing/5e4);
        // event_.timestamp.nsec = data[i].t_increasing%50000*2e4;      
        if (publisher->id == 0)
          event_.off_pixel_timestamp = 20*data[i].t_increasing;
        else
          event_.off_pixel_timestamp = uint64_t(double(20*data[i].t_increasing) + stereo_time_difference);

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
        {
          std::cout<<"["<<publisher->id<<"]:"<<"celex5's t_increasing: "<<20*data[dataSize-1].t_increasing<<",            "
                   <<"dataSize: "<<dataSize
                   <<std::endl;
          if(20*data[dataSize-1].t_increasing > 1e7)
          {
            add0 += 20*data[dataSize-1].t_increasing;
            num0++;
          }          
        }  
        else
        {
          std::cout<<"["<<publisher->id<<"]:"<<"celex5's t_increasing:             "<<20*data[dataSize-1].t_increasing<<","
                   <<"dataSize:       "<<dataSize
                   <<std::endl;
          if(20*data[dataSize-1].t_increasing > 1e7)
          {
            add1 += 20*data[dataSize-1].t_increasing;
            num1++;
          }
        }
          
      // }
      ///////////////////////// test ////////////////////////////
      publisher->pub.publish(event_vector);
      event_vector.events.clear();

    // loop_rate.sleep();      
    }
    pthread_exit(NULL);
}
