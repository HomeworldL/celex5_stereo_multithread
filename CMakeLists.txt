# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.0.2)
project(celex5_stereo_multithread)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  sensor_msgs
  celex5_msgs
  geometry_msgs
  cv_bridge)
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES celex5_stereo
)

set(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "-O3 -fopenmp -std=c++11 ${CMAKE_CXX_FLAGS}")

SET(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
include
${catkin_INCLUDE_DIRS}
)
include_directories(include ${OpenCV_INCLUDE_DIRS})

include_directories(${PYTHON_INCLUDE_DIRS})


set(CeleX ./celex5dependencies/stereo)
set(CeleX_LIB CeleX)
set(CeleX_Driver_LIB CeleDriver)

set(Aubo ./auboi5dependencies)
set(Aubo_Lib auborobotcontroller)
set(Aubo_Lib ${Aubo_Lib} config)
set(Aubo_Lib ${Aubo_Lib} log4cplus)
set(Aubo_Lib ${Aubo_Lib} protobuf.so.9)
# set(Aubo_Lib ${Aubo_Lib} libour_alg_i5p.a)

include_directories(${CeleX}/include)
link_directories(${CeleX}/lib/Linux/x64)

# include_directories(${Aubo}/libconfig/linux_x64/inc)
# include_directories(${Aubo}/log4cplus/linux_x64/inc)
# include_directories(${Aubo}/protobuf/linux_x64/google/protobuf)
# include_directories(${Aubo}/robotController/Header)

include_directories(${Aubo}/robotSDK/inc)

link_directories(${Aubo}/robotController/linux_x64/libour_alg_i5p.a
                 ${Aubo}/log4cplus/linux_x64/lib
                 ${Aubo}/libconfig/linux_x64/lib
                 ${Aubo}/protobuf/linux_x64/lib
                 ${Aubo}/protobuf/linux-x64/lib/libprotobuf.a
                 ${Aubo}/robotSDK/lib/linux_x64
                 ${catkin_LIB_DIRS})


add_executable(${PROJECT_NAME}_callback src/celex5_stereo_multithread_callback.cpp)
target_link_libraries(${PROJECT_NAME}_callback
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${CeleX_LIB}
  ${CeleX_Driver_LIB}
)
add_executable(${PROJECT_NAME}_event_circle src/celex5_stereo_multithread_event_circle.cpp)
target_include_directories(ball_catch_ev PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_event_circle
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${CeleX_LIB}
  ${CeleX_Driver_LIB}
  ${PYTHON_LIBRARIES}
)
add_executable(robot_catch src/robot_catch.cpp)
target_link_libraries(robot_catch
  ${catkin_LIBRARIES}
  auborobotcontroller config log4cplus
)
add_executable(catch_prediction src/catch_prediction.cpp)
target_include_directories(ball_catch_ev PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(catch_prediction
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

add_executable(${PROJECT_NAME}_calibration src/celex5_stereo_multithread_calibration.cpp)
target_link_libraries(${PROJECT_NAME}_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${CeleX_LIB}
  ${CeleX_Driver_LIB}
  auborobotcontroller config log4cplus
)

add_executable(kalman_test src/kalman.cpp)
target_link_libraries(kalman_test
  ${catkin_LIBRARIES}
)

add_executable(visual_servoing src/visual_servoing.cpp)
target_include_directories(visual_servoing PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(visual_servoing
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  auborobotcontroller config log4cplus
)