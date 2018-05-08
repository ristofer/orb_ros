cmake_minimum_required(VERSION 2.8.3)
project(orb_slam_ros)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -mstackrealign -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -mstackrealign -march=native ")
LIST(APPEND CMAKE_MODULE_PATH /home/nao/ORB_SLAM2/cmake_modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  image_transport
  cv_bridge
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV 2.4.8 )
#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3 QUIET)
#  if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#   endif()
#endif()

add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
find_package(Boost REQUIRED COMPONENTS system)
#find_package(Eigen3 REQUIRED)
#find_package(Eigen REQUIRED)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pocketsphinx_ros
  CATKIN_DEPENDS roscpp std_msgs  
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
include
/home/robotica/alt_orb/ORB_SLAM2/include
/home/robotica/alt_orb/ORB_SLAM2
/home/robotica/eigen
#${EIGEN3_LIBS}
#${EIGEN_LIBS}
#${EIGEN_INCLUDE_DIRS}
)
#include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIR} )
#${EIGEN3_INCLUDE_DIRS})
set(LIBS 
${OpenCV_LIBS} 
#${EIGEN3_LIBS}
/home/robotica/alt_orb/ORB_SLAM2
/home/robotica/alt_orb/ORB_SLAM2/include
/home/robotica/alt_orb/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
/home/robotica/alt_orb/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
/home/robotica/alt_orb/ORB_SLAM2/lib/libORB_SLAM2.so
/home/robotica/eigen
)


add_executable(maqui src/main.cpp src/SubscribeHandler.cpp)
add_dependencies(maqui ${catkin_EXPORTED_TARGETS})
target_link_libraries(maqui ${catkin_LIBRARIES}  ${LIBS} ${Boost_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
