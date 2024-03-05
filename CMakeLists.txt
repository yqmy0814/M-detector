cmake_minimum_required(VERSION 3.1.0)
project(m_detector)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-9")

ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -ltbb" ) 
# set( CMAKE_CXX_FLAGS "-std=c++17 -O3" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ltbb -pthread -std=c++1z -std=c++17 -fexceptions")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++1z -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 11)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=12)
    message("core for MP:  12")
  elseif(N GREATER 5)
  # if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# set(TBB_DIR "/usr/local/tbb-2019_U8")
# include("/home/huajie/package/oneTBB-2019_U8/cmake/TBBBuild.cmake")
# tbb_build(TBB_ROOT "/home/huajie/package/oneTBB-2019_U8" CONFIG_DIR TBB_DIR)

# TBB
set(TBB_ROOT_DIR ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8)
set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)

tbb_build(TBB_ROOT ${TBB_ROOT_DIR}
  compiler=gcc-9
  stdver=c++17
  ${TBB_BUILD_DIR}
  ${TBB_BUILD_PREFIX}
  CONFIG_DIR
  TBB_DIR)

find_package(TBB REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/include)
link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
# find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)


add_executable(dynfilter src/dynfilter_with_odom.cpp src/dynamic_object_filter.cpp src/dynamic_object_cluster.cpp)
target_link_libraries(dynfilter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
target_include_directories(dynfilter PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(display_prediction src/display_prediction.cpp )
target_link_libraries(display_prediction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
target_include_directories(display_prediction PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(cal_recall src/cal_recall_multi.cpp)
target_link_libraries(cal_recall ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
target_include_directories(cal_recall PRIVATE ${PYTHON_INCLUDE_DIRS})


