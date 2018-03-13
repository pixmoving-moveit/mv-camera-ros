cmake_minimum_required(VERSION 2.8.3)
project(mv_camera_ros)

###############################################################################
# PACKAGES

#message("++++++++++++++++++++++++++")
#set(OPENCV_DIRS "" CACHE PATH "")

#if(OPENCV_DIRS STREQUAL "")
#  message(SEND_ERROR "OpenCV path is absent. Please, read the project README.md file.")
#else()
#  message("OpenCV path: " ${OPENCV_DIRS})
#endif()

#find_package( COMPONENTS core highgui imgproc aruco PATHS ${OPENCV_DIRS} REQUIRED NO_DEFAULT_PATH)

find_package(OpenCV REQUIRED)
#if(CUDA_DIRS STREQUAL "")
#  find_package(CUDA REQUIRED)
#else()
#  find_package(CUDA PATHS ${CUDA_DIRS} REQUIRED)
#endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  cv_bridge
  image_geometry
  image_proc
  image_transport
  message_filters
  camera_info_manager
  tf2_ros
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/camera_controls.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    cv_bridge
    image_geometry
    image_proc
    image_transport
    message_filters
    camera_info_manager
    tf2_ros
    dynamic_reconfigure
)

###############################################################################
# INCLUDES & LINKS

set(bitness 32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness 64)
endif()

# Specify locations of header files.
include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

message("++++++++++++++++++++++++++")
message("OpenCV Version " ${OpenCV_VERSION})
message("++++++++++++++++++++++++++")


link_directories(${OpenCV_LIBRARY_DIRS})

#if(bitness EQUAL 64)
#  link_directories(${PYLON_DIRS}/lib64)
#else(bitness EQUAL 64) 
#  link_directories(${PYLON_DIRS}/lib)
#endif(bitness EQUAL 64)


message(${PROJECT_SOURCE_DIR})

link_directories(${PROJECT_SOURCE_DIR}/3rdparty/lib)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/include)

###############################################################################
# EXECUTABLE

# target file name
set(TARGET ${PROJECT_NAME})

# source and object files dirs
set(SRC_DIR src)

# Compile all .c and .cpp files using gcc
file(GLOB CPP_FILES ${SRC_DIR}/*.c ${SRC_DIR}/*.cpp)

# Compile-time definitions
add_definitions(-std=c++11)

# GCC options
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp")

# CUDA options

message(${CMAKE_SYSTEM_PROCESSOR})

#set(CUDA_NVCC_FLAGS_DEBUG -g -G -O0)
#set(CUDA_NVCC_FLAGS_RELEASE -O3)

add_executable(${TARGET}
  ${CPP_FILES}
)

add_definitions(-DNVX)

target_link_libraries(
  ${TARGET}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  -lpthread
  -lMVSDK
)

#add_dependencies(${TARGET} ${PROJECT_NAME}_gencfg)
