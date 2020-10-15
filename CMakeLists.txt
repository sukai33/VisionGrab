cmake_minimum_required(VERSION 3.14)
project(VisionGrab)

set(CMAKE_CXX_STANDARD 14)

set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(PREBUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-return-type")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")

# 配置依赖目录及程序输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}/bin" CACHE PATH "Runtime directory" FORCE)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}/lib" CACHE PATH "Archive directory" FORCE)

# 给当前cmake的查找路径添加一个额外的目录
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# OpenCV
find_package(OpenCV REQUIRED)
# Kinect2 - freenect2
find_package(freenect2 REQUIRED)
# Aubo
find_package(AuBo REQUIRED)
# PCL
find_package(PCL REQUIRED)
# Astra
find_package(Astra REQUIRED) # FindXXX.cmake XXXConfig.cmake

# include dir
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${freenect2_INCLUDE_DIRS})
include_directories(${AuBo_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
include_directories(${PREBUILD_DIR}/astra/common)
include_directories(${ASTRA_INCLUDE_DIRS})

# library dir
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${AuBo_LINK_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${ASTRA_LINK_DIRS})

# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})

#线程, Aubot机械臂用
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lpthread -Wl,--no-as-needed  -g3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

add_subdirectory(src)
add_subdirectory(src/camera)
add_subdirectory(src/util)
add_subdirectory(src/robot)
add_subdirectory(src/jaws)

add_executable(VisionGrab main.cpp)