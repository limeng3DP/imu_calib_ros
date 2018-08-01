cmake_minimum_required(VERSION 2.8.12)

set(SYS_NAME "imu_calib package")
set(SYS_DESC "calibrate imu sensor to obtain the scale and offset parameters of each axis")
message(STATUS "Cmake Project ${SYS_NAME}: ${SYS_DESC}")
project(imu_calib C CXX)

# Default build is in Release mode
if (NOT CMAKE_BUILD_TYPE AND NOT MSVC)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# guard against in-source builds
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed.")
endif()

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_BUILD_TYPE}")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake-modules/")
## Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 and c++0x support. Please use a different C++ compiler.")
endif()

find_package( OpenMP )
if(OpenMP_FOUND)
    message(STATUS "OpenMP found: Enable OpenMP parallelization")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    add_definitions(-DUSE_OPENMP)
endif()

#set(Ceres_DIR ${PROJECT_SOURCE_DIR}/thirdparty/ceres-solver/build/lib/cmake/Ceres )
find_package(Ceres 1.14 REQUIRED )
if(Ceres_FOUND)
    message(STATUS "Ceres FOUND: ${Ceres_VERSION}")
else()
    message(FATAL_ERROR "Ceres NOT FOUND!")
endif()

find_package(Eigen3  REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 FOUND: ${EIGEN3_VERSION_STRING}")
else()
    message(FATAL_ERROR "Eigen3 NOT FOUND")
endif()

#ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
#  CATKIN_DEPENDS 
#  DEPENDS 
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  )

add_executable(imu_calib_node
 src/imu_calib_node.cpp
 )

target_link_libraries(imu_calib_node ${catkin_LIBRARIES}
                                ${CERES_LIBRARIES})
