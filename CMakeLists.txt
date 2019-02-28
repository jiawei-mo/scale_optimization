cmake_minimum_required(VERSION 2.4.6)
project(dsvo_dso)

set(BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
)


set(DSO_PATH         /home/jiawei/Workspace/dso)

# set values from environment variables
if (NOT DSO_PATH)
    set(DSO_PATH $ENV{DSO_PATH})
endif()
if (NOT DSO_LIBRARY_PATH)
    set(DSO_LIBRARY_PATH $ENV{DSO_LIBRARY_PATH})
    if (NOT DSO_LIBRARY_PATH AND DSO_PATH)
        set(DSO_LIBRARY_PATH "${DSO_PATH}/build/lib")
    endif()
endif()

if (DSO_PATH)
    # Manually set path to DSO source folder
    message("DSO_PATH set, trying to find library.")
    message("---- EXPECTING DSO sources at\n\"${DSO_PATH}\"")
    set(DSO_INCLUDE_DIRS "${DSO_PATH}/src" "${DSO_PATH}/thirdparty/Sophus")
    message("---- LOOKING FOR DSO library at\n\"${DSO_LIBRARY_PATH}\"")
    find_library(DSO_LIBRARY dso ${DSO_LIBRARY_PATH})
else()
    # Detect DSO install (also catkin devel folder)
    message("DSO_PATH not set yet, trying to find installed dso headers and library.")
    find_path(DSO_INCLUDE_DIRS dso)
    if (DSO_INCLUDE_DIRS)
        set(DSO_INCLUDE_DIRS "${DSO_INCLUDE_DIRS}/dso")
        message("---- FOUND DSO headers at \"${DSO_INCLUDE_DIRS}\"")
    endif()
    find_library(DSO_LIBRARY dso)
endif()


if (NOT DSO_INCLUDE_DIRS)
    message(FATAL_ERROR "DSO headers not found and/or DSO_PATH not set.")
endif()
if (NOT DSO_LIBRARY)
    message(FATAL_ERROR "DSO library not found and/or DSO_LIBRARY_PATH not set.")
endif()

message("---- Found DSO library at \"${DSO_LIBRARY}\"")



find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
)

###########
## Build ##
###########

# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
#   "${SSE_FLAGS} -O3 -g -std=c++0x -fno-omit-frame-pointer"
)

if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

# Sources files
set(DSVOSystem_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DSVOSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/dso_helpers/DSVOSystemDebugStuff.cpp
  ${PROJECT_SOURCE_DIR}/src/dso_helpers/DSVOSystemMarginalize.cpp
  ${PROJECT_SOURCE_DIR}/src/dso_helpers/DSVOSystemOptimize.cpp
  ${PROJECT_SOURCE_DIR}/src/dso_helpers/DSVOSystemOptPoint.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(dsvo_node src/dsvo_node.cpp ${DSVOSystem_SOURCE_FILES} src/ScaleOptimizer.cpp)

target_link_libraries(dsvo_node
    ${DSO_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})
