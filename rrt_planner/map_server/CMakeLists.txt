cmake_minimum_required(VERSION 3.0.2)
project(map_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

find_package(yaml-cpp 0.6 REQUIRED)  
set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES map_server
    CATKIN_DEPENDS
        nav_msgs
        roscpp
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(mapping
    src/mapping.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/map_server.cpp)

target_link_libraries(mapping
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAMLCPP_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mapping
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
