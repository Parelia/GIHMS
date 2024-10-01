cmake_minimum_required(VERSION 3.0.2)
project(gihms_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")


# yaml-cpp
include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()


set(ROS_DEPENDS
  roscpp
  std_msgs
  message_generation
  cmake_modules
  sensor_msgs
  cv_bridge
  moveit_ros_planning_interface
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5PrintSupport
  Qt5Core
  #Qt5Gui
  #Qt5Qml
  #Qt5Quick
  TinyXML2
  OpenCV
  Eigen3
  #yaml-cpp
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  #Qt5::Gui
  #Qt5::Qml
  #Qt5::Quick
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
  )

## need this for qt auto generated headers to include!
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###
###  Add Non ROS dependencies
###
foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

###
###  Add any required ros packages here
###
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msgOpenCV
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(OpenCV
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)OpenCV
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS ${ROS_DEPENDS}
   DEPENDS ${EXTERNAL_DEPENDS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  #${yaml-cpp_INCLUDE_DIRS}
)

###Eigen/Geometry>
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files


## Include source files
file(GLOB SOURCES "src/*.cpp" "src/*.hpp")

## Include header files
file(GLOB HEADERS "include/*.h")

## Include source files
file(GLOB NODES "nodes/*.cpp")

## Include ui files -> inutile
file(GLOB_RECURSE UI_FILES "include/*.ui")
#make them into headers
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# find all resource files
file(GLOB QRC
    "rc/*.qrc"
)

###
### compile
###

foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)

  add_executable(${NODE_NAME} ${NODE_FILE} ${HEADERS} ${SOURCES} ${UI_FILES} ${QRC})
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB} ${TinyXML2_LIBRARIES} ${OpenCV_LIBRARIES}) #${YAML_CPP_LIBRARIES}
  target_include_directories(${NODE_NAME} PRIVATE "include/")
# For install
  list(APPEND NODE_LIST ${NODE_NAME})

endforeach(NODE_FILE)


## Declare a C++ library
# add_library(${PROJECT_NAME}_lib ${SOURCES} ${HEADERS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node ${EXECUTABLE} ${PROJECT_NAME})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_node
 # ${catkin_LIBRARIES}
 # ${PROJECT_NAME}_lib
 # ${EXTERNAL_DEPENDS_LIB}
#)

#target_link_libraries(${PROJECT_NAME}_lib
#  ${EXTERNAL_DEPENDS_LIB}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Template
#install(TARGETS ${NODE_LIST}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY launch/
 # DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 # )

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test test/test_example.cpp)
#if(TARGET ${PROJECT_NAME}-test)
 #  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
