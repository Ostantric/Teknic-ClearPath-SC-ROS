cmake_minimum_required(VERSION 3.1.0)
project(clearpath)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
)
find_package(Threads REQUIRED) # cmake 2.*.* might need it
#generate_messages(DEPENDENCIES std_msgs)
add_compile_options(-std=c++11) #supervisor threads
catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
 sFoundationPkg/inc/
 sFoundationPkg/inc/inc-pub/
 sFoundationPkg/inc/inc-private/linux/
 sFoundationPkg/inc/inc-private/sFound/
 sFoundationPkg/LibINI/inc/
 sFoundationPkg/LibLinuxOS/inc/
 sFoundationPkg/LibXML/inc/
 sFoundationPkg/inc/inc-private/
)

add_library(sFoundation
sFoundationPkg/sFoundation/src-linux/SerialLinux.cpp
sFoundationPkg/sFoundation/src-linux/lnkAccessLinux.cpp
sFoundationPkg/sFoundation/src/SerialEx.cpp
sFoundationPkg/sFoundation/src/converterLib.cpp
sFoundationPkg/sFoundation/src/cpmAPI.cpp
sFoundationPkg/sFoundation/src/cpmClassImpl.cpp
sFoundationPkg/sFoundation/src/iscAPI.cpp
sFoundationPkg/sFoundation/src/lnkAccessCommon.cpp
sFoundationPkg/sFoundation/src/meridianNet.cpp
sFoundationPkg/sFoundation/src/netCmdAPI.cpp
sFoundationPkg/sFoundation/src/netCoreFmt.cpp
sFoundationPkg/sFoundation/src/sysClassImpl.cpp
sFoundationPkg/LibXML/src/ErrCodeStr.cpp
sFoundationPkg/LibXML/src/pugixml.cpp
sFoundationPkg/LibLinuxOS/src/tekEventsLinux.cpp
sFoundationPkg/LibLinuxOS/src/tekThreadsLinux.cpp
sFoundationPkg/LibLinuxOS/src/version.cpp
sFoundationPkg/LibINI/src/dictionary.cpp
sFoundationPkg/LibINI/src/iniparser.cpp
)

add_executable(pub_example_node src/ROS_Examples/Publisher/Pub.cpp)

target_link_libraries(pub_example_node 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(sub_example_node src/ROS_Examples/Subscriber/Sub.cpp)
target_link_libraries(sub_example_node 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(teleop_example_node src/ROS_Examples/Joystick/teleop.cpp)
target_link_libraries(teleop_example_node 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_GPIO_Example src/SDK_Examples/GPIO/GPIO.cpp)
target_link_libraries(SDK_GPIO_Example 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_HelloWorld_Example src/SDK_Examples/HelloWorld/HelloWorld.cpp)
target_link_libraries(SDK_HelloWorld_Example 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_Homing_Example src/SDK_Examples/Homing/Homing.cpp)
target_link_libraries(SDK_Homing_Example 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)
add_executable(SDK_Motion_Example src/SDK_Examples/Motion/Motion.cpp)
target_link_libraries(SDK_Motion_Example 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)


add_executable(SDK_SingleThread_Example src/SDK_Examples/SingleThread/Axis.cpp src/SDK_Examples/SingleThread/SingleThreaded.cpp)
target_link_libraries(SDK_SingleThread_Example 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_MultiThread_Example src/SDK_Examples/MultiThread/Axis.cpp src/SDK_Examples/MultiThread/Supervisor.cpp src/SDK_Examples/MultiThread/MultiThreaded.cpp)
target_link_libraries(SDK_MultiThread_Example 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_StatusAlert_Example src/SDK_Examples/StatusAlerts/StatusAlerts.cpp)
target_link_libraries(SDK_StatusAlert_Example 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)
#add_dependencies(clearpath_node clearpath_generate_messages_cpp)
