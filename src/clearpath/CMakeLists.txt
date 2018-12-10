cmake_minimum_required(VERSION 3.1.0)
project(clearpath)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
)
#find_package(Threads REQUIRED) # cmake 2.*.* might need it
#generate_messages(DEPENDENCIES std_msgs)

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

add_executable(clearpath_node src/example.cpp)

target_link_libraries(clearpath_node 
    sFoundation
    ${catkin_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)
#add_dependencies(clearpath_node clearpath_generate_messages_cpp)
