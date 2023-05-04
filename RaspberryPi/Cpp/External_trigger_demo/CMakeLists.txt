cmake_minimum_required(VERSION 3.18)
project( ArduCam_Ext_Trigger_Demo )

function(PRINT_VAR VARNAME)
  message(STATUS "${VARNAME}: ${${VARNAME}}")
endfunction()

PRINT_VAR("CMAKE_SOURCE_DIR")

#Set Link Folder Variable ArduLib
add_library(ArduLib SHARED IMPORTED)
set_target_properties(ArduLib PROPERTIES IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/Arducam_SDK/libArduCamLib.so")

#Enable C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

#Set Flags for C++.  
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
#pkg_check_modules(GTK REQUIRED gtk+-3.0)
find_package( OpenCV REQUIRED )
PRINT_VAR("OpenCV_INCLUDE_DIRS")
find_package(json-c CONFIG)
PRINT_VAR("json-c_DIR")

#Add source for GTK headers
#include_directories(${GTK_INCLUDE_DIRS})

#Add more files here
add_executable( ArduCam_Ext_Trigger_Demo ArduCam_Ext_Trigger_Demo.cpp ini.c arducam_config_parser.c Utils.cpp )

#Specify the link directory
#link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib ${GTK_LIBRARY_DIRS})

# Add other flags to compiler
#add_definitions(${GTK_CFLAGS_OTHER})

#Include the SDK Lib and shared lib here
target_link_libraries( ArduCam_Ext_Trigger_Demo ${OpenCV_LIBS} pthread ArduLib usb-1.0
	fmt
	stdc++fs
	)

