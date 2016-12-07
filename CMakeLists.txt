cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project (bpp_projectv3)


if(CMAKE_CONFIGURATION_TYPES)
	message("Multi-configuration generator")
	set(CMAKE_CONFIGURATION_TYPES "Debug;Release" 
	CACHE STRING "My multi config types" FORCE)
else()
	message("Single-configuration generator")
endif()	

add_definitions(-D_CRT_SECURE_NO_WARNINGS)	

#set (COMPILE_DEFINITIONS_Debug _CRT_SECURE_NO_WARNINGS)
#set (COMPILE_DEFINITIONS_Release _CRT_SECURE_NO_WARNINGS)
 
find_package (PCL 1.7.2 REQUIRED)

SET(Kinect_DIR "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409")
SET(OpenCV_DIR "C:/opencv249/build")
find_package( OpenCV REQUIRED )

include_directories (${PCL_INCLUDE_DIRS} ${Kinect_DIR}/inc)
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

find_package (VTK REQUIRED)
include(${VTK_USE_FILE})

file(GLOB project_c *.c)
file(GLOB project_SOURCES *.cpp)
file(GLOB project_HEADERS *.h)
file(GLOB project_FORMS *.ui)

set   (VTK_LIBRARIES)

QT5_WRAP_CPP (project_HEADERS_MOC MainUI.h)
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (bpp_projectv3 
${project_c} 
${project_SOURCES} 
${project_HEADERS}
${project_FORMS_HEADERS}
${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (bpp_projectv3 ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${Kinect_DIR}/Lib/x64/Kinect20.lib)
qt5_use_modules (bpp_projectv3 Widgets)