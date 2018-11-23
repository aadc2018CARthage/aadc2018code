cmake_minimum_required(VERSION 3.10.0)

project(user_KIObjectDetectionCaffe)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} /opt/cuda/9.2/include /opt/caffe/caffe/include)

set (SOURCEFILES    	stdafx.h
						KIObjectDetectionCaffe.h
						KIObjectDetectionCaffe.cpp)

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

#install dlls for opencv to binary folder
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
endif(WIN32)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS} /opt/caffe/caffe/build/lib/libcaffe.so /opt/boost/1.66.0/lib/libboost_system.so /usr/lib/x86_64-linux-gnu/libglog.so)

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)

