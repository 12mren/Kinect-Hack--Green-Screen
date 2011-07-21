cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Kinect_Viewer)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}../../)
set(OpenCV_DIR /panfs/panasas1/research/gerlebacher/ndc08/OpenCV-2.2.0/)
find_package( PCL 1.0 REQUIRED)
find_package( OpenGL REQUIRED )
find_package( GLUT REQUIRED )
find_package( OpenCV REQUIRED )
add_definitions(-g)

include_directories(
                    ${PCL_INCLUDE_DIRS}
                    ${GLUT_INCLUDE_DIR}
                    ${OPENGL_INCLUDE_DIR}
                    ${OPENCV_INCLUDE_DIR}
                    )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kinect_benchmark kinect_benchmark.cpp)
add_executable (kinect_viz kinect_viz.cpp )
add_executable (cyl_seg cyl_seg.cpp)
add_executable (read_pcd read_pcd.cpp)
add_executable (pcd_normals pcd_normals.cpp)
add_executable (change_detection change_detection.cpp)

add_executable (cool_hack3 cool_hack3.cpp timer_eb.cpp)
add_executable (test test.cpp)
#add_executable (cyl_seg_kinect cyl_seg_kinect.cpp)

set( LIBRARIES ${PCL_LIBRARIES} 
               ${OPENGL_LIBRARIES} 
               ${GLUT_LIBRARY} 
               ${OpenCV_LIBS}
               )

target_link_libraries (kinect_benchmark ${LIBRARIES})
target_link_libraries (kinect_viz ${LIBRARIES} )
target_link_libraries (cyl_seg ${LIBRARIES})
target_link_libraries (read_pcd ${LIBRARIES})
target_link_libraries (pcd_normals ${LIBRARIES})
target_link_libraries (change_detection ${LIBRARIES})
target_link_libraries (test ${LIBRARIES})
target_link_libraries (cool_hack3 ${LIBRARIES})
# target_link_libraries (cyl_seg_kinect ${LIBRARIES})


