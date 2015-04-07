cmake_minimum_required(VERSION 2.8.3)
find_package(catkin REQUIRED) 
catkin_package()
catkin_python_setup()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(OpenRAVE REQUIRED)
find_package(GLEW REQUIRED)
find_package(X11 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package( PCL REQUIRED )
set(XLibraries Xcursor Xrandr Xinerama Xi Xxf86vm)
pkg_search_module(GLFW REQUIRED glfw3)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    rospy
    sensor_msgs
    openrave_catkin
    cv_bridge
    tf
    image_transport
    pcl_ros
    )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY shaders/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/shaders/
FILES_MATCHING PATTERN "*.glsl"
)

set( CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(SOURCES
    src/offscreen_render/test_cpp.cpp
    src/offscreen_render/ROSCamera.cpp
    src/offscreen_render/Shader.cpp
    src/offscreen_render/VertexBuffer.cpp
    src/offscreen_render/OffscreenRenderer.cpp
    src/offscreen_render/CloudGenerator.cpp
    src/offscreen_render/RaveObjectTracker.cpp
    src/offscreen_render/RaveArmTracker.cpp
    src/offscreen_render/RaveCamera.cpp
    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}  ${OPENRAVE_LIBRARY_DIRS} ${OPENRAVE_LIBRARIES}  ${OpenRAVE_CORE_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES}  ${GLEW_LIBRARIES} ${PCL_LIBRARIES} ${X11_LIBRARIES}  Xcursor Xrandr Xinerama Xi Xxf86vm)

include_directories(${catkin_INCLUDE_DIRS} include ${OPENGL_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${OpenRAVE_INCLUDE_DIRS})
add_executable(test_cpp src/offscreen_render/test_cpp.cpp)
target_link_libraries(test_cpp ${PROJECT_NAME} ${X11_LIBRARIES} ${XLibraries})

openrave_plugin("object_tracker" src/offscreen_render/Plugin.cpp)
target_link_libraries("object_tracker"  ${OPENRAVE_LIBRARY_DIRS} ${OPENRAVE_LIBRARIES}  ${OpenRAVE_CORE_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES}  ${GLEW_LIBRARIES} ${PCL_LIBRARIES} ${X11_LIBRARIES}  Xcursor Xrandr Xinerama Xi Xxf86vm)
target_link_libraries("object_tracker" ${PROJECT_NAME})

openrave_plugin("offscreen_render_camera" src/offscreen_render/RaveCameraPlugin.cpp)
target_link_libraries("offscreen_render_camera"  ${PROJECT_NAME} ${OPENRAVE_LIBRARY_DIRS} ${OPENRAVE_LIBRARIES}  ${OpenRAVE_CORE_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES}  ${GLEW_LIBRARIES} ${PCL_LIBRARIES} ${X11_LIBRARIES} Xcursor Xrandr Xinerama Xi Xxf86vm)



