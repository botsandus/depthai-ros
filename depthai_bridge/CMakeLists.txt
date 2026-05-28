cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(depthai_bridge VERSION 2.10.3 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(opencv_version 4)
find_package(OpenCV ${opencv_version} QUIET COMPONENTS imgproc highgui calib3d)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(opencv_version 3)
  find_package(OpenCV ${opencv_version} REQUIRED COMPONENTS imgproc highgui calib3d)
endif()


message(STATUS "------------------------------------------")
message(STATUS "Depthai Bridge is being built using AMENT.")
message(STATUS "------------------------------------------")

set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  image_transport
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  composition_interfaces
  ffmpeg_image_transport_msgs
)

include_directories(
    include
    ${BUILD_TOOL_INCLUDE_DIRS}

)

file(GLOB LIB_SRC
"src/DisparityConverter.cpp"
"src/ImageConverter.cpp"
"src/ImgDetectionConverter.cpp"
"src/SpatialDetectionConverter.cpp"
"src/ImuConverter.cpp"
"src/TFPublisher.cpp"
"src/TrackedFeaturesConverter.cpp"
"src/TrackDetectionConverter.cpp"
"src/TrackSpatialDetectionConverter.cpp"
)

add_library(${PROJECT_NAME} SHARED ${LIB_SRC})

# ament_target_dependencies was removed in ROS 2 Lyrical / ament_cmake >= 4.x.
# Use target_link_libraries with the modern per-package target conventions:
#   - message/interface packages export ${pkg}_TARGETS
#   - regular ament packages export pkg::pkg targets
foreach(_dep IN LISTS dependencies)
  if(${_dep}_TARGETS)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${${_dep}_TARGETS})
  elseif(TARGET ${_dep}::${_dep})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${_dep}::${_dep})
  elseif(TARGET ${_dep})
    # Some packages export un-namespaced targets
    target_link_libraries(${PROJECT_NAME} PUBLIC ${_dep})
  else()
    # Legacy fallback: use variables that find_package might have set
    if(DEFINED ${_dep}_INCLUDE_DIRS)
      target_include_directories(${PROJECT_NAME} PUBLIC ${${_dep}_INCLUDE_DIRS})
    endif()
    if(DEFINED ${_dep}_LIBRARIES)
      target_link_libraries(${PROJECT_NAME} PUBLIC ${${_dep}_LIBRARIES})
    endif()
  endif()
endforeach()

# Explicitly add tf2 includes - tf2 may not properly export targets in Lyrical
# Try multiple approaches to ensure includes are found
if(TARGET tf2::tf2)
  target_link_libraries(${PROJECT_NAME} PUBLIC tf2::tf2)
elseif(TARGET tf2)
  target_link_libraries(${PROJECT_NAME} PUBLIC tf2)
endif()

# Add tf2 include paths directly - they should exist in ROS 2 install
# This is a workaround for incomplete CMake config in Lyrical
target_include_directories(${PROJECT_NAME} PUBLIC
  ${tf2_INCLUDE_DIRS}
  /opt/ros/$ENV{ROS_DISTRO}/include
)

target_compile_definitions(${PROJECT_NAME} PUBLIC IS_ROS2)
message(STATUS "--------------------------$ENV{ROS_DISTRO} ---")
message(STATUS $ENV{ROS_DISTRO})

if($ENV{ROS_DISTRO} STREQUAL "galactic" OR ($ENV{ROS_DISTRO} STREQUAL "rolling" AND "${ubuntu_version}" STREQUAL "20.04\n"))
  target_compile_definitions(${PROJECT_NAME} PRIVATE IS_GALACTIC)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")
  target_compile_definitions(${PROJECT_NAME} PRIVATE IS_HUMBLE)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE
    depthai::core
    opencv_imgproc
    opencv_highgui
    opencv_calib3d)

ament_export_targets(depthai_bridgeTargets HAS_LIBRARY_TARGET)

install(DIRECTORY include/
        DESTINATION include/
)

install(TARGETS depthai_bridge
      EXPORT depthai_bridgeTargets
              ARCHIVE DESTINATION lib
              LIBRARY DESTINATION lib
              RUNTIME DESTINATION bin
              INCLUDES DESTINATION include

)
install(EXPORT depthai_bridgeTargets
      DESTINATION share/${PROJECT_NAME}/cmake)



install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(depthai_bridge)
ament_export_dependencies(${dependencies})

ament_package()

