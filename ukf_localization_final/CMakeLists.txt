cmake_minimum_required(VERSION 3.8)
project(localization)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()

  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -gdwarf-4") # useful for callgrind
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package (Eigen3 3.3 REQUIRED)

# Localization node
set(localization_node_srcs
    src/tools.cpp
    src/particle_filter.cpp
    src/ukf.cpp
    src/scan.cpp
    src/object_detector.cpp
    src/localization_node.cpp
)
set(localization_node_hdrs
    include/localization/types.hpp
    include/localization/tools.hpp
    include/localization/sensor.hpp
    include/localization/particle_filter.hpp
    include/localization/ukf.hpp
    include/localization/scan.hpp
    include/localization/object_detector.hpp
    include/localization/localization_node.hpp
)

add_executable(localization_node ${localization_node_srcs} ${localization_node_hdrs})
ament_target_dependencies(localization_node
    "rclcpp" "tf2" "tf2_ros" "std_msgs" "geometry_msgs" "sensor_msgs" "nav_msgs" "visualization_msgs"
)
target_include_directories(localization_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/EigenRand>
        $<INSTALL_INTERFACE:include>)
target_compile_features(localization_node PUBLIC cxx_std_17)

add_subdirectory(libs/EigenRand)
target_link_libraries(localization_node Eigen3::Eigen)

install(TARGETS localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
