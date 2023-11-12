cmake_minimum_required(VERSION 3.8)
project(imu_driver)

#Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# For publisher
add_executable(talker src/publisher_member_function.cpp)

# add_executable(listener src/subscriber_member_function.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)

# Add the directory containing the AHRS.h header file to the include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/navx_lib/)

# Add the source files containing the implementation of the AHRS class to the project
add_library(navx_lib
  src/navx_lib/AHRS.cpp
  src/navx_lib/AHRS.h
  src/navx_lib/AHRSProtocol.h
  src/navx_lib/ITimestampedDataSubscriber.h
  src/navx_lib/IMUProtocol.h
  src/navx_lib/IMURegisters.h
  src/navx_lib/ContinuousAngleTracker.cpp
  src/navx_lib/ContinuousAngleTracker.h
  src/navx_lib/IBoardCapabilities.h
  src/navx_lib/IIOCompleteNotification.h
  src/navx_lib/IIOProvider.h
  src/navx_lib/InertialDataIntegrator.cpp
  src/navx_lib/InertialDataIntegrator.h
  src/navx_lib/OffsetTracker.cpp
  src/navx_lib/OffsetTracker.h
  src/navx_lib/SerialIO.cpp
  src/navx_lib/SerialIO.h
  src/navx_lib/SerialPort.cpp
)

# Link the library containing the AHRS implementation to your executable
target_link_libraries(talker
  navx_lib
)

ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)

install(TARGETS
  talker
  # listener
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
