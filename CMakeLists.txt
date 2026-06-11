cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Student.msg"
  "srv/AddTwoInts.srv"
  DEPENDENCIES std_msgs
)

file(GLOB PYTHON_SCRIPTS "scripts/*.py")
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/lib
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY path
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY sensor
  DESTINATION share/${PROJECT_NAME}
)

ament_package()