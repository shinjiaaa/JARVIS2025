cmake_minimum_required(VERSION 3.8)
project(ppo_drone_ros2)

# 컴파일러 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)

# Python 스크립트 설치
ament_python_install_package(${PROJECT_NAME})

# 실행 파일 설치
install(PROGRAMS
  advanced_teleop_control.py
  check_gui_requirements.py
  gui_teleop_control_en.py
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 설정 파일 설치
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 패키지 정보 설치
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# C++ 라이브러리 (필요시)
# add_library(${PROJECT_NAME}_lib
#   src/drone_utils.cpp
# )

# C++ 실행 파일 (필요시)
# add_executable(drone_controller_cpp src/drone_controller.cpp)
# ament_target_dependencies(drone_controller_cpp
#   rclcpp
#   std_msgs
#   geometry_msgs
#   mavros_msgs
# )

# 테스트
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

# 패키지 완료
ament_package() 