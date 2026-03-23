cmake_minimum_required(VERSION 3.8)
project(motion_system_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motion_system_msgs REQUIRED)

set(MOTOR_MANAGER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../motor_manager")
if(EXISTS "${MOTOR_MANAGER_ROOT}/CMakeLists.txt")
  find_library(ETHERCAT_LIB ethercat HINTS /usr/local/lib /usr/lib)
  if(NOT ETHERCAT_LIB)
    message(FATAL_ERROR "libethercat not found (IgH EtherCAT Master). Install or set ETHERCAT_LIB.")
  endif()

  add_subdirectory("${MOTOR_MANAGER_ROOT}" motor_manager_ext)

  add_executable(motor_manager_node
    src/motor_manager_node.cpp
  )
  target_include_directories(motor_manager_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(motor_manager_node
    rclcpp
    motion_system_msgs
  )
  # IgH libethercat must resolve ecrt_* after the in-tree static libethercat.a is linked.
  target_link_libraries(motor_manager_node
    motor_manager::motor_manager
    "-Wl,--push-state,--no-as-needed"
    ${ETHERCAT_LIB}
    "-Wl,--pop-state"
  )
  target_compile_features(motor_manager_node PUBLIC cxx_std_17)

  install(TARGETS motor_manager_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "motor_manager not found at ${MOTOR_MANAGER_ROOT}; motor_manager_node will not be built.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips the linter (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
