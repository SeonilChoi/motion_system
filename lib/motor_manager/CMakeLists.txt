cmake_minimum_required(VERSION 3.16)
project(motor_manager VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(common_motor_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(core/motor_interface)
add_subdirectory(communications/ethercat)
add_subdirectory(hardware/minas)
add_subdirectory(motor_manager)

install(DIRECTORY core/motor_interface/include/ DESTINATION include)
install(DIRECTORY communications/ethercat/include/ DESTINATION include)
install(DIRECTORY hardware/minas/include/ DESTINATION include)
install(DIRECTORY motor_manager/include/ DESTINATION include)

install(
  TARGETS motor_manager ethercat minas motor_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(common_motor_interface)
ament_package()
