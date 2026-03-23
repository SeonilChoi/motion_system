find_library(MOTOR_MANAGER_IGH_ETHERCAT_LIB ethercat
  HINTS /usr/local/lib /usr/lib
  DOC "IgH EtherCAT Master client library (libethercat)")
if(NOT MOTOR_MANAGER_IGH_ETHERCAT_LIB)
  message(FATAL_ERROR
    "libethercat (IgH EtherCAT Master) not found. Install it or set MOTOR_MANAGER_IGH_ETHERCAT_LIB.")
endif()

add_library(ethercat
  src/ethercat_master.cpp
  src/ethercat_controller.cpp
)

target_include_directories(ethercat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ethercat
  PUBLIC motor_interface::motor_interface
  PRIVATE "${MOTOR_MANAGER_IGH_ETHERCAT_LIB}"
)

target_compile_features(ethercat PUBLIC cxx_std_17)

add_library(ethercat::ethercat ALIAS ethercat)
