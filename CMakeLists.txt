cmake_minimum_required(VERSION 3.16)
project(optitrack_vrpn_clean VERSION 0.1.0 LANGUAGES CXX)

option(OPTITRACK_BUILD_CLIENT "Build the VRPN client executable" ON)
option(OPTITRACK_BUILD_TESTS "Build small self-tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

add_library(optitrack_vrpn_clean_core
  src/config.cpp
  src/output_dispatcher.cpp
  src/pose.cpp
  src/time_manager.cpp
)
target_include_directories(optitrack_vrpn_clean_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_link_libraries(optitrack_vrpn_clean_core PUBLIC ${YAML_CPP_TARGET})

if(OPTITRACK_BUILD_CLIENT)
  find_package(VRPN REQUIRED)

  add_library(optitrack_vrpn_clean_client
    src/optitrack_client.cpp
    src/tracker_handler.cpp
  )
  target_link_libraries(optitrack_vrpn_clean_client
    PUBLIC
      optitrack_vrpn_clean_core
      VRPN::VRPN)

  add_executable(optitrack_vrpn_clean_node src/main.cpp)
  target_link_libraries(optitrack_vrpn_clean_node PRIVATE optitrack_vrpn_clean_client)

  add_executable(mocap_pair_diagnostic src/mocap_pair_diagnostic.cpp)
  target_link_libraries(mocap_pair_diagnostic PRIVATE optitrack_vrpn_clean_core)
endif()

if(OPTITRACK_BUILD_TESTS)
  enable_testing()
  add_executable(optitrack_conversion_test tests/conversion_test.cpp)
  target_link_libraries(optitrack_conversion_test PRIVATE optitrack_vrpn_clean_core)
  add_test(NAME optitrack_conversion_test COMMAND optitrack_conversion_test)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS optitrack_vrpn_clean_core
  EXPORT optitrack_vrpn_cleanTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
if(TARGET optitrack_vrpn_clean_client)
  install(TARGETS optitrack_vrpn_clean_client optitrack_vrpn_clean_node mocap_pair_diagnostic
    EXPORT optitrack_vrpn_cleanTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endif()
