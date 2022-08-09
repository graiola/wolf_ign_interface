cmake_minimum_required(VERSION 3.0.0)
project(wolf_ign_interface)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  wolf_hardware_interface
  ign_ros_control
  transmission_interface
  pluginlib
  realtime_tools
  urdf)

if("$ENV{IGNITION_VERSION}" STREQUAL "citadel")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Citadel")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "dome")
  find_package(ignition-gazebo4 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo4_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Dome")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "edifice")
  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Edifice")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")

else()
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Dome")
endif()

find_package(ignition-plugin1 REQUIRED)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

catkin_package(
  LIBRARIES wolf_ign_hw
  CATKIN_DEPENDS wolf_hardware_interface ign_ros_control
  INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

## Libraries
add_library(wolf_ign_hw src/wolf_ign_hw.cpp)
target_link_libraries(wolf_ign_hw ignition-gazebo${IGN_GAZEBO_VER}::core ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS wolf_ign_hw
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES wolf_ign_hw_plugins.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

