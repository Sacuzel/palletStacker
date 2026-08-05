cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(pallet_stacker_forklift_teleop LANGUAGES CXX)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Qt5 REQUIRED COMPONENTS Core Qml Quick QuickControls2)
find_package(gz-gui8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

qt5_add_resources(FORKLIFT_TELEOP_RESOURCES ForkliftTeleop.qrc)

add_library(ForkliftTeleop SHARED
  ForkliftTeleop.cc
  ForkliftTeleop.hh
  ${FORKLIFT_TELEOP_RESOURCES}
)

# Keep the build product at one predictable project-local path. main.py then
# installs a runtime copy as ~/.gz/gui/plugins/libForkliftTeleop.so. The world
# uses filename="ForkliftTeleop" so Gazebo can resolve both the library and the
# embedded :/ForkliftTeleop/ForkliftTeleop.qml resource by the same base name.
set_target_properties(ForkliftTeleop PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

target_link_libraries(ForkliftTeleop PRIVATE
  gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
  gz-transport${GZ_TRANSPORT_VER}::core
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  Qt5::Core
  Qt5::Qml
  Qt5::Quick
  Qt5::QuickControls2
)

target_compile_options(ForkliftTeleop PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)
