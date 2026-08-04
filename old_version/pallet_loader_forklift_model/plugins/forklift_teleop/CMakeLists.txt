cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(pallet_loader_forklift_teleop)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core Qml Quick QuickControls2 REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

qt5_add_resources(FORKLIFT_TELEOP_RESOURCES forklift_teleop.qrc)

add_library(ForkliftTeleop SHARED
  ForkliftTeleop.cc
  ForkliftTeleop.hh
  ${FORKLIFT_TELEOP_RESOURCES}
)

target_link_libraries(ForkliftTeleop
  PRIVATE
    gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
    gz-transport${GZ_TRANSPORT_VER}::core
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    Qt5::Core
    Qt5::Qml
    Qt5::Quick
    Qt5::QuickControls2
)
