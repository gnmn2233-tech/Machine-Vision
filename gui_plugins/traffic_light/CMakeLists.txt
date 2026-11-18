cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(gz-ros-traffic-light-gui)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Qt5 COMPONENTS Core Quick QuickControls2 REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GZ-GUI_CXX_FLAGS}")

# moc 显式跑一遍（AUTOMOC 在这套 CMake/Qt5 组合里对只被 .cc include 的头文件不生效，
# 结果是 vtable/staticMetaObject 未定义、gz-gui 加载时报 undefined symbol）
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/moc_TrafficLight.cpp
  COMMAND ${Qt5Core_MOC_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/TrafficLight.hh
          -o ${CMAKE_CURRENT_BINARY_DIR}/moc_TrafficLight.cpp
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/TrafficLight.hh
  COMMENT "moc TrafficLight.hh"
)

QT5_ADD_RESOURCES(resources_RCC TrafficLight.qrc)

add_library(TrafficLight SHARED
  TrafficLight.cc
  ${CMAKE_CURRENT_BINARY_DIR}/moc_TrafficLight.cpp
  ${resources_RCC}
)

install(TARGETS TrafficLight
  LIBRARY DESTINATION "$ENV{HOME}/.gz/gui/plugins")

target_link_libraries(TrafficLight
  gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES}
)