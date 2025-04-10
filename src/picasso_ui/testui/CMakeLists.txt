cmake_minimum_required(VERSION 3.5)

project(testui VERSION 0.1 LANGUAGES CXX)

#set(ament_cmake_export_definitions_DIR "/opt/ros/humble/share/ament_cmake_export_definitions/cmake")


set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_PREFIX_PATH "/opt/ros/humble/" ${CMAKE_PREFIX_PATH})
#set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/Qt5" ${CMAKE_PREFIX_PATH})
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(Qt5Core_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5Core")
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Multimedia MultimediaWidgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_image_to_qimage REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

if(Qt5_VERSION VERSION_GREATER "5.99.99")
    qt_add_executable(testui
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
    set_property(TARGET testui APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(testui SHARED
            ${PROJECT_SOURCES}
        )
    # Define properties for Android with Qt 5 after find_package() calls as:
    #    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        # Removed the invalid 'build_depend, exec_depend' line.
        add_executable(testui
            ${PROJECT_SOURCES}
        )
    endif()
endif()

set_target_properties(testui PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(testui)
endif()





#   # Add include directories for ROS 2
  include_directories(
      ${rclcpp_INCLUDE_DIRS}
      ${std_msgs_INCLUDE_DIRS}
  )


# Find the Qt5Widgets package
# find_package(Qt5 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets)





ament_target_dependencies(testui
  rclcpp
  std_msgs
  ros_image_to_qimage
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
)

# Link Qt5 libraries to your target
target_link_libraries(testui
  # PRIVATE
    Qt5::Widgets
    Qt5::Multimedia
    Qt5::MultimediaWidgets
    ${OpenCV_LIBS}
  #  rclcpp
   #   ${rclcpp_LIBRARIES}
  # std_msgs
)

# Add the UI file to the target
set(UI_FILES
    mainwindow.ui
)
#ament_target_dependencies(testui rclcpp std_msgs)
# Include the UI files in the build
# qt5_wrap_ui(UI_HEADERS ${UI_FILES})

#target_include_directories(testui
#    PRIVATE
#    /opt/ros/humble/include)

install(TARGETS
  #testui
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  config
  launch
  #rviz
  DESTINATION share/${PROJECT_NAME}
)

#target_include_directories(testui PRIVATE /opt/ros/humble/include)
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")