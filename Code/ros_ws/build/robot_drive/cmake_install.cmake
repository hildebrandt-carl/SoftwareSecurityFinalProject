# Install script for directory: /home/parallels/Desktop/Parallels Shared Folders/Dropbox/UVA/Classes/Software Security/Final_Project/Code/ros_ws/src/robot_drive

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/parallels/Desktop/Parallels Shared Folders/Dropbox/UVA/Classes/Software Security/Final_Project/Code/ros_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/psf/Dropbox/UVA/Classes/Software Security/Final_Project/Code/ros_ws/build/robot_drive/catkin_generated/installspace/robot_drive.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_drive/cmake" TYPE FILE FILES
    "/media/psf/Dropbox/UVA/Classes/Software Security/Final_Project/Code/ros_ws/build/robot_drive/catkin_generated/installspace/robot_driveConfig.cmake"
    "/media/psf/Dropbox/UVA/Classes/Software Security/Final_Project/Code/ros_ws/build/robot_drive/catkin_generated/installspace/robot_driveConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_drive" TYPE FILE FILES "/home/parallels/Desktop/Parallels Shared Folders/Dropbox/UVA/Classes/Software Security/Final_Project/Code/ros_ws/src/robot_drive/package.xml")
endif()

