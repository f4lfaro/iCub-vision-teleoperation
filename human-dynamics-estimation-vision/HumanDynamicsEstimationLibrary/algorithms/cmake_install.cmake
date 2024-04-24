# Install script for directory: /home/fabiana/iCub/human-dynamics-estimation-vision2/HumanDynamicsEstimationLibrary/algorithms

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/lib")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "shlib" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libHumanDynamicsEstimation_algorithms.so.2.7.1" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libHumanDynamicsEstimation_algorithms.so.2.7.1")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libHumanDynamicsEstimation_algorithms.so.2.7.1"
         RPATH "$ORIGIN/:$ORIGIN/../lib:/usr/lib/lib:/usr/local/lib:/home/fabiana/repos/robotology-superbuild/build/install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/fabiana/iCub/human-dynamics-estimation-vision2/build/lib/libHumanDynamicsEstimation_algorithms.so.2.7.1")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libHumanDynamicsEstimation_algorithms.so.2.7.1" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libHumanDynamicsEstimation_algorithms.so.2.7.1")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libHumanDynamicsEstimation_algorithms.so.2.7.1"
         OLD_RPATH "/home/fabiana/iCub/human-dynamics-estimation-vision2/build/lib:/usr/lib/lib:/usr/local/lib:/home/fabiana/repos/robotology-superbuild/build/install/lib:"
         NEW_RPATH "$ORIGIN/:$ORIGIN/../lib:/usr/lib/lib:/usr/local/lib:/home/fabiana/repos/robotology-superbuild/build/install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libHumanDynamicsEstimation_algorithms.so.2.7.1")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "shlib" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/fabiana/iCub/human-dynamics-estimation-vision2/build/lib/libHumanDynamicsEstimation_algorithms.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/hde/algorithms" TYPE FILE FILES
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/HumanDynamicsEstimationLibrary/algorithms/include/hde/algorithms/InverseVelocityKinematics.hpp"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/HumanDynamicsEstimationLibrary/algorithms/include/hde/algorithms/DynamicalInverseKinematics.hpp"
    )
endif()

