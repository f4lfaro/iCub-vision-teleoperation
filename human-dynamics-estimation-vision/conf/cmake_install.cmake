# Install script for directory: /home/fabiana/iCub/human-dynamics-estimation-vision2/conf

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/share/HumanDynamicsEstimation/Human.xml;/usr/lib/share/HumanDynamicsEstimation/HumanJointTorquesYarpScope.xml;/usr/lib/share/HumanDynamicsEstimation/HumanLoggerExample.xml;/usr/lib/share/HumanDynamicsEstimation/HumanStateProvider.xml;/usr/lib/share/HumanDynamicsEstimation/HumanStateProvider2.xml;/usr/lib/share/HumanDynamicsEstimation/HumanStateProvider_second.xml;/usr/lib/share/HumanDynamicsEstimation/RobotPosePublisher.xml;/usr/lib/share/HumanDynamicsEstimation/RobotPositionController_iCub.xml;/usr/lib/share/HumanDynamicsEstimation/RobotPositionController_iCub2_5.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_Atlas.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_Baxter.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_Human.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_Nao.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_Walkman.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_ergoCub_openxr_ifeel.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub2_5.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub2_5_Pole.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub2_5_openxr.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub2_5_openxr_ifeel.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub3.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub3_Pole.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub3_Xsens_SenseGlove.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub3_openxr.xml;/usr/lib/share/HumanDynamicsEstimation/RobotStateProvider_iCub3_openxr_ifeel.xml;/usr/lib/share/HumanDynamicsEstimation/TransformServer.xml;/usr/lib/share/HumanDynamicsEstimation/WholeBodyRetargeting.xml;/usr/lib/share/HumanDynamicsEstimation/hands-pHRI.xml;/usr/lib/share/HumanDynamicsEstimation/pHRI.xml;/usr/lib/share/HumanDynamicsEstimation/transformServer_to_iwear.xml")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/share/HumanDynamicsEstimation" TYPE FILE FILES
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/Human.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/HumanJointTorquesYarpScope.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/HumanLoggerExample.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/HumanStateProvider.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/HumanStateProvider2.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/HumanStateProvider_second.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotPosePublisher.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotPositionController_iCub.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotPositionController_iCub2_5.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_Atlas.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_Baxter.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_Human.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_Nao.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_Walkman.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_ergoCub_openxr_ifeel.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub2_5.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub2_5_Pole.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub2_5_openxr.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub2_5_openxr_ifeel.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub3.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub3_Pole.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub3_Xsens_SenseGlove.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub3_openxr.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/RobotStateProvider_iCub3_openxr_ifeel.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/TransformServer.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/WholeBodyRetargeting.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/hands-pHRI.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/pHRI.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/transformServer_to_iwear.xml"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizer.ini;/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizerWithDynamics.ini;/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizer_ergoCub_openxr_ifeel.ini;/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizer_iCub2_5.ini;/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizer_iCub2_5_openxr.ini;/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizer_iCub2_5_openxr_ifeel.ini;/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizer_iCub3.ini;/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizer_iCub3_openxr.ini;/usr/lib/share/HumanDynamicsEstimation/HumanStateVisualizer_iCub3_openxr_ifeel.ini")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/share/HumanDynamicsEstimation" TYPE FILE FILES
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizer.ini"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizerWithDynamics.ini"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizer_ergoCub_openxr_ifeel.ini"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizer_iCub2_5.ini"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizer_iCub2_5_openxr.ini"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizer_iCub2_5_openxr_ifeel.ini"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizer_iCub3.ini"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizer_iCub3_openxr.ini"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/app/HumanStateVisualizer_iCub3_openxr_ifeel.ini"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/applications" TYPE FILE FILES
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/HDERetargeting_iCub2_5.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/HDERetargeting_iCub2_5_ifeel.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/HumanDynamicsEstimation-HumanDumper.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/HumanDynamicsEstimation-HumanDynamics.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/HumanDynamicsEstimation-HumanKinematics.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/HumanDynamicsEstimation-pHRI.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/HumanDynamicsEstimation.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/XsensRetargetingPositionControlPoleiCub2_5.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/XsensRetargetingPositionControlPoleiCub3.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/XsensRetargetingVisualizationiCub2_5.xml"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/xml/applications/XsensRetargetingVisualizationiCub3.xml"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/share/HumanDynamicsEstimation/humanSubject01_66dof.urdf;/usr/lib/share/HumanDynamicsEstimation/iCubGazeboV2_5.urdf;/usr/lib/share/HumanDynamicsEstimation/iCubGenova02.urdf;/usr/lib/share/HumanDynamicsEstimation/iCubGenova04.urdf;/usr/lib/share/HumanDynamicsEstimation/teleoperation_iCub_model_V_2_5.urdf;/usr/lib/share/HumanDynamicsEstimation/teleoperation_iCub_model_V_3.urdf")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/share/HumanDynamicsEstimation" TYPE FILE FILES
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/humanSubject01_66dof.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/iCubGazeboV2_5.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/iCubGenova02.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/iCubGenova04.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/teleoperation_iCub_model_V_2_5.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/teleoperation_iCub_model_V_3.urdf"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/share/HDERviz/package.xml")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/share/HDERviz" TYPE FILE FILES "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/ros/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/share/HDERviz/launch/HDERviz.launch;/usr/lib/share/HDERviz/launch/iCub3Rviz.launch;/usr/lib/share/HDERviz/launch/iCubRviz.launch;/usr/lib/share/HDERviz/launch/twoHumansRviz.launch")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/share/HDERviz/launch" TYPE FILE FILES
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/ros/launch/HDERviz.launch"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/ros/launch/iCub3Rviz.launch"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/ros/launch/iCubRviz.launch"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/ros/launch/twoHumansRviz.launch"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/share/HDERviz/rviz/HDERviz.rviz;/usr/lib/share/HDERviz/rviz/iCubRviz.rviz;/usr/lib/share/HDERviz/rviz/twoHumansRviz.rviz")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/share/HDERviz/rviz" TYPE FILE FILES
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/ros/rviz/HDERviz.rviz"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/ros/rviz/iCubRviz.rviz"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/ros/rviz/twoHumansRviz.rviz"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/share/HDERviz/urdfs/humanSubject01_66dof.urdf;/usr/lib/share/HDERviz/urdfs/iCubGazeboV2_5.urdf;/usr/lib/share/HDERviz/urdfs/iCubGenova02.urdf;/usr/lib/share/HDERviz/urdfs/iCubGenova04.urdf;/usr/lib/share/HDERviz/urdfs/teleoperation_iCub_model_V_2_5.urdf;/usr/lib/share/HDERviz/urdfs/teleoperation_iCub_model_V_3.urdf")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/lib/share/HDERviz/urdfs" TYPE FILE FILES
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/humanSubject01_66dof.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/iCubGazeboV2_5.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/iCubGenova02.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/iCubGenova04.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/teleoperation_iCub_model_V_2_5.urdf"
    "/home/fabiana/iCub/human-dynamics-estimation-vision2/conf/urdfs/teleoperation_iCub_model_V_3.urdf"
    )
endif()

