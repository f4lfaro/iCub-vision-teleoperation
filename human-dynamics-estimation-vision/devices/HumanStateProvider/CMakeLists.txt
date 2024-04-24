# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(iDynTree REQUIRED)
find_package(OsqpEigen 0.4.0 REQUIRED)


yarp_prepare_plugin(human_state_provider
    TYPE hde::devices::HumanStateProvider
    INCLUDE HumanStateProvider.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanStateProvider
    HumanStateProvider.cpp
    HumanStateProvider.h
    IKWorkerPool.cpp
    IKWorkerPool.h)

target_include_directories(HumanStateProvider PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(HumanStateProvider PUBLIC
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    iDynTree::idyntree-model
    iDynTree::idyntree-inverse-kinematics
    iDynTree::idyntree-high-level
    iDynTree::idyntree-modelio #
    iDynTree::idyntree-visualization #
    OsqpEigen::OsqpEigen
    IHumanState
    #IWearableTargets
    IVisionTargets
    HumanDynamicsEstimation::utils
    HumanDynamicsEstimation::algorithms
    #IHumanWrench
    )
    

if(MSVC)
  target_compile_options(HumanStateProvider PRIVATE /bigobj)
endif()

yarp_install(
    TARGETS HumanStateProvider
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
