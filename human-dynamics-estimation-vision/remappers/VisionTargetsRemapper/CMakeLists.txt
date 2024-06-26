# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_prepare_plugin(vision_targets_remapper
    TYPE hde::devices::VisionTargetsRemapper
    INCLUDE VisionTargetsRemapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(VisionTargetsRemapper
VisionTargetsRemapper.cpp
VisionTargetsRemapper.h)

target_include_directories(VisionTargetsRemapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(VisionTargetsRemapper PUBLIC
    IVisionTargets
    VisionTargetsMsg
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS VisionTargetsRemapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

