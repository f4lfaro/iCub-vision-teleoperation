/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-FileCopyrightText: 2006-2010 RobotCub Consortium
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/conf/api.h>
#include <yarp/os/SharedLibraryClass.h>
#include <yarp/dev/Drivers.h>
#include </home/fabiana/iCub/human-dynamics-estimation-vision2/devices/HumanDynamicsEstimator/HumanDynamicsEstimator.h>

#ifdef YARP_STATIC_PLUGIN
#  define YARP_PLUGIN_IMPORT
#  define YARP_PLUGIN_EXPORT
#else
#  define YARP_PLUGIN_IMPORT YARP_HELPER_DLL_IMPORT
#  define YARP_PLUGIN_EXPORT YARP_HELPER_DLL_EXPORT
#endif

#ifdef YARP_STATIC_PLUGIN
YARP_PLUGIN_EXPORT void add_owned_human_dynamics_estimator(const char *owner) {
    yarp::dev::DriverCreator *factory =
        new yarp::dev::DriverCreatorOf<hde::devices::HumanDynamicsEstimator>("human_dynamics_estimator",
                "",
                "hde::devices::HumanDynamicsEstimator");
    yarp::dev::Drivers::factory().add(factory); // hand factory over to YARP
}
#endif

YARP_DEFINE_SHARED_SUBCLASS(human_dynamics_estimator, hde::devices::HumanDynamicsEstimator, yarp::dev::DeviceDriver)
