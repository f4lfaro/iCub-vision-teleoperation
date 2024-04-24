/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_VISIONTARGETSREMAPPER
#define HDE_DEVICES_VISIONTARGETSREMAPPER

#include <hde/interfaces/IVisionTargets.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace hde::msgs {
    class VisionTargets;
} // namespace hde::msgs
namespace hde::devices {
    class VisionTargetsRemapper;
} // namespace hde::devices

class hde::devices::VisionTargetsRemapper final
    : public yarp::dev::DeviceDriver
    , public hde::interfaces::IVisionTargets
    , public yarp::os::TypedReaderCallback<hde::msgs::VisionTargets>
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    VisionTargetsRemapper();
    ~VisionTargetsRemapper() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(hde::msgs::VisionTargets& visionTargetsMgs) override;

    // IVisionTargets interface
    std::vector<hde::TargetName> getAllTargetsName() const override;
    std::shared_ptr<hde::VisionTarget> getTarget(const TargetName name) const override;
};

#endif // HDE_DEVICES_VISIONTARGETSREMAPPER

