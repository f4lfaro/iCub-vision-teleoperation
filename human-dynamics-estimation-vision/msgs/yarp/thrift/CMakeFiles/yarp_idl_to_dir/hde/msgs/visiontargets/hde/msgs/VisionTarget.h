/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_VISIONTARGET_H
#define YARP_THRIFT_GENERATOR_STRUCT_VISIONTARGET_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <hde/msgs/KinematicTargetType.h>
#include <hde/msgs/Quaternion.h>
#include <hde/msgs/Transform.h>
#include <hde/msgs/Vector3.h>

namespace hde::msgs {

class VisionTarget :
        public yarp::os::idl::WirePortable
{
public:
    // Fields
    std::string inputName{};
    std::string linkName{};
    KinematicTargetType type{NONE};
    std::int16_t targetIdx{0};
    Vector3 position{};
    Quaternion orientation{};
    Vector3 linearVelocity{};
    Vector3 angularVelocity{};
    Transform calibrationWorldToMeasurementWorld{};
    Transform calibrationMeasurementToLink{};
    Vector3 positionScaleFactor{};

    // Default constructor
    VisionTarget() = default;

    // Constructor with field values
    VisionTarget(const std::string& inputName,
                 const std::string& linkName,
                 const KinematicTargetType type,
                 const std::int16_t targetIdx,
                 const Vector3& position,
                 const Quaternion& orientation,
                 const Vector3& linearVelocity,
                 const Vector3& angularVelocity,
                 const Transform& calibrationWorldToMeasurementWorld,
                 const Transform& calibrationMeasurementToLink,
                 const Vector3& positionScaleFactor);

    // Read structure on a Wire
    bool read(yarp::os::idl::WireReader& reader) override;

    // Read structure on a Connection
    bool read(yarp::os::ConnectionReader& connection) override;

    // Write structure on a Wire
    bool write(const yarp::os::idl::WireWriter& writer) const override;

    // Write structure on a Connection
    bool write(yarp::os::ConnectionWriter& connection) const override;

    // Convert to a printable string
    std::string toString() const;

    // If you want to serialize this class without nesting, use this helper
    typedef yarp::os::idl::Unwrapped<VisionTarget> unwrapped;

private:
    // read/write inputName field
    bool read_inputName(yarp::os::idl::WireReader& reader);
    bool write_inputName(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_inputName(yarp::os::idl::WireReader& reader);
    bool nested_write_inputName(const yarp::os::idl::WireWriter& writer) const;

    // read/write linkName field
    bool read_linkName(yarp::os::idl::WireReader& reader);
    bool write_linkName(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_linkName(yarp::os::idl::WireReader& reader);
    bool nested_write_linkName(const yarp::os::idl::WireWriter& writer) const;

    // read/write type field
    bool read_type(yarp::os::idl::WireReader& reader);
    bool write_type(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_type(yarp::os::idl::WireReader& reader);
    bool nested_write_type(const yarp::os::idl::WireWriter& writer) const;

    // read/write targetIdx field
    bool read_targetIdx(yarp::os::idl::WireReader& reader);
    bool write_targetIdx(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_targetIdx(yarp::os::idl::WireReader& reader);
    bool nested_write_targetIdx(const yarp::os::idl::WireWriter& writer) const;

    // read/write position field
    bool read_position(yarp::os::idl::WireReader& reader);
    bool write_position(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_position(yarp::os::idl::WireReader& reader);
    bool nested_write_position(const yarp::os::idl::WireWriter& writer) const;

    // read/write orientation field
    bool read_orientation(yarp::os::idl::WireReader& reader);
    bool write_orientation(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_orientation(yarp::os::idl::WireReader& reader);
    bool nested_write_orientation(const yarp::os::idl::WireWriter& writer) const;

    // read/write linearVelocity field
    bool read_linearVelocity(yarp::os::idl::WireReader& reader);
    bool write_linearVelocity(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_linearVelocity(yarp::os::idl::WireReader& reader);
    bool nested_write_linearVelocity(const yarp::os::idl::WireWriter& writer) const;

    // read/write angularVelocity field
    bool read_angularVelocity(yarp::os::idl::WireReader& reader);
    bool write_angularVelocity(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_angularVelocity(yarp::os::idl::WireReader& reader);
    bool nested_write_angularVelocity(const yarp::os::idl::WireWriter& writer) const;

    // read/write calibrationWorldToMeasurementWorld field
    bool read_calibrationWorldToMeasurementWorld(yarp::os::idl::WireReader& reader);
    bool write_calibrationWorldToMeasurementWorld(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_calibrationWorldToMeasurementWorld(yarp::os::idl::WireReader& reader);
    bool nested_write_calibrationWorldToMeasurementWorld(const yarp::os::idl::WireWriter& writer) const;

    // read/write calibrationMeasurementToLink field
    bool read_calibrationMeasurementToLink(yarp::os::idl::WireReader& reader);
    bool write_calibrationMeasurementToLink(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_calibrationMeasurementToLink(yarp::os::idl::WireReader& reader);
    bool nested_write_calibrationMeasurementToLink(const yarp::os::idl::WireWriter& writer) const;

    // read/write positionScaleFactor field
    bool read_positionScaleFactor(yarp::os::idl::WireReader& reader);
    bool write_positionScaleFactor(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_positionScaleFactor(yarp::os::idl::WireReader& reader);
    bool nested_write_positionScaleFactor(const yarp::os::idl::WireWriter& writer) const;
};

} // namespace hde::msgs

#endif // YARP_THRIFT_GENERATOR_STRUCT_VISIONTARGET_H