/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_QUATERNION_H
#define YARP_THRIFT_GENERATOR_STRUCT_QUATERNION_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <hde/msgs/Vector3.h>

namespace hde::msgs {

/**
 * Representation of a Quaternion
 */
class Quaternion :
        public yarp::os::idl::WirePortable
{
public:
    // Fields
    double w{0.0};
    Vector3 imaginary{};

    // Default constructor
    Quaternion() = default;

    // Constructor with field values
    Quaternion(const double w,
               const Vector3& imaginary);

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
    typedef yarp::os::idl::Unwrapped<Quaternion> unwrapped;

private:
    // read/write w field
    bool read_w(yarp::os::idl::WireReader& reader);
    bool write_w(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_w(yarp::os::idl::WireReader& reader);
    bool nested_write_w(const yarp::os::idl::WireWriter& writer) const;

    // read/write imaginary field
    bool read_imaginary(yarp::os::idl::WireReader& reader);
    bool write_imaginary(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_imaginary(yarp::os::idl::WireReader& reader);
    bool nested_write_imaginary(const yarp::os::idl::WireWriter& writer) const;
};

} // namespace hde::msgs

#endif // YARP_THRIFT_GENERATOR_STRUCT_QUATERNION_H