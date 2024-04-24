/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ENUM_KINEMATICTARGETTYPE_H
#define YARP_THRIFT_GENERATOR_ENUM_KINEMATICTARGETTYPE_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace hde::msgs {

enum KinematicTargetType : int32_t
{
    NONE = 0,
    POSE = 1,
    POSEANDVELOCITY = 2,
    POSITION = 3,
    POSITIONANDVELOCITY = 4,
    ORIENTATION = 5,
    ORIENTATIONANDVELOCITY = 6,
    GRAVITY = 7,
    FLOORCONTACT = 8
};

class KinematicTargetTypeConverter
{
public:
    static int32_t fromString(const std::string& input);
    static std::string toString(int32_t input);
};

} // namespace hde::msgs

#endif // YARP_THRIFT_GENERATOR_ENUM_KINEMATICTARGETTYPE_H
