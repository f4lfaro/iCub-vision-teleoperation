namespace yarp hde.msgs

/**
 * Representation of a 3D vector
 */
struct Vector3 {
    1: double x;
    2: double y;
    3: double z;
}

/**
 * Representation of a Quaternion
 */
struct Quaternion {
    1: double w;
    2: Vector3 imaginary;
}

struct Transform {
    1: Vector3 position
    2: Quaternion orientation
}

enum KinematicTargetType {
    NONE,
    POSE,
    POSEANDVELOCITY,
    POSITION,
    POSITIONANDVELOCITY,
    ORIENTATION,
    ORIENTATIONANDVELOCITY,
    GRAVITY,
    FLOORCONTACT
}

struct VisionTarget {
    1: string inputName;
    2: string linkName;
    3: KinematicTargetType type;
    4: i16 targetIdx;
    5: Vector3 position;
    6: Quaternion orientation;
    7: Vector3 linearVelocity;
    8: Vector3 angularVelocity;
    9: Transform calibrationWorldToMeasurementWorld;
    10: Transform calibrationMeasurementToLink;
    11: Vector3 positionScaleFactor;
}

struct VisionTargets {
    1: map<string,VisionTarget> targets;
}

