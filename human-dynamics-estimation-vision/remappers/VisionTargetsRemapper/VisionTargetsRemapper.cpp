#include "VisionTargetsRemapper.h"

#include <hde/msgs/VisionTargets.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <unordered_map>
#include <iostream>

const std::string RemapperName = "VisionTargetsRemapper";
const std::string logPrefix = RemapperName + " :";

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

const std::map<hde::msgs::KinematicTargetType, hde::KinematicTargetType> mapKinematicTargetType = {
    {hde::msgs::KinematicTargetType::NONE, hde::KinematicTargetType::none},
    {hde::msgs::KinematicTargetType::POSE, hde::KinematicTargetType::pose},
    {hde::msgs::KinematicTargetType::POSEANDVELOCITY, hde::KinematicTargetType::poseAndVelocity},
    {hde::msgs::KinematicTargetType::POSITION, hde::KinematicTargetType::position},
    {hde::msgs::KinematicTargetType::POSITIONANDVELOCITY, hde::KinematicTargetType::positionAndVelocity},
    {hde::msgs::KinematicTargetType::ORIENTATION, hde::KinematicTargetType::orientation},
    {hde::msgs::KinematicTargetType::ORIENTATIONANDVELOCITY, hde::KinematicTargetType::orientationAndVelocity},
    {hde::msgs::KinematicTargetType::GRAVITY, hde::KinematicTargetType::gravity},
    {hde::msgs::KinematicTargetType::FLOORCONTACT, hde::KinematicTargetType::floorContact},
};

iDynTree::Vector3 generateVector3FromMsg(const hde::msgs::Vector3& input)
{
    iDynTree::Vector3 vector3;
    vector3.setVal(0, input.x);
    vector3.setVal(1, input.y);
    vector3.setVal(2, input.z);
    return vector3;
}

iDynTree::Rotation generateRotationFromMsg(const hde::msgs::Quaternion& input)
{
    iDynTree::Vector4 vector4;
    vector4.setVal(0, input.w);
    vector4.setVal(1, input.imaginary.x);
    vector4.setVal(2, input.imaginary.y);
    vector4.setVal(3, input.imaginary.z);
    return iDynTree::Rotation::RotationFromQuaternion(vector4);
}

iDynTree::Transform generateTransformFromMsg(const hde::msgs::Transform& input)
{
    return iDynTree::Transform(generateRotationFromMsg(input.orientation), iDynTree::Position(generateVector3FromMsg(input.position)));
}

// Implementacion VisionTargetRemapper
class VisionTargetsRemapper::impl
{
public:
    yarp::os::Network network;
    yarp::os::BufferedPort<hde::msgs::VisionTargets> inputPort; // buffered port que almacena elementos tipo hde::msgs::visionTargets
    bool terminationCall = false;

    mutable std::recursive_mutex mutex;

    // Buffer VisionTarget variables
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::VisionTarget>> visionTargets;
};

// ==============
// IVISION REMAPPER
// ==============

VisionTargetsRemapper::VisionTargetsRemapper()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

VisionTargetsRemapper::~VisionTargetsRemapper() = default;

bool VisionTargetsRemapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    if (!(config.check("visionTargetsDataPort") && config.find("visionTargetsDataPort").isString())) { //cambiando lista por string ya que es un solo puerto
        yError() << logPrefix << "visionTargetsDataPort option does not exist or it is not a list";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    std::string visionTargetsDataPortName = config.find("visionTargetsDataPort").asString();

    // Initialize the network
    // TODO: is this required in every DeviceDriver?
    pImpl->network = yarp::os::Network();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << logPrefix << "YARP server wasn't found active.";
        return false;
    }

    // ==========================
    // CONFIGURE INPUT DATA PORTS
    // ==========================
    yDebug() << logPrefix << "Configuring input data ports";

    pImpl->inputPort.useCallback(*this);
    if (!pImpl->inputPort.open("...")) {
        yError() << logPrefix << "Failed to open port" << visionTargetsDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << logPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(visionTargetsDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << logPrefix << "Failed to connect " << visionTargetsDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << logPrefix << "Opened correctly";
    return true;
}

void VisionTargetsRemapper::threadRelease()
{}

bool VisionTargetsRemapper::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void VisionTargetsRemapper::run()
{
    return;
}

void VisionTargetsRemapper::onRead(hde::msgs::VisionTargets& visionTargetsData)
{
    if(!pImpl->terminationCall) {

        for (auto visionTarget : visionTargetsData.targets)
        {
            if (pImpl->visionTargets.find(visionTarget.first) == pImpl->visionTargets.end())
            {
                pImpl->visionTargets[visionTarget.first] = std::make_shared<hde::VisionTarget>(visionTarget.second.inputName,
                                                                                                visionTarget.second.linkName,
                                                                                                mapKinematicTargetType.at(visionTarget.second.type),
                                                                                                visionTarget.second.targetIdx);
            }

            // TODO: this can be done more efficiently considering different target type have incomplete information
            pImpl->visionTargets[visionTarget.first].get()->position = generateVector3FromMsg(visionTarget.second.position);
            pImpl->visionTargets[visionTarget.first].get()->rotation = generateRotationFromMsg(visionTarget.second.orientation);
            pImpl->visionTargets[visionTarget.first].get()->linearVelocity = generateVector3FromMsg(visionTarget.second.linearVelocity);
            pImpl->visionTargets[visionTarget.first].get()->angularVelocity = generateVector3FromMsg(visionTarget.second.angularVelocity);
            pImpl->visionTargets[visionTarget.first].get()->calibrationWorldToMeasurementWorld = generateTransformFromMsg(visionTarget.second.calibrationWorldToMeasurementWorld);
            pImpl->visionTargets[visionTarget.first].get()->calibrationMeasurementToLink = generateTransformFromMsg(visionTarget.second.calibrationMeasurementToLink);
            pImpl->visionTargets[visionTarget.first].get()->positionScaleFactor = generateVector3FromMsg(visionTarget.second.positionScaleFactor);
        }
    }
}

std::vector<hde::TargetName> VisionTargetsRemapper::getAllTargetsName() const {
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    std::vector<std::string> targetsName;
    for (auto visionTargetEntry : pImpl->visionTargets)
    {
        targetsName.push_back(visionTargetEntry.first);
    }
    return targetsName;
}

std::shared_ptr<hde::VisionTarget> VisionTargetsRemapper::getTarget(const TargetName name) const {
     std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);

    // TODO: what to do if the target name do not exist
    return pImpl->visionTargets[name];
}

