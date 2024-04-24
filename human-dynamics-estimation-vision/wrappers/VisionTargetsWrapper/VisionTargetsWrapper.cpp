
#include "VisionTargetsWrapper.h"
#include <hde/interfaces/IVisionTargets.h>
#include <hde/msgs/VisionTargets.h> //revisar esto

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <unordered_map>

const std::string DeviceName = "VisionTargetsWrapper";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::wrappers;

// =======
// Helpers
// =======

const std::map<hde::KinematicTargetType, hde::msgs::KinematicTargetType> mapKinematicTargetType = {
    {hde::KinematicTargetType::none, hde::msgs::KinematicTargetType::NONE},
    {hde::KinematicTargetType::pose, hde::msgs::KinematicTargetType::POSE},
    {hde::KinematicTargetType::poseAndVelocity, hde::msgs::KinematicTargetType::POSEANDVELOCITY},
    {hde::KinematicTargetType::position, hde::msgs::KinematicTargetType::POSITION},
    {hde::KinematicTargetType::positionAndVelocity, hde::msgs::KinematicTargetType::POSITIONANDVELOCITY},
    {hde::KinematicTargetType::orientation, hde::msgs::KinematicTargetType::ORIENTATION},
    {hde::KinematicTargetType::orientationAndVelocity, hde::msgs::KinematicTargetType::ORIENTATIONANDVELOCITY},
    {hde::KinematicTargetType::gravity, hde::msgs::KinematicTargetType::GRAVITY},
    {hde::KinematicTargetType::floorContact, hde::msgs::KinematicTargetType::FLOORCONTACT},
};

hde::msgs::KinematicTargetType generateMsgKinematicTargetType(const hde::KinematicTargetType kinematicTargetType)
{
    return mapKinematicTargetType.at(kinematicTargetType);
}

hde::msgs::Vector3 generateMsgVector3(const iDynTree::Vector3& input)
{
    return {input[0], input[1], input[2]};
}

hde::msgs::Quaternion generateMsgQuaternion(const iDynTree::Rotation& input)
{
    iDynTree::Vector4 quaternion;
    input.getQuaternion(quaternion);
    return {quaternion[0], {quaternion[1], quaternion[2], quaternion[3]}};
}

hde::msgs::Transform generateMsgTransform(const iDynTree::Transform& input)
{
    return {generateMsgVector3(input.getPosition()), generateMsgQuaternion(input.getRotation())};
}

class VisionTargetsWrapper::impl
{
public:
    hde::interfaces::IVisionTargets* iVisionTargets = nullptr;
    yarp::os::BufferedPort<hde::msgs::VisionTargets> outputPort;

    bool firstRun = true;

    // buffer variables
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::VisionTarget>> visionTargets;
};

VisionTargetsWrapper::VisionTargetsWrapper()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

VisionTargetsWrapper::~VisionTargetsWrapper() {}

bool VisionTargetsWrapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("outputPort") && config.find("outputPort").isString())) {
        yError() << LogPrefix << "outputPort option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    std::string outputPortName = config.find("outputPort").asString();

    // =============
    // OPEN THE PORT
    // =============

    if (!pImpl->outputPort.open(outputPortName)) {
        yError() << LogPrefix << "Failed to open port" << outputPortName;
        return false;
    }

    // ================
    // SETUP THE THREAD
    // ================

    setPeriod(period);

    return true;
}


bool VisionTargetsWrapper::close()
{
    pImpl->outputPort.close();
    return true;
}

void VisionTargetsWrapper::run()
{
    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        std::vector<hde::TargetName> targetsName = pImpl->iVisionTargets->getAllTargetsName();

        for (auto targetName : targetsName )
        {
            pImpl->visionTargets[targetName] = pImpl->iVisionTargets->getTarget(targetName);
        }
    }

    hde::msgs::VisionTargets& data = pImpl->outputPort.prepare();

    for (const auto& visionTargetEntry : pImpl->visionTargets) {
        data.targets[visionTargetEntry.first] = {visionTargetEntry.second.get()->inputName, 
                                                   visionTargetEntry.second.get()->modelLinkName, 
                                                   generateMsgKinematicTargetType(visionTargetEntry.second.get()->targetType),
                                                   visionTargetEntry.second.get()->targetIdx, 
                                                   generateMsgVector3(visionTargetEntry.second.get()->position),
                                                   generateMsgQuaternion(visionTargetEntry.second.get()->rotation),
                                                   generateMsgVector3(visionTargetEntry.second.get()->linearVelocity),
                                                   generateMsgVector3(visionTargetEntry.second.get()->angularVelocity),
                                                   generateMsgTransform(visionTargetEntry.second.get()->calibrationWorldToMeasurementWorld),
                                                   generateMsgTransform(visionTargetEntry.second.get()->calibrationMeasurementToLink),
                                                   generateMsgVector3(visionTargetEntry.second.get()->positionScaleFactor)
                                                   };
    }

    // Stream the data though the port
    pImpl->outputPort.write(true);

    // // Get data from the interface
    // pImpl->CoMPositionInterface = pImpl->iHumanState->getCoMPosition();
    // pImpl->CoMVelocityInterface = pImpl->iHumanState->getCoMVelocity();
    // pImpl->basePositionInterface = pImpl->iHumanState->getBasePosition();
    // pImpl->baseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
    // pImpl->baseVelocity = pImpl->iHumanState->getBaseVelocity();
    // pImpl->jointPositionsInterface = pImpl->iHumanState->getJointPositions();
    // pImpl->jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();
    // pImpl->jointNames = pImpl->iHumanState->getJointNames();
    // pImpl->baseName = pImpl->iHumanState->getBaseName();

    // // Prepare the message
    // hde::msgs::HumanState& humanStateData = pImpl->outputPort.prepare();

    // // Convert the COM position
    // humanStateData.CoMPositionWRTGlobal = {
    //     pImpl->CoMPositionInterface[0], pImpl->CoMPositionInterface[1], pImpl->CoMPositionInterface[2]};

    // // Convert the COM velocity
    // humanStateData.CoMVelocityWRTGlobal = {
    //     pImpl->CoMVelocityInterface[0], pImpl->CoMVelocityInterface[1], pImpl->CoMVelocityInterface[2]};

    // // Convert the base position
    // humanStateData.baseOriginWRTGlobal = {
    //     pImpl->basePositionInterface[0], pImpl->basePositionInterface[1], pImpl->basePositionInterface[2]};

    // // Convert the base orientation
    // humanStateData.baseOrientationWRTGlobal = {
    //     pImpl->baseOrientationInterface[0], {pImpl->baseOrientationInterface[1], pImpl->baseOrientationInterface[2], pImpl->baseOrientationInterface[3]}};

    // // Convert the base velocity
    // humanStateData.baseVelocityWRTGlobal.resize(6);
    // humanStateData.baseVelocityWRTGlobal[0] = pImpl->baseVelocity[0];
    // humanStateData.baseVelocityWRTGlobal[1] = pImpl->baseVelocity[1];
    // humanStateData.baseVelocityWRTGlobal[2] = pImpl->baseVelocity[2];
    // humanStateData.baseVelocityWRTGlobal[3] = pImpl->baseVelocity[3];
    // humanStateData.baseVelocityWRTGlobal[4] = pImpl->baseVelocity[4];
    // humanStateData.baseVelocityWRTGlobal[5] = pImpl->baseVelocity[5];

    // // Convert the joint names
    // humanStateData.jointNames.resize(pImpl->jointPositionsInterface.size());
    // for (unsigned i = 0; i < pImpl->jointPositionsInterface.size(); ++i) {
    //     humanStateData.jointNames[i] = pImpl->jointNames[i];
    // }

    // // Convert the joint positions
    // humanStateData.positions.resize(pImpl->jointPositionsInterface.size());
    // for (unsigned i = 0; i < pImpl->jointPositionsInterface.size(); ++i) {
    //     humanStateData.positions[i] = pImpl->jointPositionsInterface[i];
    // }

    // // Convert the joint velocities
    // humanStateData.velocities.resize(pImpl->jointVelocitiesInterface.size());
    // for (unsigned i = 0; i < pImpl->jointVelocitiesInterface.size(); ++i) {
    //     humanStateData.velocities[i] = pImpl->jointVelocitiesInterface[i];
    // }

    // // Store the name of the base link
    // humanStateData.baseName = pImpl->baseName;

    // // Send the data
    // pImpl->outputPort.write(/*forceStrict=*/true);
}

bool VisionTargetsWrapper::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iVisionTargets || !poly->view(pImpl->iVisionTargets) || !pImpl->iVisionTargets) {
        yError() << LogPrefix << "Failed to view the IVisionTargets interface from the PolyDriver";
        return false;
    }

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop";
        return false;
    }

    yDebug() << LogPrefix << "attach() successful";

    return true;
}

void VisionTargetsWrapper::threadRelease() {}

bool VisionTargetsWrapper::detach()
{
    while (isRunning()) {
        stop();
    }

    while (!pImpl->outputPort.isClosed()) {
        pImpl->outputPort.close();
    }

    pImpl->iVisionTargets = nullptr;

    return true;
}

bool VisionTargetsWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool VisionTargetsWrapper::detachAll()
{
    return detach();
}

