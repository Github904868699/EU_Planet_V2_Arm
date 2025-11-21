#include "p_robotcontrol.h"
#include "iplanningmanager.h"
#include "imotorcontrol.h"
#include "icoordinatemanager.h"

RobotEventCallback RobotEventFunc = nullptr;
RoportInputPortStateUpdatedCallback InputPortStateUpdatedFunc = nullptr;
RobotOutputPortStateUpdatedCallback OutportStateUpdatedFunc = nullptr;
RobotOffsetUpdatedCallback RobotOffsetUpdatedFunc = nullptr;

bool addJointValueTarget(int index, double value)
{
    return IPlanningManager::instance()->addJointValueTarget(index,value);
}

void publishPlanningFrame(const std::string &planFram)
{
    IMotorControl::instance()->publishPlanningFrame(planFram);
}

void publishRobotTargetState(const std::vector<double> &joints)
{
    IMotorControl::instance()->publishRobotTargetState(joints);
}

void publishRobotTargetState(const TargetPose &pose)
{
    IMotorControl::instance()->publishRobotTargetState(pose);
}

void publishCoordinateSystem(const std::string &poseFrame, const TargetPose &pose, const std::string &coordName)
{
    IMotorControl::instance()->publishCoordinateSystem(poseFrame,pose,coordName);
}

void publishWayPoint(const std::string &poseFrame, int id, const TargetPose &pose)
{
    IMotorControl::instance()->publishWayPoint(poseFrame,id,pose);
}

unsigned getUserInputState()
{
    return IMotorControl::instance()->getUserInputState();
}

unsigned getUserOutputState()
{
    return IMotorControl::instance()->getUserOutputState();
}

void setNamedTarget(const QString &name)
{
    IPlanningManager::instance()->setNamedTarget(name);
}

QStandardItemModel *model()
{
    return ICoordinateManager::instance()->model();
}

QStringList listCalibrateMethod()
{
    return ICoordinateManager::instance()->listCalibrateMethod();
}

bool addCoordinateSystem(const CoordinateInfo &info)
{
    return ICoordinateManager::instance()->addCoordinateSystem(info);
}

bool removeCoordinateSystem(const QString &coordName)
{
    return ICoordinateManager::instance()->removeCoordinateSystem(coordName);
}

bool alterCoordinateSystem(const QString &coordName, const CoordinateInfo &info)
{
    return ICoordinateManager::instance()->alterCoordinateSystem(coordName,info);
}

bool queryCoordinateSystemInfo(const QString &coordName, CoordinateInfo &info)
{
    return ICoordinateManager::instance()->queryCoordinateSystemInfo(coordName,info);
}

QStringList listCoordinateSystem()
{
    return ICoordinateManager::instance()->listCoordinateSystem();
}

void publishRequest(RequestCmd cmd)
{
    IMotorControl::instance()->publishRequest(cmd);
}

TargetPose JointsToPose(const std::string &frame, const std::vector<double> &joints)
{
    return IPlanningManager::instance()->JointsToPose(frame,joints);
}

TargetPose JointsToPose(const std::vector<double> &joints)
{
    return IPlanningManager::instance()->JointsToPose(joints);
}

std::vector<TargetPose> JointsToPose(const std::string &frame, const std::vector<std::vector<double> > &jointsList)
{
    return IPlanningManager::instance()->JointsToPose(frame,jointsList);
}

std::vector<TargetPose> JointsToPose(const std::vector<std::vector<double> > &jointsList)
{
    return IPlanningManager::instance()->JointsToPose(jointsList);
}
