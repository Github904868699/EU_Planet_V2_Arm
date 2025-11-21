#include "robotcontrol.h"
#include "p_robotcontrol.h"
#include "imotorcontrol.h"
#include "iplanningmanager.h"
#include "appevent.h"
#include <QCoreApplication>

static int argc = 1;
static char arg0[] = "";
static char *argv[] = {arg0,nullptr};
Q_GLOBAL_STATIC_WITH_ARGS(QCoreApplication, app, (argc, argv))

int robotSetRobotEventCallback(RobotEventCallback callback)
{
    RobotEventFunc = callback;
    return ROBOT_SUCCESS;
}

int robotSetInputPortStateUpdatedCallback(RoportInputPortStateUpdatedCallback callback)
{
    InputPortStateUpdatedFunc = callback;
    return ROBOT_SUCCESS;
}

int robotSetOutputPortStateUpdatedCallback(RobotOutputPortStateUpdatedCallback callback)
{
    OutportStateUpdatedFunc = callback;
    return ROBOT_SUCCESS;
}

int robotSetRobotOffsetUpdatedCallback(RobotOffsetUpdatedCallback callback)
{
    RobotOffsetUpdatedFunc = callback;
    return ROBOT_SUCCESS;
}

int robotInitDLL()
{
    if(!QCoreApplication::instance())
        app->processEvents();
    IPlanningManager::instance();
    IMotorControl::instance();
    AppEvent::instance();
    return ROBOT_SUCCESS;
}

int robotFreeDLL()
{
    return ROBOT_SUCCESS;
}

int robotStartUp()
{
    IMotorControl::instance()->publishRequest(RequestCmd_Enable);
    return ROBOT_SUCCESS;
}

int robotShutdown()
{
    IMotorControl::instance()->publishRequest(RequestCmd_Disable);
    return ROBOT_SUCCESS;
}

int robotOpenBrake()
{
    IMotorControl::instance()->publishRequest(RequestCmd_OpenBandTypeBrake);
    return ROBOT_SUCCESS;
}

int robotCloseBrake()
{
    IMotorControl::instance()->publishRequest(RequestCmd_CloseBandTypeBrake);
    return ROBOT_SUCCESS;
}

int robotClearOffset(int index)
{
    IMotorControl::instance()->publishSetMotorOffset(OffsetType_ActualValue,index + 1,0);
    return ROBOT_SUCCESS;
}

int robotSetOffset(int index, double offset)
{
    IMotorControl::instance()->publishSetMotorOffset(OffsetType_ToPosition,index + 1,offset);
    return ROBOT_SUCCESS;
}

int robotSetGriper(int angle)
{
    IMotorControl::instance()->publishSetGripper(angle);
    return ROBOT_SUCCESS;
}

int robotSetOutputPortState(unsigned states)
{
    IMotorControl::instance()->publishSetUserOutputState(states);
    return ROBOT_SUCCESS;
}

int robotSetOutputPortState(int index, bool state)
{
    IMotorControl::instance()->publishSetUserOutputState(index,state);
    return ROBOT_SUCCESS;
}

int robotGetWorkingMode(RobotWorkingMode &mode)
{
    mode = (RobotWorkingMode)IPlanningManager::instance()->getControlWay();
    return ROBOT_SUCCESS;
}

int robotSetWorkingMode(RobotWorkingMode mode)
{
    IPlanningManager::instance()->setControlWay((ControlMode)mode);
    return ROBOT_SUCCESS;
}

int robotGetVelocityFactor(double &factor)
{
    factor = IPlanningManager::instance()->getPlanningVelocityScalingFactor();
    return ROBOT_SUCCESS;
}

int robotSetVelocityFactor(double factor)
{
    IPlanningManager::instance()->setPlanningVelocityScalingFactor(factor);
    return ROBOT_SUCCESS;
}

int robotGetPlanningFrame(std::string &frame)
{
    frame = IPlanningManager::instance()->getPlanningFrame();
    return ROBOT_SUCCESS;
}

int robotSetPlanningFrame(const std::string &frame)
{
    IPlanningManager::instance()->setPlanningFrame(frame);
    return ROBOT_SUCCESS;
}

int robotGetToolLength(double &length)
{
    length = IPlanningManager::instance()->getToolLength();
    return ROBOT_SUCCESS;
}

int robotSetToolLength(double &length)
{
    IPlanningManager::instance()->setToolLength(length);
    return ROBOT_SUCCESS;
}

int robotJointMove(const std::vector<double> &angles)
{
    IPlanningManager::instance()->setJointValueTarget(angles);
    return ROBOT_SUCCESS;
}

int robotJointMove(int index, double angle)
{
    IPlanningManager::instance()->setJointValueTarget(index,angle);
    return ROBOT_SUCCESS;
}

int robotJointMove(const RobotTargetPose &pose)
{
    IPlanningManager::instance()->setJointValueTarget(RobotTargetPoseToTargetPose(pose));
    return ROBOT_SUCCESS;
}

int robotLineMove(const std::vector<double> &angles)
{
    IPlanningManager::instance()->trajectoryPlanning(angles);
    return ROBOT_SUCCESS;
}

int robotLineMove(const std::vector<std::vector<double> > &anglesList)
{
    IPlanningManager::instance()->trajectoryPlanning(anglesList);
    return ROBOT_SUCCESS;
}

int robotLineMove(const RobotTargetPose &pose)
{
    IPlanningManager::instance()->trajectoryPlanning(RobotTargetPoseToTargetPose(pose));
    return ROBOT_SUCCESS;
}

int robotLineMove(const std::vector<RobotTargetPose> &poses)
{
    IPlanningManager::instance()->trajectoryPlanning(RobotTargetPoseToTargetPose(poses));
    return ROBOT_SUCCESS;
}

int robotCircleMove(const std::vector<std::vector<double> > &anglesList)
{
    IPlanningManager::instance()->arcPlanning(anglesList);
    return ROBOT_SUCCESS;
}

int robotCircleMove(const std::vector<RobotTargetPose> &poses)
{
    IPlanningManager::instance()->arcPlanning(RobotTargetPoseToTargetPose(poses));
    return ROBOT_SUCCESS;
}

int robotStop()
{
    IPlanningManager::instance()->stop();
    return ROBOT_SUCCESS;
}

int robotGetTerminalPose(RobotTargetPose &pose)
{
    pose = TargetPoseToRobotTargetPose(IPlanningManager::instance()->getCurrentPose());
    return ROBOT_SUCCESS;
}

int robotGetJointAngles(std::vector<double> &angles)
{
    angles = IPlanningManager::instance()->getCurrentJointValues();
    return ROBOT_SUCCESS;
}
