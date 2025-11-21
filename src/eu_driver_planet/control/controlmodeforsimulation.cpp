#include "controlmodeforsimulation.h"
#include "trajectorythreadforsimulation.h"
#include "global.h"

ControlModeForSimulation::ControlModeForSimulation(IDriver *driver, QObject *parent) :
    IControlMode(driver, parent),
    m_trajectory(driver)
{
    for(auto it = JointsIdMap.begin();it != JointsIdMap.end();++it)
        m_driver->setAutomaticReportingInterval(it.value(),0);
}

int ControlModeForSimulation::readyForExecuteTrajectory()
{
    m_trajectory.setQuitstate(false);
    return ErrCode_Success;
}

int ControlModeForSimulation::executeTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> &gh)
{
    gh.setAccepted();
    m_trajectory.setTrajectory(gh);
    m_trajectory.start();
    return ErrCode_Success;
}

int ControlModeForSimulation::stopExecuteTrajectory()
{
    m_driver->stopBufferSend();
    m_trajectory.stop();
    return ErrCode_Success;
}
