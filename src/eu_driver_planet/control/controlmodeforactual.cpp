#include "controlmodeforactual.h"
#include "trajectorythreadforactualarm.h"
#include "global.h"
#include <QTime>

ControlModeForActual::ControlModeForActual(IDriver *driver, QObject *parent) :
    IControlMode(driver,parent),
    m_trajectory(driver)
{
    for(auto it = JointsIdMap.begin();it != JointsIdMap.end();++it)
    {
        QTime curTime = QTime::currentTime();
        while (curTime.msecsTo(QTime::currentTime()) < 1);
        m_driver->setAutomaticReportingInterval(it.value(),100);
    }
}

int ControlModeForActual::readyForExecuteTrajectory()
{
    m_trajectory.setQuitstate(false);
    return ErrCode_Success;
}

int ControlModeForActual::executeTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> &gh)
{
    gh.setAccepted();
    m_trajectory.setTrajectory(gh);
    m_trajectory.start();
    return ErrCode_Success;
}

int ControlModeForActual::stopExecuteTrajectory()
{
    m_driver->stopBufferSend();
    m_trajectory.stop();
    return ErrCode_Success;
}
