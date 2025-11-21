#include "itrajectorythread.h"
#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QMutexLocker>

ITrajectoryThread::ITrajectoryThread(IDriver *driver, QObject *parent) :
    QThread(parent),
    m_driver(driver)
{

}

ITrajectoryThread::~ITrajectoryThread()
{
    setQuitstate(true);
    this->quit();
    this->wait();
}

void ITrajectoryThread::setTrajectory(const actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> &gh)
{
    QMutexLocker locker(&m_mutex);
    m_gh = gh;
}

void ITrajectoryThread::setQuitstate(bool state)
{
    m_isquit = state;
}

void ITrajectoryThread::stop()
{
    emit closeThreadSignal();
    this->setQuitstate(true);
    this->wait();
}
