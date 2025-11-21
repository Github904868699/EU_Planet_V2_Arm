#ifndef ITRAJECTORYTHREAD_H
#define ITRAJECTORYTHREAD_H

#include <QThread>
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "ros/ros.h"
#include <QMutex>
#include "actionlib/server/action_server.h"
#include "actionlib/server/simple_action_server.h"

class StatePublisher;
class IDriver;

class ITrajectoryThread : public QThread
{
    Q_OBJECT
public:
    explicit ITrajectoryThread(IDriver *driver, QObject *parent = nullptr);
    virtual ~ITrajectoryThread();
public:
    void setTrajectory(const actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> &gh);
    void setQuitstate(bool state);
    void stop();
signals:
    void closeThreadSignal();
protected:
    ros::NodeHandle nh_;
    QMutex m_mutex;
    bool m_isquit = false;
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> m_gh;
    IDriver *m_driver;
};

#endif // ITRAJECTORYTHREAD_H
