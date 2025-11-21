#ifndef CANWAYSENDTRAJECTORY_H
#define CANWAYSENDTRAJECTORY_H

#include <QObject>
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class IDriver;

class CanWaySendTrajectory : public QObject
{
    Q_OBJECT
public:
    explicit CanWaySendTrajectory(IDriver *driver,QObject *parent = nullptr);
    void startSendTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
    void stop();
private:
    pthread_t thread;
};

#endif // CANWAYSENDTRAJECTORY_H
