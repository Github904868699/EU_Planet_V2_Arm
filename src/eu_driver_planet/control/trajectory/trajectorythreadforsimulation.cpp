#include "trajectorythreadforsimulation.h"
#include <sensor_msgs/JointState.h>
#include <QDebug>
#include <QMutexLocker>
#include <QTime>
#include "global.h"

TrajectoryThreadForSimulation::TrajectoryThreadForSimulation(IDriver *driver, QObject *parent) : ITrajectoryThread(driver, parent)
{
    Pub_jint = nh_.advertise<sensor_msgs::JointState>(Topic_fakeControllerJointStates,10);
}

TrajectoryThreadForSimulation::~TrajectoryThreadForSimulation()
{
    qDebug()<<"释放仿真轨迹线程";
}

void TrajectoryThreadForSimulation::run()
{
    QMutexLocker locker(&m_mutex);
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh = m_gh;
    locker.unlock();

    control_msgs::FollowJointTrajectoryGoal::_trajectory_type type = gh.getGoal()->trajectory;
    control_msgs::FollowJointTrajectoryActionResult result_;

    if(m_isquit)
    {
        result_.result.error_code = -6;
        gh.setSucceeded(result_.result);
        return;
    }

    if(type.points.size() <= 0 || 6 != type.points.at(0).positions.size())
    {
        result_.result.error_code = result_.result.SUCCESSFUL;
        gh.setSucceeded(result_.result);
        return;
    }

    QTime curTime = QTime::currentTime();
    for(size_t count = 0;count < type.points.size();++count)
    {
        for(size_t index = 0;index < 6;++index)
        {
            const auto &temp = type.points.at(count);
            while(curTime.msecsTo(QTime::currentTime()) < temp.time_from_start.toSec() * 1000.0){}
            if(m_isquit)
            {
                result_.result.error_code = -6;
                gh.setSucceeded(result_.result);
                return;
            }
            //            if(type.joint_names[index] == Joint_1)
            //                qDebug()<<"时间:"<<type.points.at(count).time_from_start.toSec();
            //            发布当前状态
            sensor_msgs::JointState js;
            js.position = temp.positions;
            js.velocity = temp.velocities;
            js.effort = temp.effort;
            js.header.stamp = ros::Time::now();
            js.name = type.joint_names;
            Pub_jint.publish(js);
            //            publisher->setJointState(js);
        }
    }
    //    if(type.points.size() > 0)
    //        qDebug()<<"规划的第一个轨迹点"<<Global::radianToAngle(type.points.front().positions.at(0))
    //               <<Global::radianToAngle(type.points.front().positions.at(1))
    //              <<Global::radianToAngle(type.points.front().positions.at(2))
    //             <<Global::radianToAngle(type.points.front().positions.at(3))
    //            <<Global::radianToAngle(type.points.front().positions.at(4))
    //           <<Global::radianToAngle(type.points.front().positions.at(5));
    //    if(type.points.size() > 0)
    //        qDebug()<<"规划的最后一个轨迹点"<<Global::radianToAngle(type.points.back().positions.at(0))
    //               <<Global::radianToAngle(type.points.back().positions.at(1))
    //              <<Global::radianToAngle(type.points.back().positions.at(2))
    //             <<Global::radianToAngle(type.points.back().positions.at(3))
    //            <<Global::radianToAngle(type.points.back().positions.at(4))
    //           <<Global::radianToAngle(type.points.back().positions.at(5));
    //    result_.result.error_code = result_.result.SUCCESSFUL;
    gh.setSucceeded(result_.result);
    return;
}
