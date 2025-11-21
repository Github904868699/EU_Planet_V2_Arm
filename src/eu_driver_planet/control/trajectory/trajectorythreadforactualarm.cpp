#include "trajectorythreadforactualarm.h"
#include "global.h"
#include "idriver.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt32.h>
#include <QDebug>
#include <QEventLoop>
#include <QTimer>
#include <QTime>

TrajectoryThreadForActualArm::TrajectoryThreadForActualArm(IDriver *driver, QObject *parent) : ITrajectoryThread(driver,parent)
{
}

TrajectoryThreadForActualArm::~TrajectoryThreadForActualArm()
{
    qDebug()<<"释放真实轨迹线程";
}

void TrajectoryThreadForActualArm::run()
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


    m_driver->setTargetTrajectory(gh);//④循环发送路径点数据

    QEventLoop q;
    QTimer tT;
    tT.setSingleShot(true);
    QObject::connect(&tT, SIGNAL(timeout()), &q, SLOT(quit()));
    QObject::connect(m_driver,SIGNAL(cacheSendFinished()),&q, SLOT(quit()));
    QObject::connect(this,SIGNAL(closeThreadSignal()),&q, SLOT(quit()));

    int msec = type.points.back().time_from_start.toSec() * 1000 * 20;
    qDebug()<<"等待时间:"<<msec<<"ms";
    tT.start(msec); // 5s timeout
    q.exec();

    if(tT.isActive()){
        // download complete
        tT.stop();
        qDebug()<<"轨迹执行成功";
    } else {
        // timeout
        qDebug()<<"轨迹执行失败";
    }

    result_.result.error_code = result_.result.SUCCESSFUL;
    gh.setSucceeded(result_.result);

    return;
}
