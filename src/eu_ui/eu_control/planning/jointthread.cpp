#include "jointthread.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "imotorcontrol.h"
#include "appevent.h"

JointThread::JointThread(QObject *parent) : QThread(parent)
{

}

JointThread::~JointThread()
{
    this->wait();
}

JointThread *JointThread::instance()
{
    static JointThread data;
    return &data;
}

void JointThread::setPara(moveit::planning_interface::MoveGroupInterface *group, double velocityFactor, double sampleDuration)
{
    QMutexLocker locker(&m_mutex);
    m_group = group;
    m_velocityFactor = velocityFactor;
    m_sample_duration_ = sampleDuration;
}

void JointThread::run()
{
    QMutexLocker locker(&m_mutex);
    moveit::planning_interface::MoveGroupInterface *group = m_group;
    double velocityFactor = m_velocityFactor;
    double sample_duration_ = m_sample_duration_;
    locker.unlock();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if(!group->plan(plan))
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><joint> 关节轨迹规划失败");
        return;
    }
    IMotorControl::instance()->setPlanningResult(1);                    //通知规划的结果
    moveit_msgs::RobotTrajectory trajectory_temp = plan.trajectory_;    //保存生成轨迹的副本
    Global::scale_trajectory_speed(&trajectory_temp,velocityFactor);    //对轨迹副本进行速度上的调整

    size_t pointCont = trajectory_temp.joint_trajectory.points.size();  //轨迹副本的轨迹个数
    size_t index_in = 0;                                                //参与插补计算的索引
    trajectory_msgs::JointTrajectoryPoint p1, p2, interp_pt;
    plan.trajectory_.joint_trajectory.points.clear();                   //清除规划的轨迹点，下面需要重新插补规划

    double currentInterpolateTime = 0.0;//当前插补点时间
    double totalTime = trajectory_temp.joint_trajectory.points.back().time_from_start.toSec();//轨迹总时间
    while (currentInterpolateTime < totalTime)
    {
        //        找到插补点时间的后一个索引
        while (currentInterpolateTime > trajectory_temp.joint_trajectory.points[index_in + 1].time_from_start.toSec())
        {
            ++index_in;
            if (index_in >= pointCont)
            {
                emit planFailed();
                emit AppEvent::instance()->errorOccured("<planning><joint> 轨迹点错误");
                return;
            }
        }
        p1 = trajectory_temp.joint_trajectory.points[index_in];
        p2 = trajectory_temp.joint_trajectory.points[index_in + 1];
        if (!Global::interpolatePt(p1, p2, currentInterpolateTime, interp_pt))
        {
            emit planFailed();
            emit AppEvent::instance()->errorOccured("<planning><joint> 插补计算失败");
            return;
        }
        plan.trajectory_.joint_trajectory.points.push_back(interp_pt);
        currentInterpolateTime += sample_duration_;
    }

    p2 = trajectory_temp.joint_trajectory.points.back();
    p2.time_from_start = ros::Duration(currentInterpolateTime);
    plan.trajectory_.joint_trajectory.points.push_back(p2);
    while(plan.trajectory_.joint_trajectory.points.size() > MaxTrajectoryNum)
        plan.trajectory_.joint_trajectory.points.pop_back();
    group->asyncExecute(plan);
}
