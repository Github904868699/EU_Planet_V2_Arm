#include "linethread.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "imotorcontrol.h"
#include "appevent.h"

LineThread::LineThread(QObject *parent) : QThread(parent)
{

}

LineThread::~LineThread()
{
    this->wait();
}

LineThread *LineThread::instance()
{
    static LineThread data;
    return &data;
}

void LineThread::setPara(moveit::planning_interface::MoveGroupInterface *group, const std::vector<TargetPose> &targetPoses, double velocityFactor, double sampleDuration)
{
    QMutexLocker locker(&m_mutex);
    m_group = group;
    m_targetPoses = targetPoses;
    m_velocityFactor = velocityFactor;
    m_sample_duration_ = sampleDuration;
}

void LineThread::run()
{
    QMutexLocker locker(&m_mutex);
    moveit::planning_interface::MoveGroupInterface *group = m_group;
    std::vector<TargetPose> targetPoses = m_targetPoses;
    double velocityFactor = m_velocityFactor;
    double sample_duration_ = m_sample_duration_;
    locker.unlock();

    group->getCurrentState();//等待各关节当前状态读取成功
    moveit_msgs::RobotTrajectory trajectory_temp;
    double result = group->computeCartesianPath(Global::targetPoseTo_Geometry_msgs_Pose(targetPoses), 0.001, 3, trajectory_temp);
    if(result < 0 || result > 1.0)
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><line> 笛卡尔空间规划计算失败");
        //        QLOG_ERROR() << "<planning><line> 笛卡尔空间规划计算失败";
        return;
    }
    IMotorControl::instance()->setPlanningResult(result);

    //    group->asyncExecute(trajectory_temp);
    //    return;
    //    应用加速度
    //    robot_trajectory::RobotTrajectory rt(m_group->getRobotModel(), m_group->getName());
    //    rt.setRobotTrajectoryMsg(*m_group->getCurrentState(), trajectory_temp);
    //    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    //    iptp.computeTimeStamps(rt, 1, 0.0001);
    //    rt.getRobotTrajectoryMsg(trajectory_temp);

    //    group->asyncExecute(trajectory_temp);
    //    return;

    Global::scale_trajectory_speed(&trajectory_temp,velocityFactor);
    size_t size_in = trajectory_temp.joint_trajectory.points.size();                                //轨迹点个数
    double duration_in = trajectory_temp.joint_trajectory.points.back().time_from_start.toSec();    //轨迹总时间
    if(duration_in <= 0)
    {
        group->asyncExecute(trajectory_temp);
        return;
    }

    moveit_msgs::RobotTrajectory trajectory = trajectory_temp;
    size_t index_in = 0;                                                                            //当前轨迹点索引
    trajectory_msgs::JointTrajectoryPoint p1, p2, interp_pt;
    trajectory.joint_trajectory.points.clear(); //清除轨迹点

    double interpolated_time = 0.0;                                                                 //当前轨迹时间
    while (interpolated_time < duration_in)
    {
        while (interpolated_time > trajectory_temp.joint_trajectory.points[index_in + 1].time_from_start.toSec())
        {
            index_in++;
            if (index_in >= size_in)
            {
                emit planFailed();
                emit AppEvent::instance()->errorOccured("<planning><line> 轨迹点错误");
                return;
            }
        }
        p1 = trajectory_temp.joint_trajectory.points[index_in];
        p2 = trajectory_temp.joint_trajectory.points[index_in + 1];
        if (!Global::interpolatePt(p1, p2, interpolated_time, interp_pt))
        {
            emit planFailed();
            emit AppEvent::instance()->errorOccured("<planning><line> 插补计算失败");
            return;
        }
        trajectory.joint_trajectory.points.push_back(interp_pt);
        interpolated_time += sample_duration_;
    }
    p2 = trajectory.joint_trajectory.points.back();
    p2.time_from_start = ros::Duration(interpolated_time);
    trajectory.joint_trajectory.points.push_back(p2);
    while(trajectory.joint_trajectory.points.size() > MaxTrajectoryNum)
        trajectory.joint_trajectory.points.pop_back();
    group->asyncExecute(trajectory);
}
