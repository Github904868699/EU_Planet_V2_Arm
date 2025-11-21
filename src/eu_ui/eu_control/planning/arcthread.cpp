#include "arcthread.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "imotorcontrol.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"
#include "iplanningmanager.h"
#include "appevent.h"

ArcThread::ArcThread(QObject *parent) : QThread(parent)
{

}

ArcThread *ArcThread::instance()
{
    static ArcThread data;
    return &data;
}

void ArcThread::setPara(moveit::planning_interface::MoveGroupInterface *group, const std::vector<TargetPose> &targetPoses, const std::string &planFrame, double velocityFactor, double sampleDuration)
{
    QMutexLocker locker(&m_mutex);
    m_group = group;
    m_targetPoses = targetPoses;
    m_planFrame = planFrame;
    m_velocityFactor = velocityFactor;
    m_sample_duration_ = sampleDuration;
}

void ArcThread::Use3PointsComputeNormalVector(std::vector<float> p1, std::vector<float> p2, std::vector<float> p3,std::vector<float> &NormalVector)
{
    float x1 = p2[0] - p1[0];
    float y1 = p2[1] - p1[1];
    float z1 = p2[2] - p1[2];
    float x2 = p3[0] - p1[0];
    float y2 = p3[1] - p1[1];
    float z2 = p3[2] - p1[2];
    float a = y1 * z2 - y2 * z1;
    float b = z1 * x2 - z2 * x1;
    float c = x1 * y2 - x2 * y1;
    //保证深度方向向外
    if (c < 0)
    {
        a = -a;
        b = -b;
        c = -c;
    }
    //归一化
    float length = sqrt(a * a + b * b + c * c);
    a = a / length;
    b = b / length;
    c = c / length;
    NormalVector.push_back(a);
    NormalVector.push_back(b);
    NormalVector.push_back(c);
}

double ArcThread::solveCenterPointOfCircle(std::vector<PT3> pt, double centerpoint[])
{
    double a1, b1, c1, d1;
    double a2, b2, c2, d2;
    double a3, b3, c3, d3;

    double x1 = pt[0].x, y1 = pt[0].y, z1 = pt[0].z;
    double x2 = pt[1].x, y2 = pt[1].y, z2 = pt[1].z;
    double x3 = pt[2].x, y3 = pt[2].y, z3 = pt[2].z;

    a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2);
    b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2);
    c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
    d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

    a2 = 2 * (x2 - x1);
    b2 = 2 * (y2 - y1);
    c2 = 2 * (z2 - z1);
    d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

    a3 = 2 * (x3 - x1);
    b3 = 2 * (y3 - y1);
    c3 = 2 * (z3 - z1);
    d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

    centerpoint[0] = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1)
            /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    centerpoint[1] =  (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1)
            /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    centerpoint[2] = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1)
            /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);

    return sqrt(std::pow(x1 - centerpoint[0],2) + std::pow(y1 - centerpoint[1],2) + std::pow(z1 - centerpoint[2],2));
}

void ArcThread::run()
{
    QMutexLocker locker(&m_mutex);
    moveit::planning_interface::MoveGroupInterface *group = m_group;
    std::vector<TargetPose> targetPoses = m_targetPoses;
    double velocityFactor = m_velocityFactor;
    double sample_duration_ = m_sample_duration_;
    std::string preFrame = m_planFrame;
    locker.unlock();

    //    将当前末端与world之间的关系，广播为静态坐标
    //    tf2_ros::Buffer buffer;
    //    tf2_ros::TransformListener listener(buffer);
    moveit::core::RobotStatePtr current_state = m_group->getCurrentState(1);
    if(!current_state.get())
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><arc> 机械臂状态获取失败");
        //        QLOG_ERROR() << "<planning><arc> 机械臂状态获取失败";
        return;
    }
    geometry_msgs::Pose p;
    const robot_model::LinkModel* lm = current_state.get()->getLinkModel(m_group->getEndEffectorLink());
    if(!lm)
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><arc> 关节模型获取失败");
        //        QLOG_ERROR() << "<planning><arc> 关节模型获取失败";
        return;
    }
    p = tf2::toMsg(current_state.get()->getGlobalLinkTransform(lm));//    这边默认按world坐标系获得数据
    static tf2_ros::StaticTransformBroadcaster pub;//!静态
    geometry_msgs::TransformStamped tfs;
    tfs.header.seq = 0;
    tfs.header.stamp=ros::Time::now();
    tfs.header.frame_id = World_Frame;
    tfs.child_frame_id = Plan_Frame;
    tfs.transform.translation.x = p.position.x;
    tfs.transform.translation.y = p.position.y;
    tfs.transform.translation.z = p.position.z;
    tfs.transform.rotation = p.orientation;
    pub.sendTransform(tfs);

    if(3 != targetPoses.size())
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><arc> 规划预设点少于3个");
        //        QLOG_ERROR() << "<planning><arc> 规划预设点少于3个";
        return;
    }
    geometry_msgs::Pose targetPose = Global::targetPoseTo_Geometry_msgs_Pose(targetPoses.front());
    std::vector<geometry_msgs::Pose> wayPoints;
    wayPoints.push_back(targetPose);

    std::vector<PT3> pt;
    for(int index = 0;index < 3;++index)
    {
        geometry_msgs::Pose pose = Global::targetPoseTo_Geometry_msgs_Pose(targetPoses.at(index));
        pt.push_back(PT3(pose.position.x,pose.position.y,pose.position.z));
    }
    double centerpoint[3];                                      //圆心坐标
    double radius = solveCenterPointOfCircle(pt,centerpoint);   //园的半径
    std::vector<float> v1;
    geometry_msgs::Pose pose1 = Global::targetPoseTo_Geometry_msgs_Pose(targetPoses.at(0));
    v1.push_back(pose1.position.x);
    v1.push_back(pose1.position.y);
    v1.push_back(pose1.position.z);
    std::vector<float> v2;
    geometry_msgs::Pose pose2 = Global::targetPoseTo_Geometry_msgs_Pose(targetPoses.at(1));
    v2.push_back(pose2.position.x);
    v2.push_back(pose2.position.y);
    v2.push_back(pose2.position.z);
    std::vector<float> v3;
    geometry_msgs::Pose pose3 = Global::targetPoseTo_Geometry_msgs_Pose(targetPoses.at(2));
    v3.push_back(pose3.position.x);
    v3.push_back(pose3.position.y);
    v3.push_back(pose3.position.z);
    std::vector<float> fVector;                         //法向量
    Use3PointsComputeNormalVector(v1,v2,v3,fVector);
    double ax = v1[0] - centerpoint[0];
    double ay = v1[1] - centerpoint[1];
    double az = v1[2] - centerpoint[2];
    float aLen = sqrt(ax * ax + ay * ay + az * az);
    ax = ax / aLen;
    ay = ay / aLen;
    az = az / aLen;
    double bx = ay * fVector[2] - fVector[1] * az;
    double by = -(ax * fVector[2] - fVector[0] * az);
    double bz = ax * fVector[1] - fVector[0] * ay;
    for(double th = 0.0;th <= 6.28;th += 0.004)
    {
        targetPose.position.x = centerpoint[0] + radius * cos(th) * ax + radius * sin(th) * bx;
        targetPose.position.y = centerpoint[1] + radius * cos(th) * ay + radius * sin(th) * by;
        targetPose.position.z = centerpoint[2] + radius * cos(th) * az + radius * sin(th) * bz;
        wayPoints.push_back(targetPose);
    }
    wayPoints.push_back(wayPoints.front());
    moveit_msgs::RobotTrajectory trajectory_temp;
    if(Tool_Frame == preFrame)
        group->setPoseReferenceFrame(Plan_Frame);
    double fraction = group->computeCartesianPath(wayPoints,10000,3,trajectory_temp);
    IMotorControl::instance()->setPlanningResult(fraction);
    if(Tool_Frame == preFrame)
        group->setPoseReferenceFrame(Tool_Frame);
    if(fraction < 0 || fraction > 1.0)
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><arc>笛卡尔空间规划计算失败");
        //        QLOG_INFO() << "<planning><arc> 笛卡尔空间规划计算失败";
        return;
    }
    size_t size_in = trajectory_temp.joint_trajectory.points.size();                                //轨迹点个数

    for(int index = 0;index < size_in;++index)
    {
        size_t size_in = trajectory_temp.joint_trajectory.points.size();                                //轨迹点个数
        double duration_in = trajectory_temp.joint_trajectory.points.back().time_from_start.toSec();    //轨迹总时间
        trajectory_temp.joint_trajectory.points.at(index).time_from_start = ros::Duration(index * (duration_in / size_in));
    }

    Global::scale_trajectory_speed(&trajectory_temp,velocityFactor);
    double duration_in = trajectory_temp.joint_trajectory.points.back().time_from_start.toSec();    //轨迹总时间
    if(duration_in <= 0)
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><arc> 规划后的的轨迹总时间错误");
        //        QLOG_ERROR() << "<planning><arc> 规划后的的轨迹总时间错误";
        //        group->asyncExecute(trajectory_temp);
        return;
    }

    double interpolated_time = 0.0;                                                                 //当前轨迹时间
    size_t index_in = 0;                                                                            //当前轨迹点索引
    trajectory_msgs::JointTrajectoryPoint p1, p2, interp_pt;
    moveit_msgs::RobotTrajectory trajectory = trajectory_temp;
    trajectory.joint_trajectory.points.clear(); //清除轨迹点
    while (interpolated_time < duration_in)
    {
        while (interpolated_time > trajectory_temp.joint_trajectory.points[index_in + 1].time_from_start.toSec())
        {
            index_in++;
            if (index_in >= size_in)
            {
                emit planFailed();
                emit AppEvent::instance()->errorOccured("<planning><arc> 轨迹点错误");
                //                QLOG_ERROR() << "<planning><arc> 轨迹点错误";
                return;
            }
        }
        p1 = trajectory_temp.joint_trajectory.points[index_in];
        p2 = trajectory_temp.joint_trajectory.points[index_in + 1];
        if (!Global::interpolatePt(p1, p2, interpolated_time, interp_pt))
        {
            emit planFailed();
            emit AppEvent::instance()->errorOccured("<planning><arc> 插补计算失败");
            //            QLOG_INFO() << "<planning><arc> 插补计算失败";
            return;
        }
        trajectory.joint_trajectory.points.push_back(interp_pt);
        interpolated_time += sample_duration_;
    }
    p2 = trajectory.joint_trajectory.points.back();
    p2.time_from_start = ros::Duration(interpolated_time);
    trajectory.joint_trajectory.points.push_back(p2);

    //    qDebug()<<"轨迹点个数"<<trajectory.joint_trajectory.points.size();
    while(trajectory.joint_trajectory.points.size() > MaxTrajectoryNum)
        trajectory.joint_trajectory.points.pop_back();
    group->asyncExecute(trajectory);
}
