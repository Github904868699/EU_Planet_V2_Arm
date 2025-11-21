#include "imotorcontrol.h"
#include "MotorOffset.h"
#include "markerpublisherthread.h"
#include "rosthread.h"
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <bitset>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include "p_robotcontrol.h"
#include "appevent.h"

IMotorControl::IMotorControl(QObject *parent) : QObject(parent)
{
    RosThread::instance();

    qRegisterMetaType<std::vector<float>>("std::vector<float>");

    m_euMotorCtlPub = m_node.advertise<std_msgs::UInt8>(Topic_Eu_RequestCmd,10);
    m_setUserOutputStatePub = m_node.advertise<std_msgs::UInt32>(Topic_Eu_SetUserOutputPortState,10);
    m_euTargetRobotModelStatePub = m_node.advertise<moveit_msgs::DisplayRobotState>(Topic_Eu_TargetRobotState,10);
    //    m_pathPub = m_node.advertise<nav_msgs::Path>("trajectory",10, true);
    m_setOffsetPub = m_node.advertise<eu_msgs::MotorOffset>(Topic_Eu_SetMotorOffset,10);
    m_markerPub = m_node.advertise<visualization_msgs::Marker>(Topic_Eu_CoordMark, 10);
    m_PointMarkMap[1] = m_node.advertise<visualization_msgs::Marker>(Topic_Eu_CoordPoint1, 10);
    m_PointMarkMap[2] = m_node.advertise<visualization_msgs::Marker>(Topic_Eu_CoordPoint2, 10);
    m_PointMarkMap[3] = m_node.advertise<visualization_msgs::Marker>(Topic_Eu_CoordPoint3, 10);
    m_setGripper = m_node.advertise<std_msgs::Int32>(Topic_Eu_SetGripper,10);

    m_motorCtlResultSub = m_node.subscribe(Topic_FollowJointTrajectoryResult,100,&IMotorControl::delMotorControlResult,this);
    m_UserInputStateSub = m_node.subscribe(Topic_Eu_UserInputPortState,100,&IMotorControl::delUserInputState,this);
    m_UserOutputStateSub = m_node.subscribe(Topic_Eu_UserOutputPortState,100,&IMotorControl::delUserOutputState,this);
    m_errorSub = m_node.subscribe(Topic_Eu_Error,100,&IMotorControl::delErrorMsg,this);
    m_offSetSub = m_node.subscribe(Topic_Eu_MotorOffset,100,&IMotorControl::delMotorOffset,this);
    m_responseSub = m_node.subscribe(Topic_Eu_ResponseCmd, 100, &IMotorControl::delResponseCmd,this);

    static robot_model_loader::RobotModelLoader loader(RobotDescription);
    if(!loader.getModel()) return;
    m_robotState = robot_state::RobotStatePtr(new robot_state::RobotState(loader.getModel()));
}

void IMotorControl::delMotorControlResult(const control_msgs::FollowJointTrajectoryActionResult &result)
{
    emit this->motorPlanControlFinished();
    if(result.result.SUCCESSFUL == result.result.error_code)
    {
        if(0.99 > m_planResult || 1 < m_planResult )
            emit AppEvent::instance()->errorOccured(QString("<motorcontrol><motorcontrol>进入非法区域!%1").arg(m_planResult));
        //            QLOG_ERROR() << "<motorcontrol><motorcontrol>进入非法区域!" << m_planResult;
    }
}

IMotorControl *IMotorControl::instance()
{
    static IMotorControl data;
    return &data;
}

void IMotorControl::publishPlanningFrame(const std::string &planFram)
{
    MarkerPublisherThread::instance()->setPlanFrame(planFram);
}

void IMotorControl::publishRequest(RequestCmd cmd) const
{
    std_msgs::UInt8 msg;
    msg.data = cmd;
    m_euMotorCtlPub.publish(msg);
}

void IMotorControl::publishRobotTargetState(const std::vector<double> &joints)
{
    m_robotState.get()->setJointGroupPositions(m_robotState.get()->getJointModelGroup(Arm_Group),Global::angleToRadian(joints));
    moveit_msgs::DisplayRobotState msg;
    robot_state::robotStateToRobotStateMsg(*m_robotState,msg.state);
    m_euTargetRobotModelStatePub.publish(msg);
}

void IMotorControl::publishRobotTargetState(const TargetPose &pose)
{
    m_robotState.get()->setFromIK(m_robotState.get()->getJointModelGroup(Arm_Group),Global::targetPoseTo_Geometry_msgs_Pose(pose));
    moveit_msgs::DisplayRobotState msg;
    robot_state::robotStateToRobotStateMsg(*m_robotState,msg.state);
    m_euTargetRobotModelStatePub.publish(msg);
}

//void IMotorControl::publishTrajectory(const nav_msgs::Path &path)
//{
//    m_pathPub.publish(path);
//}

void IMotorControl::publishSetUserOutputState(unsigned states)
{
    std_msgs::UInt32 msg;
    msg.data = states;
    m_setUserOutputStatePub.publish(msg);
}

void IMotorControl::publishSetUserOutputState(int index, bool state)
{
    std::bitset<32> states = m_userOutputPortState;
    states[index] = state;
    std_msgs::UInt32 msg;
    msg.data = states.to_ulong();
    m_setUserOutputStatePub.publish(msg);
}

void IMotorControl::publishSetMotorOffset(OffsetType type, int id, float offSet)
{
    eu_msgs::MotorOffset msg;
    msg.type = type;
    msg.id = id;
    msg.offset = offSet;
    m_setOffsetPub.publish(msg);
}

unsigned IMotorControl::getUserInputState() const
{
    return m_userInputPortState;
}

unsigned IMotorControl::getUserOutputState() const
{
    return m_userOutputPortState;
}

void IMotorControl::publishSetGripper(int value)
{
    std_msgs::Int32 msg;
    msg.data = value;
    m_setGripper.publish(msg);
}

void IMotorControl::setPlanningResult(double result)
{
    m_planResult = result;
}

void IMotorControl::publishCoordinateSystem(const std::string &poseFrame, const TargetPose &pose, const std::string &coordName)
{
    geometry_msgs::Pose p = Global::targetPoseTo_Geometry_msgs_Pose(pose);
    static tf2_ros::StaticTransformBroadcaster pub;//!静态
    geometry_msgs::TransformStamped tfs;
    tfs.header.seq = 0;
    tfs.header.stamp=ros::Time::now();
    tfs.header.frame_id = poseFrame;
    tfs.child_frame_id = coordName;
    tfs.transform.translation.x = p.position.x;
    tfs.transform.translation.y = p.position.y;
    tfs.transform.translation.z = p.position.z;
    tfs.transform.rotation.x = p.orientation.x;
    tfs.transform.rotation.y = p.orientation.y;
    tfs.transform.rotation.z = p.orientation.z;
    tfs.transform.rotation.w = p.orientation.w;
    pub.sendTransform(tfs);
}

void IMotorControl::publishWayPoint(const std::string &poseFrame, int id, const TargetPose &pose)
{
    geometry_msgs::Pose targetPose = Global::targetPoseTo_Geometry_msgs_Pose(pose);

    //    发布末端marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = poseFrame;
    marker.header.stamp = ros::Time::now();
    marker.ns = QString("PointMark%1").arg(id).toStdString();// 为这个形状设置命名空间及id 从而创建独一无二的目标 若出现新的目标，其与旧目标有相同的命名空间及id，那么新目标会覆盖旧的目标；
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = targetPose.position;
    marker.pose.orientation = targetPose.orientation;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = static_cast<float>(1.0);
    marker.lifetime = ros::Duration();//存在0.1s，// 永久存在  表示存在多久，ros::Duration();意味着永不删除，但是新的具有相同的ns和id的目标将会覆盖旧的，不想被覆盖（想留下轨迹）的话，可以将id+1；否则，就会显示每次的最新值
    //            发布末端marker
    m_PointMarkMap[id].publish(marker);
}

void IMotorControl::delResponseCmd(const std_msgs::UInt8 &msg)
{
    if(ResponseCmd_EnterInitState == msg.data)
        emit motorInitStateUpdated(true);
    else if(ResponseCmd_EnterDisableState == msg.data)
        emit motorInitStateUpdated(false);
    else if(ResponseCmd_EnterBandTypeBrakeOpened == msg.data)
        emit motorBandTypeBrakeUpdated(true);
    else if(ResponseCmd_EnterBandTypeBrakeClosed == msg.data)
        emit motorBandTypeBrakeUpdated(false);
}

void IMotorControl::delUserInputState(const std_msgs::UInt32 &msg)
{
    m_userInputPortState = msg.data;
    emit this->userInputPortStateUpdated(msg.data);
}

void IMotorControl::delUserOutputState(const std_msgs::UInt32 &msg)
{
    m_userOutputPortState = msg.data;
    emit this->userOutputPortStateUpdated(msg.data);
}

void IMotorControl::delErrorMsg(const std_msgs::String &msg)
{
    emit AppEvent::instance()->errorOccured("<driver>" + QString::fromStdString(msg.data));
    //    QLOG_ERROR() << "<driver>!" + QString::fromStdString(msg.data);
}

void IMotorControl::delMotorOffset(const std_msgs::Float32MultiArray &msg)
{
    static std::vector<float> m_motorOffset{0,0,0,0,0,0};
    for(int index = 0;index < msg.data.size();++index)
        m_motorOffset[index] = msg.data.at(index);
    emit motorOffsetUpdated(m_motorOffset);
}
