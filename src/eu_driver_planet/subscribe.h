#ifndef SUBSCRIBE_H
#define SUBSCRIBE_H

#include "icontrolmode.h"
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <QThread>
#include <QScopedPointer>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>

class Subscribe : public QThread
{
    Q_OBJECT
public:
    Subscribe(QObject *parent = nullptr);
private:
    void publishError(const QString &error);                    //发布错误信息
    void delSetUserOutputState(const std_msgs::UInt32 &msg);    //设置output端口状态
    void delSetMotorOffset(const eu_msgs::MotorOffset &msg);    //设置电机偏移值
    void delCustomTopic(const std_msgs::UInt8 &msg);            //自定义指令
    void delTrajectoryAction(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);//运行一段轨迹
    void delSetGripper(const std_msgs::Int32 &msg);             //设置夹爪
    void publishErrorIfNeed(IDriver::ErrCode errCode);          //判断是否需要发送错误，如果需要的话
private:
    ros::NodeHandle nh_;
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;//订阅运动轨迹
    ros::Publisher m_errorPub;                  //发布错误信息
    ros::Publisher m_userInputPortStatePub;     //用户输入端口状态发布
    ros::Publisher m_userOutputPortStatePub;    //用户输出端口状态发布
    ros::Publisher m_offsetPub;                 //发布电机的当前偏移值
    ros::Publisher m_reponseCmdPub;             //电机使能成功状态发布
    ros::Subscriber m_motorCtlSub;              //订阅电机控制命令（自定义话题）
    ros::Subscriber m_setUserOutputStateSub;    //订阅用户设置的输出状态
    ros::Subscriber m_setOffsetSub;             //订阅设置电机偏移值
    ros::Subscriber m_setGripper;               //订阅设置夹爪

    QScopedPointer<IDriver> m_driver;           //当前驱动方式，socket驱动还是can驱动
    QScopedPointer<IControlMode> m_ctlMode;     //当前控制模式，真实机械臂还是仿真模式
};

#endif // SUBSCRIBE_H
