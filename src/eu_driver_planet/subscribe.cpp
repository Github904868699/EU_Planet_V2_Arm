#include "subscribe.h"
#include "global.h"
#include "controlmodeforactual.h"
#include "controlmodeforsimulation.h"
#include "driverforcan.h"
#include "driverforsocket.h"
#include <QApplication>
#include <QDebug>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

Subscribe::Subscribe(QObject *parent) :
    QThread(parent),
    as_(nh_,Action_FollowJointTrajectory,boost::bind(&Subscribe::delTrajectoryAction,this,_1),boost::bind(&Subscribe::delTrajectoryAction,this,_1),false)
{
    if(!nh_.ok())
    {
        qDebug()<<"ros节点检测失败";
        qApp->exit();
    }
    as_.start();

    m_driver.reset(new DriverForCan);
    m_ctlMode.reset(new ControlModeForSimulation(m_driver.data()));

    m_errorPub = nh_.advertise<std_msgs::String>(Topic_Eu_Error,10);
    m_motorCtlSub = nh_.subscribe(Topic_Eu_RequestCmd, 100, &Subscribe::delCustomTopic, this);
    m_userInputPortStatePub = nh_.advertise<std_msgs::UInt32>(Topic_Eu_UserInputPortState,10);
    m_userOutputPortStatePub = nh_.advertise<std_msgs::UInt32>(Topic_Eu_UserOutputPortState,10);
    m_setUserOutputStateSub = nh_.subscribe(Topic_Eu_SetUserOutputPortState,100, &Subscribe::delSetUserOutputState,this);
    m_offsetPub = nh_.advertise<std_msgs::Float32MultiArray>(Topic_Eu_MotorOffset,10);
    m_setOffsetSub = nh_.subscribe(Topic_Eu_SetMotorOffset,10,&Subscribe::delSetMotorOffset,this);
    m_reponseCmdPub = nh_.advertise<std_msgs::UInt8>(Topic_Eu_ResponseCmd,10);
    m_setGripper = nh_.subscribe(Topic_Eu_SetGripper,10,&Subscribe::delSetGripper,this);
}

void Subscribe::publishError(const QString &error)
{
    std_msgs::String msg;
    msg.data = error.toStdString();
    m_errorPub.publish(msg);
}

void Subscribe::delSetUserOutputState(const std_msgs::UInt32 &msg)
{
    publishErrorIfNeed(static_cast<IDriver::ErrCode>(m_ctlMode.data()->setUserOutputPortsState(msg)));
}

void Subscribe::delSetMotorOffset(const eu_msgs::MotorOffset &msg)
{
    if(IDriver::ErrCode_Success == m_ctlMode.data()->setMotorPositionOffset(msg))
    {
        std_msgs::Float32MultiArray msg;
        m_ctlMode.data()->readMotorPositionOffset(&msg);
        m_offsetPub.publish(msg);
    }
    else
        publishError(m_ctlMode.data()->getLastError());
}

void Subscribe::delCustomTopic(const std_msgs::UInt8 &msg)
{
    if(RequestCmd_Enable == msg.data)
    {
        qDebug()<<"正在使能电机...";
        if(IDriver::ErrCode_Success == m_ctlMode.data()->enableMotors())
        {
            std_msgs::Bool msg;
            msg.data = ResponseCmd_EnterInitState;
            m_reponseCmdPub.publish(msg);
        }
        else
            publishError(m_ctlMode.data()->getLastError());
    }
    else if(RequestCmd_Disable == msg.data)
    {
        qDebug()<<"正在失能电机...";
        if(IDriver::ErrCode_Success == m_ctlMode.data()->disableMotors())
        {
            std_msgs::Bool msg;
            msg.data = ResponseCmd_EnterDisableState;
            m_reponseCmdPub.publish(msg);
        }
        else
            publishError(m_ctlMode.data()->getLastError());
    }
    else if(RequestCmd_Enter == msg.data)
    {
        qDebug()<<"进入预运行状态...";
        publishErrorIfNeed(static_cast<IDriver::ErrCode>(m_ctlMode.data()->readyForExecuteTrajectory()));
    }
    else if(RequestCmd_Stop == msg.data)
    {
        qDebug()<<"停止状态...";
        publishErrorIfNeed(static_cast<IDriver::ErrCode>(m_ctlMode.data()->stopExecuteTrajectory()));
    }
    else if(RequestCmd_Simulation == msg.data)
    {
        qDebug()<<"进入仿真状态...";
        m_ctlMode.reset();
        m_ctlMode.reset(new ControlModeForSimulation(m_driver.data()));
    }
    else if(RequestCmd_ActualArm == msg.data)
    {
        qDebug()<<"正在控制真实机械臂";
        m_ctlMode.reset();
        m_ctlMode.reset(new ControlModeForActual(m_driver.data()));
    }
    else if(RequestCmd_MotorPositionOffset == msg.data)
    {
        qDebug()<<"读取电机偏移值";
        std_msgs::Float32MultiArray msg;
        m_ctlMode.data()->readMotorPositionOffset(&msg);
        m_offsetPub.publish(msg);
    }
    else if(RequestCmd_StartRecordTrack == msg.data)
    {
    }
    else if(RequestCmd_StopRecordTrack == msg.data)
    {
    }
    else if(RequestCmd_OpenBandTypeBrake == msg.data)
    {
        qDebug()<<"打开报闸";
        if(m_ctlMode.data()->setBandTypeBrake(true))
        {
            std_msgs::Bool msg;
            msg.data = ResponseCmd_EnterBandTypeBrakeClosed;
            m_reponseCmdPub.publish(msg);
        }
        else
            publishError(m_ctlMode.data()->getLastError());
    }
    else if(RequestCmd_CloseBandTypeBrake == msg.data)
    {
        qDebug()<<"关闭报闸";
        if(IDriver::ErrCode_Success == m_ctlMode.data()->setBandTypeBrake(false))
        {
            std_msgs::Bool msg;
            msg.data = ResponseCmd_EnterBandTypeBrakeClosed;
            m_reponseCmdPub.publish(msg);
        }
        else
            publishError(m_ctlMode.data()->getLastError());
    }
}

void Subscribe::delTrajectoryAction(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
    publishErrorIfNeed(static_cast<IDriver::ErrCode>(m_ctlMode.data()->executeTrajectory(gh)));
}

void Subscribe::delSetGripper(const std_msgs::Int32 &msg)
{
    publishErrorIfNeed(static_cast<IDriver::ErrCode>(m_ctlMode.data()->setGripper(msg.data)));
}

void Subscribe::publishErrorIfNeed(IDriver::ErrCode errCode)
{
    if(IDriver::ErrCode_Success != errCode)
        publishError(m_ctlMode.data()->getLastError());
}
