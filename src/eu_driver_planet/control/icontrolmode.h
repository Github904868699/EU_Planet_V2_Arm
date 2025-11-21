#ifndef ICONTROLMODE_H
#define ICONTROLMODE_H

#include "itrajectorythread.h"
#include "idriver.h"
#include "../../devel/include/eu_msgs/MotorOffset.h"
#include <QObject>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32MultiArray.h>

class ITrajectoryThread;

class IControlMode : public QObject
{
    Q_OBJECT
public:
    enum ErrCode
    {
        ErrCode_Success = 0,
        ErrCode_Failed_UnKnown = 1
    };
    explicit IControlMode(IDriver *driver, QObject *parent = nullptr);
    virtual int enableMotors();                                             //使能6轴电机
    virtual int disableMotors();                                            //失能6轴电机
    virtual int setBandTypeBrake(bool state);                               //设置6轴报闸状态
    virtual int setUserOutputPortsState(const std_msgs::UInt32 &msg);       //设置用户输出端口状态
    virtual int setGripper(int value);                                      //设置末端夹抓角度
    virtual int readMotorPositionOffset(std_msgs::Float32MultiArray *msg);  //读取电机位置偏置
    virtual int setMotorPositionOffset(const eu_msgs::MotorOffset &msg);    //设置电机位置偏置
    virtual int readyForExecuteTrajectory() = 0;                            //准备好执行轨迹，每次执行轨迹前需要先执行该命令
    virtual int executeTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> &gh) = 0;//开始执行轨迹
    virtual int stopExecuteTrajectory() = 0;                                //停止执行轨迹
    QString getLastError() const {return m_error;}                          //获得最后一次的错误信息
signals:
    void errorOccured(const QString &err);
private:
    QString m_error;
protected:
    IDriver *m_driver;
};

#endif // ICONTROLMODE_H
