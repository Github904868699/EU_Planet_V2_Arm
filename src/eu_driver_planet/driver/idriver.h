#ifndef IDRIVER_H
#define IDRIVER_H

#include <QObject>
#include <QMutex>
#include <QMap>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>

class QSettings;

class IDriver : public QObject
{
    Q_OBJECT
public:
    enum ErrCode
    {
        ErrCode_Success = 0,            //成功
        ErrCode_Failed_UnKnown = 1      //失败
    };
    explicit IDriver(QObject *parent = nullptr);
    std::vector<float> getPositionOffset() const;                                           //获得6个电机的位置偏移
    void setPositionOffset(const std::vector<float> &offsets);                              //设置6个电机的位置偏移
    void setPositionOffset(int id, float offset);                                           //设置1个电机的位置偏移
    virtual float getPosition(int id) = 0;                                                  //获得电机位置
    virtual int getPosition(int id, float *pos, int timeOut = 300) = 0;                     //同步方式获得位置
    virtual int setAutomaticReportingInterval(int id, int ms, int timeOut = 300) = 0;       //设置自动上报周期，0为停止
    virtual int setEnabled(int id, bool enable, int timeOut = 300) = 0;                     //设置电机使能状态
    virtual int setMode(int id, int mode, int timeOut = 300) = 0;                           //设置电机运行模式
    virtual int setTargetPosition(int id, float position, int timeOut = 300) = 0;           //设置电机目标位置
    virtual int setTargetCurrent(int id, float current, int timeOut = 300) = 0;             //设置目标电流值
    virtual int setTargetVelocity(int id, float velocity, int timeOut = 300) = 0;           //设置目标速度值
    virtual int setTargetAcceleration(int id, float acceleration, int timeOut = 300) = 0;   //设置目标加速度值
    virtual int setTargetDeceleration(int id, float deceleration, int timeOut = 300) = 0;   //设置目标减速度值
    virtual int setBandTypeBrakeState(int id, bool state, int timeOut = 300) = 0;           //设置抱闸状态
    virtual int setTargetTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) = 0;//写入一段轨迹
    virtual int stopBufferSend() = 0;                                                       //停止缓存发送(下位机停止执行缓存的路径点)
    virtual int setOutputIOState(unsigned states) = 0;                                      //设置用户输出IO状态
signals:
    void cacheSendFinished();                       //缓存发送成功
    void motorPositionUpdate(const sensor_msgs::JointState &js);
    void inputIOStateUpdate(unsigned value);        //接收到Input信号
    void outputIOStateUpdate(unsigned value);       //接收到Output信号
public:
    //保存读写的数据
    static QMutex motorPosMutex;
    static QMap<int, float> motorPosMap;            //所有电机的当前位置
    static QMap<int, QMap<int, bool>> dataResult;   //保存透传命令的执行状态,第一个int是指令类型，第二个int是电机id，第三个参数是执行状态
    static std::vector<float> m_Offsets;            //保存电机偏移值
private:
    QSettings *m_configIni;
};

#endif // IDRIVER_H
