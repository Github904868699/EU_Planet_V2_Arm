#ifndef DRIVERFORSOCKET_H
#define DRIVERFORSOCKET_H

#include "idriver.h"
#include "dataprocessing.h"

class QSettings;

class DriverForSocket : public IDriver
{
    Q_OBJECT
public:
    explicit DriverForSocket(QObject *parent = nullptr);
    virtual float getPosition(int id) override;
    virtual int getPosition(int id, float *pos, int timeOut = 300) override;
    virtual int setAutomaticReportingInterval(int id, int ms, int timeOut = 300) override;
    virtual int setEnabled(int id, bool enable, int timeOut = 300) override;
    virtual int setMode(int id, int mode, int timeOut = 300) override;
    virtual int setTargetPosition(int id, float position, int timeOut = 300) override;
    virtual int setTargetCurrent(int id, float current, int timeOut = 300) override;
    virtual int setTargetVelocity(int id, float velocity, int timeOut = 300) override;
    virtual int setTargetAcceleration(int id, float acceleration, int timeOut = 300) override;
    virtual int setTargetDeceleration(int id, float deceleration, int timeOut = 300) override;
    virtual int setBandTypeBrakeState(int id, bool state, int timeOut = 300) override;
    virtual int setTargetTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) override;
    virtual int stopBufferSend() override;
    virtual int setOutputIOState(unsigned states) override;
private:
    void setTargetPosition(int id, float position, unsigned char num);
    void setInterpolationCycle(unsigned char ms);   //设置插补周期
    void startBufferSend();                         //开始缓存发送(下位机缓存的路径点开始执行)
    void clearBufferSend();                         //清空缓存发送(清空下位机缓存的路径点)
    void readPosition(int id);                                                  //发送读取电机位置命令
    bool getReadPositionResult(int id);                                         //
    void addVerificationCodeAndSend(unsigned char data[17]);                    //添加校验码并发送
    void addCmd0PrefixAndSend(int id, unsigned char *data, unsigned char len);   //添加cmd0的前缀后缀并发送
    int waitForResult(int address, int id, int timeOut);                       //等待透传命令的结果
private:
    DataProcessing m_delReceive;
};

#endif // DRIVERFORSOCKET_H
