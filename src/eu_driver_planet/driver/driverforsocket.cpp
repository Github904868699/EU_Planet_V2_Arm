#include "driverforsocket.h"
#include "eu_planet_address.h"
#include <QTime>
#include "global.h"
#include "inetworkmanager.h"

static const unsigned char m_channel = 0;                                                                                                       //CAN通道
//static float valueToCurrent(long value){return static_cast<float>(value);}                                                                    //写入值转电流值(毫安)
static long currentToValue(float current){return static_cast<long>(current);}                                                                   //电流值(毫安)转写入值
//static float valueToPosition(long value){return value / static_cast<float>(65536.0) * static_cast<float>(360.0);}                             //写入值转位置值(度)
static long positionToValue(float position){return static_cast<long>(position / static_cast<float>(360.0) * static_cast<float>(65536.0));}      //位置值(度)转写入值
//static float valueToVelocity(long value){return value * static_cast<float>(60.0) / static_cast<float>(65536.0);}                              //写入值转速度值(RPM)
static long velocityToValue(float velocity){return static_cast<long>(velocity * static_cast<float>(65536.0) / static_cast<float>(60.0));}       //速度值(RPM)转写入值

DriverForSocket::DriverForSocket(QObject *parent) : IDriver(parent),
    m_delReceive(this)
{
    INetworkManager::instance()->init();
//    connect(INetworkManager::instance(),SIGNAL(dataReceived(QByteArray)),&m_delReceive,SLOT(delReceiveData(QByteArray)),Qt::QueuedConnection);
    qDebug()<<connect(INetworkManager::instance()->m_tcpClient.data(),SIGNAL(dataReceived(QByteArray)),&m_delReceive,SLOT(delReceiveData(QByteArray)));
    qDebug()<<connect(INetworkManager::instance()->m_udpClient.data(),SIGNAL(dataReceived(QByteArray)),&m_delReceive,SLOT(delReceiveData(QByteArray)));
    qDebug()<<connect(INetworkManager::instance()->m_udpClient.data(),SIGNAL(dataReceived(QByteArray)),INetworkManager::instance(),SLOT(test1(QByteArray)));
}

void DriverForSocket::addVerificationCodeAndSend(unsigned char data[17])
{
    int sum = 0;
    for(int index = 2;index < 16;++index)
        sum += data[index];
    data[16] = (sum & 0x000000ff);
    //    qDebug()<<"发送:"<<QTime::currentTime().toString("HH:mm:ss:zzz") + ":" + QByteArray((const char *)(sendData),15).toHex(' ');
    INetworkManager::instance()->writeTcpData(QByteArray(reinterpret_cast<char *>(data),17));
}

void DriverForSocket::addCmd0PrefixAndSend(int id, unsigned char *data, unsigned char len)
{
    QMutexLocker locker(&motorPosMutex);
    dataResult[data[1]][id] = false;
    locker.unlock();

    unsigned char sendData[17] = {0};
    sendData[0] = 0xAA;
    sendData[1] = 0x55;
    sendData[2] = m_channel;
    sendData[3] = 0;
    sendData[4] = 0;
    sendData[5] = (id & 0x0000ff00) >> 8;
    sendData[6] = (id & 0x000000ff);
    sendData[7] = len;
    memcpy(&sendData[8],data,len);
    addVerificationCodeAndSend(sendData);
}

int DriverForSocket::waitForResult(int address, int id, int timeOut)
{
    QTime curTime = QTime::currentTime();
    while(curTime.msecsTo(QTime::currentTime()) <= timeOut)
    {
        if(dataResult[address][id])
            return ErrCode_Success;
    }
    return ErrCode_Failed_UnKnown;
}

void DriverForSocket::readPosition(int id)
{
    unsigned char data0 = Cmd_ReadCommand;
    unsigned char data1 = Reg_R_Position;
    unsigned char data[] = {data0,data1,0,0,0,0};
    addCmd0PrefixAndSend(id,data,6);
}

bool DriverForSocket::getReadPositionResult(int id)
{
    return dataResult[Reg_R_Position][id];
}

float DriverForSocket::getPosition(int id)
{
    return motorPosMap[id] + m_Offsets[static_cast<std::vector<float>::size_type>(id - 1)];
}

int DriverForSocket::getPosition(int id, float *pos, int timeOut)
{
    readPosition(id);
    QTime curTime = QTime::currentTime();
    while(curTime.msecsTo(QTime::currentTime()) <= timeOut)
    {
        if(true == getReadPositionResult(id))
        {
            *pos = getPosition(id);
            return ErrCode_Success;
        }
    }
    return ErrCode_Failed_UnKnown;
}

int DriverForSocket::setAutomaticReportingInterval(int id, int ms, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = 0x4A;
    unsigned char data4 = (ms & 0x0000ff00) >> 8;
    unsigned char data5 = ms & 0x000000ff;
    unsigned char data[] = {data0,data1,0,0,data4,data5};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

int DriverForSocket::setEnabled(int id, bool enable, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = Reg_RW_Enabled;
    unsigned char data2 = static_cast<unsigned char>(enable);
    unsigned char data[] = {data0,data1,0,0,0,data2};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

int DriverForSocket::setMode(int id, int mode, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = Reg_RW_Mode;
    unsigned char data2 = static_cast<unsigned char>(mode);
    unsigned char data[] = {data0,data1,0,0,0,data2};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

int DriverForSocket::setTargetPosition(int id, float position, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = Reg_RW_SetPosition;
    long v1 = positionToValue(position);
    unsigned char data2 = (v1 & 0xff000000) >> 24;
    unsigned char data3 = (v1 & 0x00ff0000) >> 16;
    unsigned char data4 = (v1 & 0x0000ff00) >> 8;
    unsigned char data5 = v1 & 0x000000ff;
    unsigned char data[] = {data0,data1,data2,data3,data4,data5};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

int DriverForSocket::setTargetCurrent(int id, float current, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = Reg_RW_SetCurrent;
    long v1 = currentToValue(current);
    unsigned char data2 = (v1 & 0xff000000) >> 24;
    unsigned char data3 = (v1 & 0x00ff0000) >> 16;
    unsigned char data4 = (v1 & 0x0000ff00) >> 8;
    unsigned char data5 =  v1 & 0x000000ff;
    unsigned char data[] = {data0,data1,data2,data3,data4,data5};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

int DriverForSocket::setTargetVelocity(int id, float velocity, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = Reg_RW_SetVelocity;
    long v1 = velocityToValue(velocity);
    unsigned char data2 = (v1 & 0xff000000) >> 24;
    unsigned char data3 = (v1 & 0x00ff0000) >> 16;
    unsigned char data4 = (v1 & 0x0000ff00) >> 8;
    unsigned char data5 =  v1 & 0x000000ff;
    unsigned char data[] = {data0,data1,data2,data3,data4,data5};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

int DriverForSocket::setTargetAcceleration(int id, float acceleration, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = Reg_RW_SetAcceleration;
    long v1 = velocityToValue(acceleration);
    unsigned char data2 = (v1 & 0xff000000) >> 24;
    unsigned char data3 = (v1 & 0x00ff0000) >> 16;
    unsigned char data4 = (v1 & 0x0000ff00) >> 8;
    unsigned char data5 =  v1 & 0x000000ff;
    unsigned char data[] = {data0,data1,data2,data3,data4,data5};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

int DriverForSocket::setTargetDeceleration(int id, float deceleration, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = Reg_RW_SetDeceleration;
    long v1 = velocityToValue(deceleration);
    unsigned char data2 = (v1 & 0xff000000) >> 24;
    unsigned char data3 = (v1 & 0x00ff0000) >> 16;
    unsigned char data4 = (v1 & 0x0000ff00) >> 8;
    unsigned char data5 =  v1 & 0x000000ff;
    unsigned char data[] = {data0,data1,data2,data3,data4,data5};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

int DriverForSocket::setBandTypeBrakeState(int id, bool state, int timeOut)
{
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = 0x42;
    unsigned char data2 = static_cast<unsigned char>(state);
    unsigned char data[] = {data0,data1,0,0,0,data2};
    addCmd0PrefixAndSend(id,data,6);
    return waitForResult(data1,id,timeOut);
}

void DriverForSocket::setTargetPosition(int id, float position, unsigned char num)
{
    unsigned char sendData[17] = {0};
    sendData[0] = 0xAA;
    sendData[1] = 0x55;
    sendData[2] = m_channel;
    sendData[3] = 0x01;
    sendData[4] = num;
    sendData[5] = (id & 0x0000ff00) >> 8;
    sendData[6] = (id & 0x000000ff);
    sendData[7] = 6;
    sendData[8] = Cmd_QuickWrite;
    sendData[9] = Reg_RW_SetPosition;
    long v1 = positionToValue(position - m_Offsets[static_cast<std::vector<float>::size_type>(id - 1)]);
    sendData[10] = (v1 & 0xff000000) >> 24;
    sendData[11] = (v1 & 0x00ff0000) >> 16;
    sendData[12] = (v1 & 0x0000ff00) >> 8;
    sendData[13] = v1 & 0x000000ff;
    sendData[14] = 0;
    sendData[15] = 0;
    addVerificationCodeAndSend(sendData);
}

int DriverForSocket::setTargetTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
    this->stopBufferSend();             //①停止下位机执行
    this->clearBufferSend();            //②清空下位机路径点缓存
    this->clearBufferSend();            //②清空下位机路径点缓存
    this->clearBufferSend();            //②清空下位机路径点缓存
    this->setInterpolationCycle(static_cast<unsigned char>(SampleDuration * 1000));    //③设置插补周期

    control_msgs::FollowJointTrajectoryGoal::_trajectory_type type = gh.getGoal()->trajectory;
    static unsigned char num = 0;//④循环发送路径点数据
    //    qDebug()<<"开始发送时间"<<QTime::currentTime()<<"轨迹点帧数"<<type.points.size()<<"总帧数"<<type.points.size() * 6;
    QList<QByteArray> sendList;
    for(size_t count = 0;count < type.points.size();++count)
    {
        for(size_t index = 0;index < 6;++index)
        {
            const auto &temp = type.points.at(count);
            if(JointsIdMap.contains(type.joint_names[index]))
            {
                int id = JointsIdMap.value(type.joint_names[index]);
                unsigned char sendData[17] = {0};
                sendData[0] = 0xAA;
                sendData[1] = 0x55;
                sendData[2] = m_channel;
                sendData[3] = 0x01;
                sendData[4] = num;
                sendData[5] = (id & 0x0000ff00) >> 8;
                sendData[6] = (id & 0x000000ff);
                sendData[7] = 6;
                sendData[8] = Cmd_QuickWrite;
                sendData[9] = Reg_RW_SetPosition;
                long v1 = positionToValue(static_cast<float>(Global::radianToAngle(temp.positions.at(index)) - static_cast<double>(m_Offsets[static_cast<std::vector<float>::size_type>(id - 1)])));
                sendData[10] = (v1 & 0xff000000) >> 24;
                sendData[11] = (v1 & 0x00ff0000) >> 16;
                sendData[12] = (v1 & 0x0000ff00) >> 8;
                sendData[13] = v1 & 0x000000ff;
                sendData[14] = 0;
                sendData[15] = 0;
                int sum = 0;
                for(int index = 2;index < 16;++index)
                    sum += sendData[index];
                sendData[16] = (sum & 0x000000ff);
                sendList.append(QByteArray(reinterpret_cast<const char *>(sendData),17));
            }
        }
        ++num;
    }
    if(sendList.count())
        INetworkManager::instance()->writeTcpData(sendList);

    this->startBufferSend();//⑤开始执行
    return ErrCode_Success;
}

void DriverForSocket::setInterpolationCycle(unsigned char ms)
{
    unsigned char sendData[17] = {0};
    sendData[0] = 0xAA;
    sendData[1] = 0x55;
    sendData[2] = m_channel;
    sendData[3] = 0x03;
    sendData[4] = ms;
    addVerificationCodeAndSend(sendData);
}

void DriverForSocket::startBufferSend()
{
    unsigned char sendData[17] = {0};
    sendData[0] = 0xAA;
    sendData[1] = 0x55;
    sendData[2] = m_channel;
    sendData[3] = 0x04;
    addVerificationCodeAndSend(sendData);
}

int DriverForSocket::stopBufferSend()
{
    unsigned char sendData[17] = {0};
    sendData[0] = 0xAA;
    sendData[1] = 0x55;
    sendData[2] = m_channel;
    sendData[3] = 0x05;
    addVerificationCodeAndSend(sendData);
    return ErrCode_Success;
}

void DriverForSocket::clearBufferSend()
{
    unsigned char sendData[17] = {0};
    sendData[0] = 0xAA;
    sendData[1] = 0x55;
    sendData[2] = m_channel;
    sendData[3] = 0x06;
    addVerificationCodeAndSend(sendData);
}

int DriverForSocket::setOutputIOState(unsigned states)
{
    qDebug()<<"设置IO数据:0x"<<QString::number(states,16);
    unsigned char sendData[17] = {0};
    sendData[0] = 0xAA;
    sendData[1] = 0x55;
    sendData[2] = m_channel;
    sendData[3] = 0x09;
    sendData[7] = 0x03;
    sendData[8] = (states & 0x00ff0000) >> 16;
    sendData[9] = (states & 0x0000ff00) >> 8;
    sendData[10] = states & 0x000000ff;
    addVerificationCodeAndSend(sendData);
    return ErrCode_Success;
}

//void DriverForSocket::readBufferCount()
//{
//    unsigned char sendData[17] = {0};
//    sendData[0] = 0xAA;
//    sendData[1] = 0x55;
//    sendData[2] = m_channel;
//    sendData[3] = 0x07;
//    addVerificationCodeAndSend(sendData);
//}
