#include "driverforcan.h"
#include "eu_planet.h"
#include "eu_planet_address.h"
#include "global.h"
#include <std_msgs/UInt32.h>
#include <sensor_msgs/JointState.h>
#include <QDebug>

static float valueToPosition(float value)//写入值转位置值(度)
{
    return value / 65536.f * 360.0f;
}

void receiveCallback(int id, const unsigned char *data, int size)
{
    static ros::NodeHandle nh_;
    static ros::Publisher m_inputPub = nh_.advertise<std_msgs::UInt32>(Topic_Eu_UserInputPortState,10);
    static ros::Publisher m_outputPub = nh_.advertise<std_msgs::UInt32>(Topic_Eu_UserOutputPortState,10);
    static ros::Publisher Pub_jint = nh_.advertise<sensor_msgs::JointState>(Topic_fakeControllerJointStates,10);

    //    qDebug()<<"接收到数据"<<QByteArray((char *)data,size).toHex(' ');

    if(6 == size && Cmd_ReadCommandReturn == data[0] && Reg_R_Position == data[1])
    {
        float temp = ((data[2] << 24) | data[3] << 16 | data[4] << 8 | data[5]);
        float result = valueToPosition(temp);
        QMutexLocker locker(&IDriver::motorPosMutex);
        IDriver::motorPosMap[id] = result;
    }
    else if(8 == size && 0x08 == data[0] && 0x4A == data[1])
    {
        float temp = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
        float result = valueToPosition(temp);
        QMutexLocker locker(&IDriver::motorPosMutex);
        IDriver::motorPosMap[id] = result;
        sensor_msgs::JointState js;
        js.name.resize(6);
        js.position.resize(6);
        js.name[0] = Joint_1;
        js.name[1] = Joint_2;
        js.name[2] = Joint_3;
        js.name[3] = Joint_4;
        js.name[4] = Joint_5;
        js.name[5] = Joint_6;
        for(size_t index = 0;index < 6;++index)
        {
            int id = JointsIdMap.value(js.name[index]);
            js.position[index] = Global::angleToRadian(static_cast<double>(IDriver::motorPosMap[id] + IDriver::m_Offsets[static_cast<std::vector<float>::size_type>(id - 1)]));
        }
        js.header.stamp = ros::Time::now();
        Pub_jint.publish(js);
    }
    //        //IO端口指令
    //        else if(9 == cmd)
    //        {
    //            unsigned output = static_cast<unsigned>((static_cast<unsigned char>(data.at(1)) << 8) | static_cast<unsigned char>(data.at(0)));
    //            unsigned input = static_cast<unsigned>((static_cast<unsigned char>(data.at(3)) << 8) | static_cast<unsigned char>(data.at(2)));
    //            std_msgs::UInt32 inputMsg;
    //            inputMsg.data = input;
    //            m_inputPub.publish(inputMsg);
    //            std_msgs::UInt32 outputMsg;
    //            outputMsg.data = output;
    //            m_outputPub.publish(outputMsg);
    //        }
}

DriverForCan::DriverForCan(QObject *parent) : IDriver(parent)
{
    m_send = new CanWaySendTrajectory(this,this);
    planet_initDLL(planet_DeviceType_Canable,m_devIndex,0,planet_Baudrate_1000);
    planet_setReceiveCallFunction(receiveCallback);
}

float DriverForCan::getPosition(int id)
{
    return motorPosMap[id] + m_Offsets[static_cast<std::vector<float>::size_type>(id - 1)];
}

int DriverForCan::getPosition(int id, float *pos, int timeOut)
{
    return PLANET_SUCCESS == planet_getPosition(m_devIndex,id,pos,timeOut) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setAutomaticReportingInterval(int id, int ms, int timeOut)
{
    Q_UNUSED(timeOut)
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = 0x4A;
    unsigned char data4 = (ms & 0x0000ff00) >> 8;
    unsigned char data5 = ms & 0x000000ff;
    unsigned char data[] = {data0,data1,0,0,data4,data5};
    return PLANET_SUCCESS == planet_writeData(m_devIndex,id,data,6) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setEnabled(int id, bool enable, int timeOut)
{
    return PLANET_SUCCESS == planet_setEnabled(m_devIndex,id,enable,timeOut) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setMode(int id, int mode, int timeOut)
{
    return PLANET_SUCCESS == planet_setMode(m_devIndex,id,mode,timeOut) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setTargetPosition(int id, float position, int timeOut)
{
    return PLANET_SUCCESS == planet_setTargetPosition(m_devIndex,id,position,timeOut) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setTargetCurrent(int id, float current, int timeOut)
{
    return PLANET_SUCCESS == planet_setTargetCurrent(m_devIndex,id,current,timeOut) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setTargetVelocity(int id, float velocity, int timeOut)
{
    return PLANET_SUCCESS == planet_setTargetVelocity(m_devIndex,id,velocity,timeOut) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setTargetAcceleration(int id, float acceleration, int timeOut)
{
    return PLANET_SUCCESS == planet_setTargetAcceleration(m_devIndex,id,acceleration,timeOut) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setTargetDeceleration(int id, float deceleration, int timeOut)
{
    return PLANET_SUCCESS == planet_setTargetDeceleration(m_devIndex,id,deceleration,timeOut) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setBandTypeBrakeState(int id, bool state, int timeOut)
{
    Q_UNUSED(timeOut);
    unsigned char data0 = Cmd_WriteCommand;
    unsigned char data1 = 0x42;
    unsigned char data2 = static_cast<unsigned char>(state);
    unsigned char data[] = {data0,data1,0,0,0,data2};
    return PLANET_SUCCESS == planet_writeData(m_devIndex,id,data,6) ? ErrCode_Success : ErrCode_Failed_UnKnown;
}

int DriverForCan::setTargetTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
    m_send->startSendTrajectory(gh);
    return ErrCode_Success;
}

int DriverForCan::stopBufferSend()
{
    m_send->stop();
    return ErrCode_Success;
}

int DriverForCan::setOutputIOState(unsigned states)
{
    Q_UNUSED(states)
    return ErrCode_Success;
}
