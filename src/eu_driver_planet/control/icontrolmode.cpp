#include "icontrolmode.h"
#include "driverforcan.h"
#include "driverforsocket.h"
#include "global.h"
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

IControlMode::IControlMode(IDriver *driver, QObject *parent) :
    QObject(parent),
    m_driver(driver)
{
}

int IControlMode::enableMotors()
{
    for(int id = 1;id <= 6;++id)
    {
        if(IDriver::ErrCode_Success != m_driver->setMode(id,5)
                || IDriver::ErrCode_Success != m_driver->setEnabled(id,true)
                || IDriver::ErrCode_Success != m_driver->setTargetVelocity(id,80)
                || IDriver::ErrCode_Success != m_driver->setTargetAcceleration(id,80)
                || IDriver::ErrCode_Success != m_driver->setTargetDeceleration(id,80)
                /*|| IDriver::ErrCode_Success != m_driver->setTargetCurrent(id,3000)*/)
        {
            //            emit errorOccured(QString("电机使能失败(id:%1)").arg(id));
            m_error = QString("电机使能失败(id:%1)").arg(id);
            return ErrCode_Failed_UnKnown;
        }
        m_driver->setTargetCurrent(id,3000);//!这句返回值是5，所以一直判定失败，移动到这里执行
    }
    m_driver->setEnabled(7,true,0);
    return ErrCode_Success;
}

int IControlMode::disableMotors()
{
    for(int id = 1;id <= 6;++id)
    {
        if(IDriver::ErrCode_Success != m_driver->setEnabled(id,false))
        {
            m_error = QString("电机失能失败(id:%1)").arg(id);
            return ErrCode_Failed_UnKnown;
        }
    }
    m_driver->setEnabled(7,false,0);
    return ErrCode_Success;
}

int IControlMode::setBandTypeBrake(bool state)
{
    for(int id = 1;id <= 6;++id)
    {
        if(IDriver::ErrCode_Success != m_driver->setBandTypeBrakeState(id,state))
        {
            m_error = QString("设置抱闸状态失败(id:%1)").arg(id);
            return ErrCode_Failed_UnKnown;
        }
    }
    return ErrCode_Success;
}

int IControlMode::setUserOutputPortsState(const std_msgs::UInt32 &msg)
{
    m_driver->setOutputIOState(msg.data);
    return ErrCode_Success;
}

int IControlMode::readMotorPositionOffset(std_msgs::Float32MultiArray *msg)
{
    std::vector<float> offsets = m_driver->getPositionOffset();
    for(std::vector<float>::size_type index = 0;index < offsets.size();++index)
        msg->data.push_back(offsets.at(index));
    return ErrCode_Success;
}

int IControlMode::setMotorPositionOffset(const eu_msgs::MotorOffset &msg)
{
    switch(msg.type)
    {
    case OffsetType_ActualValue:
    {
        m_driver->setPositionOffset(msg.id,0);
        return ErrCode_Success;
    }
    case OffsetType_ToPosition:
    {
        float temp = m_driver->getPositionOffset().at(msg.id - 1);

        float pos = 0;
        if(IDriver::ErrCode_Success != m_driver->getPosition(msg.id,&pos))
        {
            m_error = QString("设置位置偏移失败!(获取电机位置失败，id:%1)").arg(msg.id);
            return ErrCode_Failed_UnKnown;
        }
        pos -= temp;
        m_driver->setPositionOffset(msg.id,msg.offset - pos);
        return IDriver::ErrCode_Success;
    }
    }
    return ErrCode_Failed_UnKnown;
}

int IControlMode::setGripper(int value)
{
    m_driver->setMode(7,1);
    m_driver->setTargetVelocity(7,80);
    m_driver->setTargetAcceleration(7,1000);
    m_driver->setTargetDeceleration(7,1000);
    m_driver->setTargetCurrent(7,3000);
    return m_driver->setTargetPosition(7,value);
}
