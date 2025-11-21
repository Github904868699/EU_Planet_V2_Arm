#include "dataprocessing.h"
#include "global.h"
#include "driverforcan.h"
#include "eu_planet_address.h"
#include <QDebug>
#include <QMutexLocker>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt32.h>

static float valueToPosition(float value)//写入值转位置值(度)
{
    return value / 65536.f * 360.0f;
}

DataProcessing::DataProcessing(IDriver *driver, QObject *parent) : QObject(parent),
    m_driver(driver)
{
    this->moveToThread(&m_thread);
    m_thread.start();
}

DataProcessing::~DataProcessing()
{
    m_thread.quit();
    m_thread.wait();
}

void DataProcessing::delReceiveData(const QByteArray &recData)
{
    static ros::Publisher m_inputPub = nh_.advertise<std_msgs::UInt32>(Topic_Eu_UserInputPortState,10);
    static ros::Publisher m_outputPub = nh_.advertise<std_msgs::UInt32>(Topic_Eu_UserOutputPortState,10);
    static ros::Publisher Pub_jint = nh_.advertise<sensor_msgs::JointState>(Topic_fakeControllerJointStates,10);

    qDebug()<<"接收到数据:"<<recData.toHex(' ');
    if(17 == recData.size() && 0x55 == static_cast<unsigned char>(recData.at(0)) && 0xaa == static_cast<unsigned char>(recData.at(1)))
    {
        int sum = 0;
        for(int index = 2;index < 16;++index)
            sum += static_cast<unsigned char>(recData.at(index));
        if((sum & 0x000000ff) != static_cast<unsigned char>(recData.at(16)))
        {
            emit errorSignal("驱动节点数据校验失败");
            qDebug()<<"数据校验失败("<<(sum & 0x000000ff)<<"):"<<recData.toHex(' ');
            return;
        }
        //        unsigned char channel = static_cast<unsigned char>(recData.at(2));
        unsigned char cmd = static_cast<unsigned char>(recData.at(3));
        //        unsigned char ser = recData.at(4);
        int id = (static_cast<unsigned char>(recData.at(5)) << 8) | static_cast<unsigned char>(recData.at(6));
        int len = static_cast<unsigned char>(recData.at(7));
        QByteArray data = recData.mid(8,len);
        //        qDebug()<<"数据接收成功 CMD:"<<(int)cmd<<"id:"<<id<<"len:"<<len<<"data:"<<data.toHex(' ');
        //透传指令
        if(0 == cmd)
        {
            //        接收到电机位置数据
            if(6 == len && Cmd_ReadCommandReturn == static_cast<unsigned char>(data.at(0)) && Reg_R_Position == static_cast<unsigned char>(data.at(1)))
            {
                float temp = (static_cast<unsigned char>(data.at(2)) << 24 | static_cast<unsigned char>(data.at(3)) << 16 | static_cast<unsigned char>(data.at(4)) << 8 | static_cast<unsigned char>(data.at(5)));
                float result = valueToPosition(temp);
                QMutexLocker locker(&IDriver::motorPosMutex);
                IDriver::motorPosMap[id] = result;
                //            qDebug()<<"接收到获取位置数据：ID:"<<id<<"位置:"<<value;
            }
            else if(8 == len && 0x08 == static_cast<unsigned char>(data.at(0)) && 0x4A == static_cast<unsigned char>(data.at(1)))
            {
                float temp = (static_cast<unsigned char>(data.at(2)) << 24 | static_cast<unsigned char>(data.at(3)) << 16 | static_cast<unsigned char>(data.at(4)) << 8 | static_cast<unsigned char>(data.at(5)));
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
                    js.position[index] = Global::angleToRadian(static_cast<double>(m_driver->getPosition(JointsIdMap.value(js.name[index]))));
                js.header.stamp = ros::Time::now();
                Pub_jint.publish(js);
            }
            QMutexLocker locker(&IDriver::motorPosMutex);
            IDriver::dataResult[static_cast<unsigned char>(data.at(1))][id] = true;
            locker.unlock();
        }
        //缓存发送成功
        else if(1 == cmd)
        {
            qDebug()<<"缓存发送成功";
            emit m_driver->cacheSendFinished();
        }
        //IO端口指令
        else if(9 == cmd)
        {
            unsigned output = static_cast<unsigned>((static_cast<unsigned char>(data.at(1)) << 8) | static_cast<unsigned char>(data.at(0)));
            unsigned input = static_cast<unsigned>((static_cast<unsigned char>(data.at(3)) << 8) | static_cast<unsigned char>(data.at(2)));
            std_msgs::UInt32 inputMsg;
            inputMsg.data = input;
            m_inputPub.publish(inputMsg);
            std_msgs::UInt32 outputMsg;
            outputMsg.data = output;
            m_outputPub.publish(outputMsg);
        }
    }
    else
    {
        emit errorSignal("驱动节点数据接受异常");
        qDebug()<<"数据异常:"<<recData.toHex(' ');
    }
}
