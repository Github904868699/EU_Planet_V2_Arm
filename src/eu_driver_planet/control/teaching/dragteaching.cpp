//#include "dragteaching.h"
//#include <QTimer>
//#include <QTime>
//#include "sensor_msgs/JointState.h"
//#include "../../eu_ui/appconfig/global.h"
//#include <QDebug>

//DragTeaching::DragTeaching(QObject *parent) : QThread(parent)
//{

//}

////DragTeaching *DragTeaching::instance()
////{
////    static DragTeaching data;
////    return &data;
////}

////void DragTeaching::startRecord()
////{
////    this->start();
////}

////void DragTeaching::stopRecord()
////{
////    this->quit();
////    this->wait();
////}

////void DragTeaching::run()
////{
////    sensor_msgs::JointState state;
////    state.name.resize(6);
////    state.position.resize(6);
////    state.name[0] = Joint_1;
////    state.name[1] = Joint_2;
////    state.name[2] = Joint_3;
////    state.name[3] = Joint_4;
////    state.name[4] = Joint_5;
////    state.name[5] = Joint_6;
////    double startTime = 0; //单位s

////    QTimer timer;
////    timer.setInterval(m_inv);
////    timer.setTimerType(Qt::PreciseTimer);
////    connect(&timer,&QTimer::timeout,[&]{
////        for(int index = 0;index < 6;++index)
////            state.position[index] = Global::angleToRadian(IMotorDriver::instance()->getPosition(JointsIdMap.value(state.name[index])));
////        state.header.stamp = ros::Time().fromSec(startTime);
////        qDebug()<<"示教时间:"<<QTime::currentTime()<<startTime<<state.header.stamp.toSec();
////        startTime += m_inv / 1000.0;
//////        Subscribe::instance()->publishRecord(state);
////    });
////    timer.start();
////    this->exec();
////}

