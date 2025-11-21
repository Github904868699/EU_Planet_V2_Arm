#include "rosthread.h"
#include "ros/ros.h"

RosThread::RosThread(QObject *parent) :
    QThread(parent)
{
    this->start();
}

RosThread::~RosThread()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    this->wait();
}

RosThread *RosThread::instance()
{
    static RosThread data;
    return &data;
}

void RosThread::run()
{
    while(ros::ok())
    {
        ros::Rate loop_rate(10);
        loop_rate.sleep();
        ros::spinOnce();
    }
}
