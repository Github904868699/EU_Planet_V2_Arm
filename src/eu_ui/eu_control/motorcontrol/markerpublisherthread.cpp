#include "markerpublisherthread.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "global.h"

MarkerPublisherThread::MarkerPublisherThread(QObject *parent) : QThread(parent)
{
    this->start();
}

MarkerPublisherThread::~MarkerPublisherThread()
{
    m_isQuit = true;
    this->wait();
}

MarkerPublisherThread *MarkerPublisherThread::instance()
{
    static MarkerPublisherThread data;
    return &data;
}

void MarkerPublisherThread::setPlanFrame(const std::string &planFrame)
{
    QMutexLocker locker(&m_mutex);
    m_planFrame = planFrame;
}

void MarkerPublisherThread::run()
{
    ros::NodeHandle n;
    ros::Publisher m_markerPub = n.advertise<visualization_msgs::Marker>(Topic_Eu_CoordMark, 10);

    //    发布末端marker
    visualization_msgs::Marker marker;
    //    marker.header.frame_id = m_planFram;
    //    marker.header.stamp = ros::Time::now();
    marker.ns = "RadarArray_shapes";// 为这个形状设置命名空间及id 从而创建独一无二的目标 若出现新的目标，其与旧目标有相同的命名空间及id，那么新目标会覆盖旧的目标；
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;
    marker.color.r = 120;
    marker.color.g = 120;
    marker.color.b = 120;
    marker.color.a = static_cast<float>(0.3);
    marker.lifetime = ros::Duration();//存在0.1s，// 永久存在  表示存在多久，ros::Duration();意味着永不删除，但是新的具有相同的ns和id的目标将会覆盖旧的，不想被覆盖（想留下轨迹）的话，可以将id+1；否则，就会显示每次的最新值

    while(!m_isQuit)
    {
        //            发布末端marker
        marker.header.frame_id = m_planFrame;
        marker.header.stamp = ros::Time::now();
        m_markerPub.publish(marker);
        this->msleep(100);
    }
}
