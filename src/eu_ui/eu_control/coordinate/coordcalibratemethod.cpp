#include "coordcalibratemethod.h"
#include "global.h"
#include <QDebug>
#include <Eigen/Dense>
#include <iostream>
#include <tf/tf.h>

CoordCalibrateMethodByXOXY::CoordCalibrateMethodByXOXY(QObject *parent) : ICoordCalibrateMethod(parent)
{

}

TargetPose CoordCalibrateMethodByXOXY::calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3)
{
    Eigen::Vector3d p1{tPose2.x - tPose1.x,tPose2.y - tPose1.y,tPose2.z - tPose1.z};
    Eigen::Vector3d p2{tPose3.x - tPose1.x,tPose3.y - tPose1.y,tPose3.z - tPose1.z};
    Eigen::Vector3d p3 = p1.cross(p2);

    double yaw = atan2(p1.y(), p1.x());
    double pitch = atan2(-p1.z(), sqrt(p1.x() * p1.x() + p1.y() * p1.y()));

    Eigen::Vector3d p4 = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).matrix()
            * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()).matrix()
            * Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX()).matrix()
            * Eigen::Vector3d(0,1,0);

    double roll = std::acos(p3.dot(p4) / (p3.norm() * p4.norm()));

    TargetPose temp = tPose1;
    temp.roll = -90 + Global::radianToAngle(roll);
    temp.pitch = Global::radianToAngle(pitch);
    temp.yaw = Global::radianToAngle(yaw);
//    qDebug()<<"正确的角度"<<temp.roll<<temp.pitch<<temp.yaw;
    return temp;
}

CoordCalibrateMethodByXOXZ::CoordCalibrateMethodByXOXZ(QObject *parent) : ICoordCalibrateMethod(parent)
{

}

TargetPose CoordCalibrateMethodByXOXZ::calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3)
{
    Eigen::Vector3d p1{tPose2.x - tPose1.x,tPose2.y - tPose1.y,tPose2.z - tPose1.z};
    Eigen::Vector3d p2{tPose3.x - tPose1.x,tPose3.y - tPose1.y,tPose3.z - tPose1.z};
    Eigen::Vector3d p3 = p1.cross(p2);

    double yaw = atan2(p1.y(), p1.x());
    double pitch = atan2(-p1.z(), sqrt(p1.x() * p1.x() + p1.y() * p1.y()));

    Eigen::Vector3d p4 = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).matrix()
            * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()).matrix()
            * Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX()).matrix()
            * Eigen::Vector3d(0,1,0);

    double roll = std::acos(p3.dot(p4) / (p3.norm() * p4.norm()));

    TargetPose temp = tPose1;
    temp.roll = 0;
    temp.pitch = Global::radianToAngle(pitch);
    temp.yaw = Global::radianToAngle(yaw);
    qDebug()<<"非正确的角度"<<temp.roll<<temp.pitch<<temp.yaw;
    return temp;
}

CoordCalibrateMethodByYOYZ::CoordCalibrateMethodByYOYZ(QObject *parent) : ICoordCalibrateMethod(parent)
{

}

TargetPose CoordCalibrateMethodByYOYZ::calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3)
{
    Eigen::Vector3d p1{tPose2.x - tPose1.x,tPose2.y - tPose1.y,tPose2.z - tPose1.z};
    Eigen::Vector3d p2{tPose3.x - tPose1.x,tPose3.y - tPose1.y,tPose3.z - tPose1.z};
    double yaw1 = atan2(p1.y(), p1.x());
    double pitch1 = atan2(-p1.z(), sqrt(p1.x() * p1.x() + p1.y() * p1.y()));
    //    double roll1 = atan2(v.y() * cos(yaw) + v.x() * sin(yaw), v.z());

    TargetPose temp = tPose1;
    temp.roll = 0;
    temp.pitch = Global::radianToAngle(pitch1);
    temp.yaw = Global::radianToAngle(yaw1);
    //    qDebug()<<"正确的角度"<<temp.orientation.roll<<temp.orientation.pitch<<temp.orientation.yaw;
    return temp;
}

CoordCalibrateMethodByYOYX::CoordCalibrateMethodByYOYX(QObject *parent) : ICoordCalibrateMethod(parent)
{

}

TargetPose CoordCalibrateMethodByYOYX::calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3)
{
    Eigen::Vector3d p1{tPose2.x - tPose1.x,tPose2.y - tPose1.y,tPose2.z - tPose1.z};
    Eigen::Vector3d p2{tPose3.x - tPose1.x,tPose3.y - tPose1.y,tPose3.z - tPose1.z};
    double yaw1 = atan2(p1.y(), p1.x());
    double pitch1 = atan2(-p1.z(), sqrt(p1.x() * p1.x() + p1.y() * p1.y()));
    double roll1 = atan2(p2.y() * cos(yaw1) + p2.x() * sin(yaw1), p2.z());

    TargetPose temp = tPose1;
    temp.roll = 90 + Global::radianToAngle(roll1);
    temp.pitch = Global::radianToAngle(pitch1);
    temp.yaw = Global::radianToAngle(yaw1);
    //    qDebug()<<"正确的角度"<<temp.orientation.roll<<temp.orientation.pitch<<temp.orientation.yaw;
    return temp;
}

CoordCalibrateMethodByZOZX::CoordCalibrateMethodByZOZX(QObject *parent) : ICoordCalibrateMethod(parent)
{

}

TargetPose CoordCalibrateMethodByZOZX::calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3)
{
    Eigen::Vector3d p1{tPose2.x - tPose1.x,tPose2.y - tPose1.y,tPose2.z - tPose1.z};
    Eigen::Vector3d p2{tPose3.x - tPose1.x,tPose3.y - tPose1.y,tPose3.z - tPose1.z};
    double yaw1 = atan2(p1.y(), p1.x());
    double pitch1 = atan2(-p1.z(), sqrt(p1.x() * p1.x() + p1.y() * p1.y()));
    double roll1 = atan2(p2.y() * cos(yaw1) + p2.x() * sin(yaw1), p2.z());

    TargetPose temp = tPose1;
    temp.roll = 90 + Global::radianToAngle(roll1);
    temp.pitch = Global::radianToAngle(pitch1);
    temp.yaw = Global::radianToAngle(yaw1);
    //    qDebug()<<"正确的角度"<<temp.orientation.roll<<temp.orientation.pitch<<temp.orientation.yaw;
    return temp;
}

CoordCalibrateMethodByZOZY::CoordCalibrateMethodByZOZY(QObject *parent) : ICoordCalibrateMethod(parent)
{

}

TargetPose CoordCalibrateMethodByZOZY::calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3)
{
    Eigen::Vector3d p1{tPose2.x - tPose1.x,tPose2.y - tPose1.y,tPose2.z - tPose1.z};
    Eigen::Vector3d p2{tPose3.x - tPose1.x,tPose3.y - tPose1.y,tPose3.z - tPose1.z};
    Eigen::Vector3d v_norm = p1.normalized();
    Eigen::Vector3d p(v_norm.x(), 0, v_norm.z());
    p.normalize();
    double cos_yaw = p.dot(Eigen::Vector3d::UnitX());
    double sin_yaw = p.dot(Eigen::Vector3d::UnitZ());
    double yaw = atan2(sin_yaw, cos_yaw);
    double cos_pitch = v_norm.dot(p);
    double pitch = atan2(-v_norm.y(), cos_pitch);
    double roll = 0; // 可任意选择
    TargetPose temp = tPose1;
    temp.roll = 90 + Global::radianToAngle(roll);
    temp.pitch = Global::radianToAngle(pitch);
    temp.yaw = Global::radianToAngle(yaw);
    return temp;
}
