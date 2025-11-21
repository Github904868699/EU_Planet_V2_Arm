#include "icoordcalibratemethod.h"
#include "coordcalibratemethod.h"
#include <QDebug>
#include "imotorcontrol.h"
#include "iplanningmanager.h"

ICoordCalibrateMethod::ICoordCalibrateMethod(QObject *parent) :
    QObject(parent)
{

}

void ICoordCalibrateMethod::calculateResultAndPublishCoord(const SqlCoordinateInfo &info)
{
    RobotPose pose1 = Global::stringToRobotPose(info.point1);
    RobotPose pose2 = Global::stringToRobotPose(info.point2);
    RobotPose pose3 = Global::stringToRobotPose(info.point3);
    TargetPose tPose1 = IPlanningManager::instance()->JointsToPose(World_Frame, pose1.angles);
    TargetPose tPose2 = IPlanningManager::instance()->JointsToPose(World_Frame, pose2.angles);
    TargetPose tPose3 = IPlanningManager::instance()->JointsToPose(World_Frame, pose3.angles);
    IMotorControl::instance()->publishCoordinateSystem(World_Frame,calculateImp(tPose1,tPose2,tPose3),info.coordName.toStdString());
}

ICoordCalibrateMethodFactory::ICoordCalibrateMethodFactory(QObject *parent) :
    QObject(parent)
{

}

ICoordCalibrateMethodFactory *ICoordCalibrateMethodFactory::instance()
{
    static ICoordCalibrateMethodFactory data;
    return &data;
}

ICoordCalibrateMethod *ICoordCalibrateMethodFactory::createMethod(const QString &methodName)
{
    if(Method_xOxy == methodName)
        return new CoordCalibrateMethodByXOXY(this);
    else if(Method_xOxz == methodName)
        return new CoordCalibrateMethodByXOXZ(this);
    else if(Method_yOyz == methodName)
        return new CoordCalibrateMethodByYOYZ(this);
    else if(Method_yOyx == methodName)
        return new CoordCalibrateMethodByYOYX(this);
    else if(Method_zOzx == methodName)
        return new CoordCalibrateMethodByZOZX(this);
    else if(Method_zOzy == methodName)
        return new CoordCalibrateMethodByZOZY(this);
    else
    {
        qDebug()<<"坐标系标定方式不存在!";
        return nullptr;
    }
}
