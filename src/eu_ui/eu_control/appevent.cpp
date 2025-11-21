#include "appevent.h"
#include "imotorcontrol.h"
#include "icoordinatemanager.h"
#include "iplanningmanager.h"
#include "p_robotcontrol.h"

AppEvent::AppEvent(QObject *parent) : QObject(parent)
{
    connect(this,SIGNAL(errorOccured(QString)),this,SLOT(errorOccuredSlot(QString)));

    connect(IMotorControl::instance(),SIGNAL(motorPlanControlFinished()),this,SLOT(motorPlanControlFinished()));
    connect(IMotorControl::instance(),SIGNAL(userInputPortStateUpdated(uint)),this,SLOT(userInputPortStateUpdated(uint)));
    connect(IMotorControl::instance(),SIGNAL(userOutputPortStateUpdated(uint)),this,SLOT(userOutputPortStateUpdated(uint)));
    connect(IMotorControl::instance(),SIGNAL(motorInitStateUpdated(bool)),this,SLOT(motorInitStateUpdated(bool)));
    connect(IMotorControl::instance(),SIGNAL(motorOffsetUpdated(std::vector<float>)),this,SLOT(motorOffsetUpdated(std::vector<float>)));
    connect(IMotorControl::instance(),SIGNAL(motorBandTypeBrakeUpdated(bool)),this,SLOT(motorBandTypeBrakeUpdated(bool)));

    connect(IPlanningManager::instance(),SIGNAL(planFailed()),this,SLOT(planFailed()));
    connect(IPlanningManager::instance(),SIGNAL(currentPlanFrameChanged(QString)),this,SLOT(currentPlanFrameChanged(QString)));

    connect(ICoordinateManager::instance(),SIGNAL(coordinateSystemInfoUpdated()),this,SLOT(coordinateSystemInfoUpdated()));
}

AppEvent *AppEvent::instance()
{
    static AppEvent data;
    return &data;
}

void AppEvent::motorPlanControlFinished()
{
    if(RobotEventFunc)
        RobotEventFunc(RobotEvent_RobotArriveTargetPos,"");
}

void AppEvent::userInputPortStateUpdated(unsigned state)
{
    if(InputPortStateUpdatedFunc)
        InputPortStateUpdatedFunc(state);
}

void AppEvent::userOutputPortStateUpdated(unsigned state)
{
    if(OutportStateUpdatedFunc)
        OutportStateUpdatedFunc(state);
}

void AppEvent::motorInitStateUpdated(bool enable)
{
    if(RobotEventFunc)
        enable ? RobotEventFunc(RobotEvent_RobotStartupDone,"") : RobotEventFunc(RobotEvent_RobotShutdownDone,"");
}

void AppEvent::motorOffsetUpdated(const std::vector<float> &angles)
{
    if(RobotOffsetUpdatedFunc)
        RobotOffsetUpdatedFunc(angles);
}

void AppEvent::motorBandTypeBrakeUpdated(bool isOpen)
{
    if(RobotEventFunc)
        isOpen ? RobotEventFunc(RobotEvent_RobotBrakeOpenDone,"") : RobotEventFunc(RobotEvent_RobotBrakeCloseDone,"");
}

void AppEvent::errorOccuredSlot(const QString &err)
{
    if(RobotEventFunc)
        RobotEventFunc(RobotEvent_Error,err.toStdString());
}

void AppEvent::planFailed()
{
    if(RobotEventFunc)
        RobotEventFunc(RobotEvent_RobotPlanFailed,"");
}

void AppEvent::currentPlanFrameChanged(const QString &frame)
{
    if(RobotEventFunc)
        RobotEventFunc(RobotEvent_PlanFrameChanged,frame.toStdString());
}

void AppEvent::coordinateSystemInfoUpdated()
{
    if(RobotEventFunc)
        RobotEventFunc(RobotEvent_PlanFrameInfoUpdated,"");
}
