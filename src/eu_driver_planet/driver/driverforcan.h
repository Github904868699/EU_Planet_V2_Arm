#ifndef DRIVERFORCAN_H
#define DRIVERFORCAN_H

#include "idriver.h"
#include "canwaysendtrajectory.h"

class DriverForCan : public IDriver
{
    Q_OBJECT
public:
    explicit DriverForCan(QObject *parent = nullptr);
public:
    float getPosition(int id) override;
    int getPosition(int id, float *pos, int timeOut) override;
    int setAutomaticReportingInterval(int id, int ms, int timeOut) override;
    int setEnabled(int id, bool enable, int timeOut) override;
    int setMode(int id, int mode, int timeOut) override;
    int setTargetPosition(int id, float position, int timeOut) override;
    int setTargetCurrent(int id, float current, int timeOut) override;
    int setTargetVelocity(int id, float velocity, int timeOut) override;
    int setTargetAcceleration(int id, float acceleration, int timeOut) override;
    int setTargetDeceleration(int id, float deceleration, int timeOut) override;
    int setBandTypeBrakeState(int id, bool state, int timeOut) override;
    int setTargetTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) override;
    int stopBufferSend() override;
    int setOutputIOState(unsigned states) override;
private:
    CanWaySendTrajectory *m_send;
    static const int m_devIndex = 0;
};

#endif // DRIVERFORCAN_H
