#ifndef CONTROLMODEFORACTUAL_H
#define CONTROLMODEFORACTUAL_H

#include "icontrolmode.h"
#include "trajectorythreadforactualarm.h"

class ControlModeForActual : public IControlMode
{
    Q_OBJECT
public:
    explicit ControlModeForActual(IDriver *driver, QObject *parent = nullptr);
public:
    int readyForExecuteTrajectory() override;
    int executeTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> &gh) override;
    int stopExecuteTrajectory() override;
private:
    TrajectoryThreadForActualArm m_trajectory;
};

#endif // CONTROLMODEFORACTUAL_H
