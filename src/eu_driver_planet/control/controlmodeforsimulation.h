#ifndef CONTROLMODEFORSIMULATION_H
#define CONTROLMODEFORSIMULATION_H

#include "icontrolmode.h"
#include "trajectorythreadforsimulation.h"

class ControlModeForSimulation : public IControlMode
{
    Q_OBJECT
public:
    explicit ControlModeForSimulation(IDriver *driver, QObject *parent = nullptr);
public:
    int readyForExecuteTrajectory() override;
    int executeTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> &gh) override;
    int stopExecuteTrajectory() override;
private:
    TrajectoryThreadForSimulation m_trajectory;
};

#endif // CONTROLMODEFORSIMULATION_H
