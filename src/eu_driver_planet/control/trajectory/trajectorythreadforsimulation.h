#ifndef TRAJECTORYTHREADFORSIMULATION_H
#define TRAJECTORYTHREADFORSIMULATION_H

#include "itrajectorythread.h"

class TrajectoryThreadForSimulation : public ITrajectoryThread
{
    Q_OBJECT
public:
    explicit TrajectoryThreadForSimulation(IDriver *driver, QObject *parent = nullptr);
    ~TrajectoryThreadForSimulation();
protected:
    virtual void run() override;
private:
    ros::Publisher Pub_jint;
};

#endif // TRAJECTORYTHREADFORSIMULATION_H
