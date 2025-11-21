#ifndef TRAJECTORYTHREADFORACTUALARM_H
#define TRAJECTORYTHREADFORACTUALARM_H

#include "itrajectorythread.h"

class TrajectoryThreadForActualArm : public ITrajectoryThread
{
    Q_OBJECT
public:
    explicit TrajectoryThreadForActualArm(IDriver *driver, QObject *parent = nullptr);
    ~TrajectoryThreadForActualArm();
protected:
    virtual void run() override;
};

#endif // TRAJECTORYTHREADFORACTUALARM_H
