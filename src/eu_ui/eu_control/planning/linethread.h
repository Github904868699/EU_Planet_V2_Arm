#ifndef LINETHREAD_H
#define LINETHREAD_H

#include <QThread>
#include "global.h"
#include <QMutex>

namespace moveit
{
namespace planning_interface
{
class MoveGroupInterface;
}
}

class LineThread : public QThread
{
    Q_OBJECT
private:
    explicit LineThread(QObject *parent = nullptr);
    ~LineThread();
public:
    static LineThread *instance();
    void setPara(moveit::planning_interface::MoveGroupInterface *group, const std::vector<TargetPose> &targetPoses, double velocityFactor, double sampleDuration);
protected:
    virtual void run() override;
signals:
    void planFailed();
private:
    std::vector<TargetPose> m_targetPoses;
    moveit::planning_interface::MoveGroupInterface *m_group;
    double m_velocityFactor;
    double m_sample_duration_;
    QMutex m_mutex;
};

#endif // LINETHREAD_H
