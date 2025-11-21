#ifndef JOINTTHREAD_H
#define JOINTTHREAD_H

#include <QThread>
#include <QMutex>

namespace moveit
{
namespace planning_interface
{
class MoveGroupInterface;
}
}

class JointThread : public QThread
{
    Q_OBJECT
public:
    explicit JointThread(QObject *parent = nullptr);
    ~JointThread();
public:
    static JointThread *instance();
    void setPara(moveit::planning_interface::MoveGroupInterface *group, double velocityFactor, double sampleDuration);
protected:
    virtual void run() override;
signals:
    void planFailed();
private:
    moveit::planning_interface::MoveGroupInterface *m_group;
    double m_velocityFactor;
    double m_sample_duration_;
    QMutex m_mutex;
};

#endif // JOINTTHREAD_H
