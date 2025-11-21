#ifndef ARCTHREAD_H
#define ARCTHREAD_H

#include <QThread>
#include <QMutex>
#include "global.h"

namespace moveit
{
namespace planning_interface
{
class MoveGroupInterface;
}
}

class ArcThread : public QThread
{
    Q_OBJECT
private:
    struct PT3
    {
        PT3(double c_x,double c_y,double c_z) :
            x(c_x),
            y(c_y),
            z(c_z){}
        double x, y, z;
    };
    explicit ArcThread(QObject *parent = nullptr);
    void Use3PointsComputeNormalVector(std::vector<float> p1, std::vector<float> p2, std::vector<float> p3,std::vector<float> &NormalVector);
    double solveCenterPointOfCircle(std::vector<PT3> pt, double centerpoint[]);
public:
    static ArcThread *instance();    
    void setPara(moveit::planning_interface::MoveGroupInterface *group, const std::vector<TargetPose> &targetPoses,const std::string &planFrame, double velocityFactor, double sampleDuration);
protected:
    virtual void run() override;
signals:
    void planFailed();
private:
    std::vector<TargetPose> m_targetPoses;
    moveit::planning_interface::MoveGroupInterface *m_group;
    double m_velocityFactor;
    double m_sample_duration_;
    std::string m_planFrame;
    QMutex m_mutex;
};

#endif // ARCTHREAD_H
