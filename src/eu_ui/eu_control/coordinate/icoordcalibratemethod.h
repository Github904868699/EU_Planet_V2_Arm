#ifndef ICOORDCALIBRATEMETHOD_H
#define ICOORDCALIBRATEMETHOD_H

#include <QObject>
#include "dbcoordinateinfoaccess.h"
#include "global.h"

class ICoordCalibrateMethod : public QObject
{
    Q_OBJECT
public:
    explicit ICoordCalibrateMethod(QObject *parent = nullptr);
    void calculateResultAndPublishCoord(const SqlCoordinateInfo &info);
protected:
    virtual TargetPose calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3) = 0;
};

class ICoordCalibrateMethodFactory : public QObject
{
    Q_OBJECT
private:
    explicit ICoordCalibrateMethodFactory(QObject *parent = nullptr);
public:
    static ICoordCalibrateMethodFactory *instance();
    ICoordCalibrateMethod *createMethod(const QString &methodName);
};

#endif // ICOORDCALIBRATEMETHOD_H
