#ifndef COORDCALIBRATEMETHOD_H
#define COORDCALIBRATEMETHOD_H

#include "icoordcalibratemethod.h"

class CoordCalibrateMethodByXOXY : public ICoordCalibrateMethod
{
    Q_OBJECT
public:
    explicit CoordCalibrateMethodByXOXY(QObject *parent = nullptr);
protected:
    virtual TargetPose calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3);
};

class CoordCalibrateMethodByXOXZ : public ICoordCalibrateMethod
{
    Q_OBJECT
public:
    explicit CoordCalibrateMethodByXOXZ(QObject *parent = nullptr);
protected:
    virtual TargetPose calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3);
};

class CoordCalibrateMethodByYOYZ : public ICoordCalibrateMethod
{
    Q_OBJECT
public:
    explicit CoordCalibrateMethodByYOYZ(QObject *parent = nullptr);
protected:
    virtual TargetPose calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3);
};

class CoordCalibrateMethodByYOYX : public ICoordCalibrateMethod
{
    Q_OBJECT
public:
    explicit CoordCalibrateMethodByYOYX(QObject *parent = nullptr);
protected:
    virtual TargetPose calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3);
};

class CoordCalibrateMethodByZOZX : public ICoordCalibrateMethod
{
    Q_OBJECT
public:
    explicit CoordCalibrateMethodByZOZX(QObject *parent = nullptr);
protected:
    virtual TargetPose calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3);
};

class CoordCalibrateMethodByZOZY : public ICoordCalibrateMethod
{
    Q_OBJECT
public:
    explicit CoordCalibrateMethodByZOZY(QObject *parent = nullptr);
protected:
    virtual TargetPose calculateImp(const TargetPose &tPose1, const TargetPose &tPose2, const TargetPose &tPose3);
};
#endif // COORDCALIBRATEMETHOD_H
