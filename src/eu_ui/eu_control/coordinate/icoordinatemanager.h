#ifndef ICOORDINATEMANAGER_H
#define ICOORDINATEMANAGER_H

#include <QObject>
#include "global.h"

class QStandardItemModel;

struct _SqlCoordinateInfo;
typedef struct _SqlCoordinateInfo SqlCoordinateInfo;

class ICoordinateManager : public QObject
{
    Q_OBJECT
public:
    static ICoordinateManager *instance();
    QStandardItemModel *model();                                                            //坐标系信息列表
    QStringList listCalibrateMethod();                                                      //列出标定方法
    bool addCoordinateSystem(const CoordinateInfo &info);                                   //添加用户坐标系
    bool removeCoordinateSystem(const QString &coordName);                                  //移除用户坐标系
    bool alterCoordinateSystem(const QString &coordName, const CoordinateInfo &info);       //修改用户坐标系
    bool queryCoordinateSystemInfo(const QString &coordName, CoordinateInfo &info);         //查询用户坐标系
    QStringList listCoordinateSystem();                                                     //列出所有用户坐标系
signals:
    void coordinateSystemInfoUpdated();
private:
    explicit ICoordinateManager(QObject *parent = nullptr);
    CoordinateInfo fromDbCoordinateInfo(const SqlCoordinateInfo &info) const;
    SqlCoordinateInfo fromCoordinateInfo(const CoordinateInfo &info) const;
private:
    void updateModel();
private:
    QStandardItemModel *m_model;
};
#endif // ICOORDINATEMANAGER_H
