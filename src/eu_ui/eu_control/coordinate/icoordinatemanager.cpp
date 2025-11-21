#include "icoordinatemanager.h"
#include <QStandardItemModel>
#include "icoordcalibratemethod.h"
#include <QDebug>
#include "dbcoordinateinfoaccess.h"

ICoordinateManager::ICoordinateManager(QObject *parent) : QObject(parent)
{
    m_model = new QStandardItemModel(this);
    m_model->setHorizontalHeaderItem(0,new QStandardItem("坐标系名称"));
    m_model->setHorizontalHeaderItem(1,new QStandardItem("工具名称"));
    m_model->setHorizontalHeaderItem(2,new QStandardItem("坐标系类型"));
    updateModel();
}

CoordinateInfo ICoordinateManager::fromDbCoordinateInfo(const SqlCoordinateInfo &info) const
{
    CoordinateInfo result;
    result.coordName = info.coordName;
    result.method = info.method;
    result.point1 = info.point1;
    result.point2 = info.point2;
    result.point3 = info.point3;
    return result;
}

SqlCoordinateInfo ICoordinateManager::fromCoordinateInfo(const CoordinateInfo &info) const
{
    SqlCoordinateInfo result;
    result.coordName = info.coordName;
    result.method = info.method;
    result.point1 = info.point1;
    result.point2 = info.point2;
    result.point3 = info.point3;
    return result;
}

ICoordinateManager *ICoordinateManager::instance()
{
    static ICoordinateManager data;
    return &data;
}

QStandardItemModel *ICoordinateManager::model()
{
    return m_model;
}

QStringList ICoordinateManager::listCalibrateMethod()
{
    return QStringList() << Method_xOxy
                         << Method_xOxz
                         << Method_yOyz
                         << Method_yOyx
                         << Method_zOzx
                         << Method_zOzy;
}

bool ICoordinateManager::addCoordinateSystem(const CoordinateInfo &info)
{
    static QStringList InvalidFrames{"base_link",
                                     "base_link",
                                     "base_link1",
                                     "base_link2",
                                     "base_link3",
                                     "base_link4",
                                     "base_link5",
                                     "base_link6"};
    if(InvalidFrames.contains(info.coordName))
    {
        qDebug()<<"坐标系名称非法";
        return false;
    }
    bool result = DbCoordinateInfoAccess::instance()->insert(fromCoordinateInfo(info));
    if(result)
        updateModel();
    return result;
}

bool ICoordinateManager::removeCoordinateSystem(const QString &coordName)
{
    bool result = DbCoordinateInfoAccess::instance()->removeByCoordName(coordName);
    if(result)
        updateModel();
    return result;
}

bool ICoordinateManager::alterCoordinateSystem(const QString &coordName, const CoordinateInfo &info)
{
    bool result = DbCoordinateInfoAccess::instance()->update(coordName,fromCoordinateInfo(info));
    if(result)
        updateModel();
    return result;
}

bool ICoordinateManager::queryCoordinateSystemInfo(const QString &coordName, CoordinateInfo &info)
{
    SqlCoordinateInfo temp;
    bool result = DbCoordinateInfoAccess::instance()->queryByCoordName(coordName,temp);
    info = fromDbCoordinateInfo(temp);
    return result;
}

QStringList ICoordinateManager::listCoordinateSystem()
{
    QList<SqlCoordinateInfo> infoList = DbCoordinateInfoAccess::instance()->queryAll();
    QStringList coordList;
    for(int index = 0;index < infoList.count();++index)
        coordList.append(infoList.at(index).coordName);
    coordList.append(QString::fromStdString(World_Frame));
    coordList.append(QString::fromStdString(Tool_Frame));
    return coordList;
}

void ICoordinateManager::updateModel()
{
    emit coordinateSystemInfoUpdated();
    QList<SqlCoordinateInfo> infoList = DbCoordinateInfoAccess::instance()->queryAll();
    m_model->setRowCount(infoList.count());
    for(int index = 0;index < infoList.count();++index)
    {
        const SqlCoordinateInfo &info = infoList.at(index);
        m_model->setData(m_model->index(index,0),info.coordName);
        m_model->setData(m_model->index(index,1),info.toolName);
        m_model->setData(m_model->index(index,2),info.method);
    }
    for(int index = 0;index < infoList.count();++index)
    {
        const SqlCoordinateInfo &info = infoList.at(index);
        ICoordCalibrateMethodFactory::instance()->createMethod(info.method)->calculateResultAndPublishCoord(info);
    }
}
