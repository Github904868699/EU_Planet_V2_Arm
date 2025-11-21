#include "dbcoordinateinfoaccess.h"
#include <QFile>
#include <QCoreApplication>
#include "qdbutil.h"
#include <QVariantMap>
#include <QDir>
#include "appevent.h"

static const QString CoordName = "coordName";
static const QString ToolName = "toolName";
static const QString Method = "method";
static const QString Point1 = "point1";
static const QString Point2 = "point2";
static const QString Point3 = "point3";

DbCoordinateInfoAccess::DbCoordinateInfoAccess(QObject *parent) : QObject(parent)
{
    QDir dir(QCoreApplication::applicationDirPath() + "/database");
    if (!dir.exists())
    {
        dir.mkpath(".");
    }

    QFile fp(QCoreApplication::applicationDirPath() + "/database/coordinate.db");
    bool isExist = fp.exists();
    m_db = new QDBUtil(this);
    m_db->init(QCoreApplication::applicationDirPath() + "/database/coordinate.db","CoordinateInfo");
    if(!isExist)
    {
        QString cmd = "CREATE TABLE CoordinateInfo ("
                      "coordName              VARCHAR (20)  NOT NULL,"
                      "toolName               VARCHAR (32)  NOT NULL,"
                      "method VARCHAR (50)    NOT NULL,"
                      "point1                 VARCHAR (256) NOT NULL,"
                      "point2                 VARCHAR (256) NOT NULL,"
                      "point3                 VARCHAR (256) NOT NULL,"
                      "PRIMARY KEY (coordName));";
        m_db->update(cmd);
    }
}

QVariantMap DbCoordinateInfoAccess::fromEntityToDataSet(const SqlCoordinateInfo &sqlCoordinateInfo) const
{
    QVariantMap valueMap;
    valueMap[CoordName] = QVariant::fromValue<QString>(sqlCoordinateInfo.coordName);
    valueMap[ToolName] = QVariant::fromValue<QString>(sqlCoordinateInfo.toolName);
    valueMap[Method] = QVariant::fromValue<QString>(sqlCoordinateInfo.method);
    valueMap[Point1] = QVariant::fromValue<QString>(sqlCoordinateInfo.point1);
    valueMap[Point2] = QVariant::fromValue<QString>(sqlCoordinateInfo.point2);
    valueMap[Point3] = QVariant::fromValue<QString>(sqlCoordinateInfo.point3);
    return valueMap;
}

SqlCoordinateInfo DbCoordinateInfoAccess::fromDataSetToEntity(const QVariantMap &valueMap) const
{
    SqlCoordinateInfo info;
    info.coordName = valueMap[CoordName].value<QString>();
    info.toolName = valueMap[ToolName].value<QString>();
    info.method = valueMap[Method].value<QString>();
    info.point1 = valueMap[Point1].value<QString>();
    info.point2 = valueMap[Point2].value<QString>();
    info.point3 = valueMap[Point3].value<QString>();
    return info;
}

QList<SqlCoordinateInfo> DbCoordinateInfoAccess::fromDataSetToEntity(const QList<QVariantMap> &valueMapList) const
{
    QList<SqlCoordinateInfo> infoList;
    for(int index = 0;index < valueMapList.count();++index)
        infoList << fromDataSetToEntity(valueMapList.at(index));
    return infoList;
}

QString DbCoordinateInfoAccess::getInsertCmd(const SqlCoordinateInfo &info) const
{
    QVariantMap valueMap = fromEntityToDataSet(info);
    QStringList keyList,valueList;
    for(QVariantMap::Iterator it = valueMap.begin();it != valueMap.end();it++)
    {
        keyList << it.key();
        valueList << QString("'%1'").arg(it.value().toString());
    }
    return QString("insert into CoordinateInfo (%1) values (%2)").arg(keyList.join(",")).arg(valueList.join(","));
}

QString DbCoordinateInfoAccess::getUpdateCmd(const QString &coordName, const SqlCoordinateInfo &info) const
{
    QVariantMap valueMap = fromEntityToDataSet(info);
    QStringList setList;
    for(QVariantMap::iterator it = valueMap.begin();it != valueMap.end();it++)
        setList << QString("%1 = '%2'").arg(it.key()).arg(it.value().toString());
    return QString("update CoordinateInfo set %1 where coordName = '%2'").arg(setList.join(",")).arg(coordName);
}

DbCoordinateInfoAccess *DbCoordinateInfoAccess::instance()
{
    static DbCoordinateInfoAccess data;
    return &data;
}

bool DbCoordinateInfoAccess::insert(const SqlCoordinateInfo &info)
{
    return m_db->update(getInsertCmd(info));
}

bool DbCoordinateInfoAccess::update(const QString &coordName, const SqlCoordinateInfo &info)
{
    return m_db->update(getUpdateCmd(coordName, info));
}

bool DbCoordinateInfoAccess::removeByCoordName(const QString &coordName)
{
    QString cmd = QString("delete from CoordinateInfo where coordName = '%1'").arg(coordName);
    return m_db->update(cmd);
}

bool DbCoordinateInfoAccess::queryByCoordName(const QString &coordName, SqlCoordinateInfo &info)
{
    QString cmd = QString("select * from CoordinateInfo where coordName = '%1'").arg(coordName);
    QVariantMap temp;
    bool result = m_db->queryOne(cmd,temp);
    if(result)
        info = fromDataSetToEntity(temp);
    return result;
}

QList<SqlCoordinateInfo> DbCoordinateInfoAccess::queryAll()
{
    QString cmd = QString("select * from CoordinateInfo");
    QList<QVariantMap> temp;
    QList<SqlCoordinateInfo> infoList;
    bool result = m_db->queryAll(cmd,temp);
    if(result)
        infoList = fromDataSetToEntity(temp);
    return infoList;
}
