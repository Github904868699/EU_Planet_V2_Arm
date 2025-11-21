#include "qdbutil.h"
#include <QSqlError>
#include <QSqlRecord>
#include <QSqlField>
#include <QDebug>
#include "appevent.h"

QDBUtil::QDBUtil(QObject *parent) :
    QObject(parent)
{

}

QDBUtil::~QDBUtil()
{
    if(m_db.isOpen())
        m_db.close();
    m_db = QSqlDatabase();//必须添加这一行，提前释放db，使引用计数清除，不然后面的移除连接会出现警告
    if(m_db.contains(m_db.connectionName()))
        m_db.removeDatabase(m_db.connectionName());
}

bool QDBUtil::init(const QString &dbName, const QString &connectName)
{
    if(m_db.contains(connectName)) //这里直接将文件名作为连接名
        m_db = QSqlDatabase::database(connectName);
    else
        m_db = QSqlDatabase::addDatabase("QSQLITE",connectName);
    m_db.setDatabaseName(dbName);
    if(!m_db.open())
    {
        emit AppEvent::instance()->errorOccured("<database><dbutil>数据库文件打开失败!");
        //        QLOG_ERROR() << "<database><dbutil>数据库文件打开失败!";
        return false;
    }
    m_sqlQuery = QSqlQuery(m_db);
    return true;
}

bool QDBUtil::queryOne(const QString &cmd, QVariantMap &resultMap)
{
    if(!m_sqlQuery.exec(cmd))
    {
        emit AppEvent::instance()->errorOccured("<database><dbutil>数据查询失败!");
        //        QLOG_INFO() << "<database><dbutil>数据查询失败!";
        return false;
    }
    if(m_sqlQuery.next())
    {
        for(int index = 0;index < m_sqlQuery.record().count();index++)
        {
            QSqlField field = m_sqlQuery.record().field(index);
            resultMap[field.name()] = field.value();
        }
        return true;
    }
    return false;
}

bool QDBUtil::queryAll(const QString &cmd, QList<QVariantMap> &resultMapList)
{
    if(!m_sqlQuery.exec(cmd))
    {
        emit AppEvent::instance()->errorOccured("<database><dbutil>数据查询失败!");
        //        QLOG_INFO() << "<database><dbutil>数据查询失败!";
        return false;
    }
    while(m_sqlQuery.next())
    {
        QVariantMap resultMap;
        for(int index = 0;index < m_sqlQuery.record().count();index++)
        {
            QSqlField field = m_sqlQuery.record().field(index);
            resultMap[field.name()] = field.value();
        }
        resultMapList.append(resultMap);
    }
    return true;
}

bool QDBUtil::update(const QString &cmd)
{
    if(!m_sqlQuery.exec(cmd))
    {
        //        qDebug()<<cmd;
        emit AppEvent::instance()->errorOccured("<database><dbutil>数据执行失败!");
        //        QLOG_ERROR() << "<database><dbutil>数据执行失败!";
        return false;
    }
    return true;
}

bool QDBUtil::update(const QStringList &cmdList)
{
    m_db.transaction();
    for(int index = 0; index < cmdList.count(); ++index)
        if(!m_sqlQuery.exec(cmdList.at(index)))
        {
            emit AppEvent::instance()->errorOccured("<database><dbutil>数据执行失败!");
            //            QLOG_ERROR() << "<database><dbutil>数据执行失败!";
            return false;
        }
    if(!m_db.commit())
    {
        emit AppEvent::instance()->errorOccured("<database><dbutil>数据提交失败!");
        //        QLOG_ERROR() << "<database><dbutil>数据提交失败!";
        return false;
    }
    return true;
}
