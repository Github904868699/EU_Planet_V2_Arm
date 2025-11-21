/****************************************************************************
**  数据库连接管理
****************************************************************************/
#ifndef QDBUTIL_H
#define QDBUTIL_H

#include <QObject>
#include <QSqlDatabase>
#include <QSqlQuery>

class QDBUtil : public QObject
{
    Q_OBJECT
public:
    explicit QDBUtil(QObject *parent = nullptr);
    ~QDBUtil();
    bool init(const QString &dbName, const QString &connectName);
    bool queryOne(const QString &cmd, QVariantMap &resultMap);
    bool queryAll(const QString &cmd, QList<QVariantMap> &resultMapList);
    bool update(const QString &cmd);
    bool update(const QStringList &cmdList);
signals:
    void errorOccured(const QString &err);
private:
    QSqlDatabase m_db;
    QSqlQuery m_sqlQuery;
};

#endif // QDBUTIL_H
