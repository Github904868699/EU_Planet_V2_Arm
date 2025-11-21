#ifndef DBCOORDINATEINFOACCESS_H
#define DBCOORDINATEINFOACCESS_H

#include <QObject>

class QDBUtil;

static const QString Method_xOy = "xOy";
static const QString Method_yOz = "yOz";
static const QString Method_zOx = "zOx";
static const QString Method_xOxy = "xOxy";
static const QString Method_xOxz = "xOxz";
static const QString Method_yOyz = "yOyz";
static const QString Method_yOyx = "yOyx";
static const QString Method_zOzx = "zOzx";
static const QString Method_zOzy = "zOzy";

typedef struct _SqlCoordinateInfo
{
    QString coordName;
    QString toolName;
    QString method; //见上面定义
    QString point1;
    QString point2;
    QString point3;
}SqlCoordinateInfo;

class DbCoordinateInfoAccess : public QObject
{
    Q_OBJECT
private:
    explicit DbCoordinateInfoAccess(QObject *parent = nullptr);
    QVariantMap fromEntityToDataSet(const SqlCoordinateInfo &sqlCoordinateInfo) const;
    SqlCoordinateInfo fromDataSetToEntity(const QVariantMap &valueMap) const;
    QList<SqlCoordinateInfo> fromDataSetToEntity(const QList<QVariantMap> &valueMapList) const;
    QString getInsertCmd(const SqlCoordinateInfo &info) const;
    QString getUpdateCmd(const QString &coordName, const SqlCoordinateInfo &info) const;
public:
    static DbCoordinateInfoAccess *instance();
    bool insert(const SqlCoordinateInfo &info);
    bool update(const QString &coordName, const SqlCoordinateInfo &info);
    bool removeByCoordName(const QString &coordName);
    bool queryByCoordName(const QString &coordName, SqlCoordinateInfo &info);
    QList<SqlCoordinateInfo> queryAll();
private:
    QDBUtil *m_db;
};

#endif // DBCOORDINATEINFOACCESS_H
