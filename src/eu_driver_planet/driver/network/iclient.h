#ifndef ICLIENT_H
#define ICLIENT_H

#include <QObject>
#include <QThread>
#include <QAbstractSocket>

class IClient : public QObject
{
    Q_OBJECT
public:
    explicit IClient(QObject *parent = nullptr);
public slots:
    virtual void init() = 0;
    virtual void writeData(const QByteArray &data) = 0;
    virtual void writeData(const QList<QByteArray> &dataList) = 0;
signals:
    void connectStateUpdated(bool isConnect);
    void dataSend(const QByteArray &);
    void dataReceived(const QByteArray &);       //接收到数据
    void error(const QString &err);
};

#endif // ICLIENT_H
