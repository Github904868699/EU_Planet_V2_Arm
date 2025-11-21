#ifndef INETWORKMANAGER_H
#define INETWORKMANAGER_H

#include <QObject>
#include "tcpclient.h"
#include "udpclient.h"

class INetworkManager : public QObject
{
    Q_OBJECT
public:
    static INetworkManager *instance();
    void init();
    void writeTcpData(const QByteArray &data);
    void writeTcpData(const QList<QByteArray> &dataList);
    void writeUdpData(const QByteArray &data);
    bool getConnectState() const;
signals:
    void connectStateUpdated(bool isConnect);
    void dataSend(const QByteArray &data);
    void dataReceived(const QByteArray &data);
    void error(const QString &err);
private:
    explicit INetworkManager(QObject *parent = nullptr);
private slots:
    void delTcpConnectStateUpdated(bool isConenct);
    void delUdpConnectStateUpdated(bool isConnect);
public slots:
    void test1(const QByteArray &arr);
public:
    QScopedPointer<IClient> m_tcpClient;
    QScopedPointer<IClient> m_udpClient;
    int m_tcpConnectState{-1}; //0代表未连接，1代表连接，-1代表未知
    int m_udpConnectState{-1};
};

#endif // INETWORKMANAGER_H
