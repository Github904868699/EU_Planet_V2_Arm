#ifndef UDPCLIENT_H
#define UDPCLIENT_H

#include "iclient.h"
#include <QUdpSocket>
#include <QPointer>

class UdpClient : public IClient
{
    Q_OBJECT
public:
    explicit UdpClient(QObject *parent = nullptr);
    ~UdpClient();
public slots:
    virtual void init() override;
    virtual void writeData(const QByteArray &data) override;
    virtual void writeData(const QList<QByteArray> &dataList) override;
private slots:
    void readDatagrams();
private:
    QThread m_thread;
    QPointer<QUdpSocket> m_socket;
};

#endif // UDPCLIENT_H
