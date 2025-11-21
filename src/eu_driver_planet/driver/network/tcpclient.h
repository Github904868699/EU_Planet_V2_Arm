#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include "iclient.h"
#include <QTcpSocket>
#include <QPointer>

class TcpClient : public IClient
{
    Q_OBJECT
public:
    explicit TcpClient(QObject *parent = nullptr);
    ~TcpClient();
public slots:
    virtual void init() override;
    virtual void writeData(const QByteArray &data) override;
    virtual void writeData(const QList<QByteArray> &dataList) override;
private slots:
    void readyReadSlot();
    void connectSlot();
private:
    QThread m_thread;
    QPointer<QTcpSocket> m_socket;
    bool m_isExist = false;
};

#endif // TCPCLIENT_H
