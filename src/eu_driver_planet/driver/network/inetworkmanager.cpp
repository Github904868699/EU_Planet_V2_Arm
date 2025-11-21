#include "inetworkmanager.h"
#include "tcpclient.h"
#include "udpclient.h"
#include <QMetaObject>
#include <QDebug>

INetworkManager::INetworkManager(QObject *parent) : QObject(parent)
{
    m_tcpClient.reset(new TcpClient);
    m_udpClient.reset(new UdpClient);
    connect(m_tcpClient.data(),SIGNAL(connectStateUpdated(bool)),this,SLOT(delTcpConnectStateUpdated(bool)));
    connect(m_udpClient.data(),SIGNAL(connectStateUpdated(bool)),this,SLOT(delUdpConnectStateUpdated(bool)));
    connect(m_tcpClient.data(),SIGNAL(dataSend(QByteArray)),this,SIGNAL(dataSend(QByteArray)));
    connect(m_udpClient.data(),SIGNAL(dataSend(QByteArray)),this,SIGNAL(dataSend(QByteArray)));
    connect(m_tcpClient.data(),SIGNAL(dataReceived(QByteArray)),this,SIGNAL(dataReceived(QByteArray)));
    connect(m_udpClient.data(),SIGNAL(dataReceived(QByteArray)),this,SIGNAL(dataReceived(QByteArray)));
    connect(m_tcpClient.data(),SIGNAL(error(QString)),this,SIGNAL(error(QString)));
    connect(m_udpClient.data(),SIGNAL(error(QString)),this,SIGNAL(error(QString)));
    connect(m_udpClient.data(),SIGNAL(dataReceived(QByteArray)),this,SLOT(test1(QByteArray)));
}

INetworkManager *INetworkManager::instance()
{
    static INetworkManager data;
    return &data;
}

void INetworkManager::init()
{
    m_tcpConnectState = -1;
    m_udpConnectState = -1;
    QMetaObject::invokeMethod(m_tcpClient.data(),"init");
    QMetaObject::invokeMethod(m_udpClient.data(),"init");
}

void INetworkManager::writeTcpData(const QByteArray &data)
{
    QMetaObject::invokeMethod(m_tcpClient.data(),"writeData",Q_ARG(QByteArray,data));
}

void INetworkManager::writeTcpData(const QList<QByteArray> &dataList)
{
    QMetaObject::invokeMethod(m_tcpClient.data(),"writeData",Q_ARG(QList<QByteArray>,dataList));
}

void INetworkManager::writeUdpData(const QByteArray &data)
{
    QMetaObject::invokeMethod(m_udpClient.data(),"writeData",Q_ARG(QByteArray,data));
}

bool INetworkManager::getConnectState() const
{
    if(-1 == m_tcpConnectState || -1 == m_udpConnectState) return false;
    return (m_tcpConnectState & m_udpConnectState);
}

void INetworkManager::delTcpConnectStateUpdated(bool isConenct)
{
    m_tcpConnectState = isConenct;
    if(-1 == m_tcpConnectState || -1 == m_udpConnectState) return;
    emit connectStateUpdated(m_tcpConnectState & m_udpConnectState);
}

void INetworkManager::delUdpConnectStateUpdated(bool isConnect)
{
    m_udpConnectState = isConnect;
    if(-1 == m_tcpConnectState || -1 == m_udpConnectState) return;
    emit connectStateUpdated(m_tcpConnectState & m_udpConnectState);
}

void INetworkManager::test1(const QByteArray &arr)
{
    qDebug()<<"________________________________________________"<<arr.toHex(' ');
    qDebug()<<"________________________________________________";
    qDebug()<<"________________________________________________";
    qDebug()<<"________________________________________________";
    qDebug()<<"________________________________________________";
}
