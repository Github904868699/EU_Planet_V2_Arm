#include "tcpclient.h"
#include <QHostAddress>
#include <QTime>
#include <QDebug>
#include "global.h"

TcpClient::TcpClient(QObject *parent) : IClient(parent)
{
    this->moveToThread(&m_thread);
    m_thread.start();
}

TcpClient::~TcpClient()
{
    m_isExist = true;
    m_thread.quit();
    m_thread.wait();
}

void TcpClient::writeData(const QByteArray &data)
{
    if(m_socket.isNull()) return;

    QByteArray sendData = data;
    while(sendData.size())
    {
        //        qDebug()<<"Client:TCP发送:"<<QTime::currentTime().toString("HH:mm:ss:zzz") + ": " + data;
        int size = m_socket.data()->write(sendData, sendData.size());
        sendData.remove(0,size);
    }
    emit dataSend(data);
}

void TcpClient::writeData(const QList<QByteArray> &dataList)
{
    if(m_socket.isNull()) return;

    static QByteArray sendData;
    sendData.clear();
    for(int index = 0;index < dataList.size();++index)
        sendData.append(dataList.at(index));

    while(sendData.size())
    {
        //        qDebug()<<"tcp规划发送时间:"<<QTime::currentTime();
        int size = m_socket.data()->write(sendData,sendData.size());
        sendData.remove(0,size);
    }

}

void TcpClient::init()
{
    if(m_isExist) return;
    if(m_socket.isNull())
    {
        m_socket = new QTcpSocket;
        connect(m_socket.data(),SIGNAL(disconnected()),this,SLOT(init()),Qt::QueuedConnection);
        connect(m_socket.data(),SIGNAL(connected()),this,SLOT(connectSlot()),Qt::QueuedConnection);
        connect(m_socket.data(),SIGNAL(readyRead()),this,SLOT(readyReadSlot()));
    }
    m_socket.data()->connectToHost(QHostAddress("192.168.1.88"),TCPPort);
    if(!m_socket.data()->waitForConnected(1000))
    {
        qDebug()<<"TCP端口连接失败";
        emit error("client tcp connect to server failed");
        emit connectStateUpdated(false);
        init();
        return;
    }
    qDebug()<<"TCP端口连接成功";
    emit connectStateUpdated(true);
}

void TcpClient::readyReadSlot()
{
    if(m_socket.data()->bytesAvailable() <= 0) return;
    static QByteArray data;
    data.append(m_socket.data()->readAll());
    //    qDebug()<<"接收:"<<QTime::currentTime().toString("HH:mm:ss:zzz") + ": " + data.toHex(' ');

    while(data.size() >= 17)
    {
        //        qDebug()<<"tcp接收:"<<QTime::currentTime().toString("HH:mm:ss:zzz") + ": " + data.left(17).toHex(' ');
        emit dataReceived(data.left(17));
       data.remove(0,17);
    }

}

void TcpClient::connectSlot()
{
    emit error("client tcp connect to server success!");
}
