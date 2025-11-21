#include "udpclient.h"
#include <QUdpSocket>
#include <QTime>
#include "global.h"

UdpClient::UdpClient(QObject *parent) :
    IClient(parent)
{
    this->moveToThread(&m_thread);
    m_thread.start();
}

UdpClient::~UdpClient()
{
    m_thread.quit();
    m_thread.wait();
}

void UdpClient::init()
{
    if(m_socket.isNull())
    {
        m_socket = new QUdpSocket;
        connect(m_socket.data(),SIGNAL(readyRead()),this,SLOT(readDatagrams()));
    }
    if(!m_socket.data()->bind(UDPPort))//绑定本机发送的端口，否则随机端口发送
    {
        emit error("client udp listened failed");
        qDebug()<<"udp端口监听失败,请重新启动";
        emit connectStateUpdated(false);
        return;
    }
    qDebug()<<"UDP端口连接成功";
    emit connectStateUpdated(true);
}

void UdpClient::writeData(const QByteArray &data)
{
    if(m_socket.isNull()) return;
    //    qDebug()<<"Client:UDP发送:"<<QTime::currentTime().toString("HH:mm:ss:zzz") + ":" + QString(data);
    m_socket.data()->writeDatagram(data,QHostAddress("192.168.1.88"),UDPPort);
    emit dataSend(data);
}

void UdpClient::writeData(const QList<QByteArray> &dataList)
{
    for(int index = 0;index < dataList.count();++index)
    {
        const QByteArray &temp = dataList.at(index);
        m_socket.data()->writeDatagram(temp,QHostAddress("192.168.1.88"),UDPPort);
        emit dataSend(temp);
    }
}

void UdpClient::readDatagrams()
{
    while (m_socket.data()->hasPendingDatagrams()) //数据报等待被读取
    {
        QByteArray arr;
        arr.resize(m_socket.data()->bytesAvailable()); //接收数据
        int len = m_socket.data()->readDatagram(arr.data(),arr.size());
        //        qDebug()<<"udp接收:"<<QTime::currentTime().toString("HH:mm:ss:zzz") + ": " + arr.left(len).toHex(' ');
        emit dataReceived(arr.left(len));
    }
}
