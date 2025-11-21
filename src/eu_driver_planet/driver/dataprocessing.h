#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#include <QThread>
#include <ros/node_handle.h>

class IDriver;

class DataProcessing : public QObject
{
    Q_OBJECT
public:
    explicit DataProcessing(IDriver *driver, QObject *parent = nullptr);
    ~DataProcessing();
public slots:
    void delReceiveData(const QByteArray &recData); //处理接收到的数据，只能通过信号的方式触发
signals:
    void errorSignal(const QString &error);
private:
    QThread m_thread;
    ros::NodeHandle nh_;
    IDriver *m_driver;
};

#endif // DATAPROCESSING_H
