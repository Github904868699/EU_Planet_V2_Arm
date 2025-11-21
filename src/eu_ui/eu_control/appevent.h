#ifndef APPEVENT_H
#define APPEVENT_H

#include <QObject>

class AppEvent : public QObject
{
    Q_OBJECT
private:
    explicit AppEvent(QObject *parent = nullptr);
public:
    static AppEvent *instance();
signals:
    void errorOccured(const QString &err);
private slots:
    void motorPlanControlFinished();                                //电机规划运动结束信号
    void userInputPortStateUpdated(unsigned state);                 //用户输入数字端口状态更新
    void userOutputPortStateUpdated(unsigned state);                //用户输出数字端口状态更新
    void motorInitStateUpdated(bool enable);                        //电机初始化状态更新
    void motorOffsetUpdated(const std::vector<float> & angles);     //电机偏移数据更新
    void motorBandTypeBrakeUpdated(bool isOpen);                    //电机抱闸状态更新
    void errorOccuredSlot(const QString &err);                          //命令执行的过程中发生错误

    void planFailed();                                              //规划失败
    void currentPlanFrameChanged(const QString &frame);             //的

    void coordinateSystemInfoUpdated();
};

#endif // APPEVENT_H
