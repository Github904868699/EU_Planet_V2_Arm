#ifndef ROSTHREAD_H
#define ROSTHREAD_H

#include <QThread>

class RosThread : public QThread
{
private:
    explicit RosThread(QObject *parent = nullptr);
    ~RosThread() override;
public:
    static RosThread *instance();
protected:
    virtual void run() override;
};

#endif // ROSTHREAD_H
