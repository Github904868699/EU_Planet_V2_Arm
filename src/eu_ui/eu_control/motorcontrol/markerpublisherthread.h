#ifndef MARKERPUBLISHERTHREAD_H
#define MARKERPUBLISHERTHREAD_H

#include <QThread>
#include <QMutex>

class MarkerPublisherThread : public QThread
{
    Q_OBJECT
private:
    explicit MarkerPublisherThread(QObject *parent = nullptr);
    ~MarkerPublisherThread();
public:
    static MarkerPublisherThread *instance();
    void setPlanFrame(const std::string &planFrame);
protected:
    virtual void run() override;
private:
    std::string m_planFrame;
    bool m_isQuit = false;
    QMutex m_mutex;
};

#endif // MARKERPUBLISHERTHREAD_H
