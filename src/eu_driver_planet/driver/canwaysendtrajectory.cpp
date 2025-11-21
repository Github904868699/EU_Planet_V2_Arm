#include "canwaysendtrajectory.h"
#include "global.h"
#include "driverforcan.h"
#include "eu_planet.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>
#include <QDebug>
#include <QTime>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

static bool IsStop = true;
static actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GH;
static IDriver *m_driver;

struct period_info
{
    struct timespec next_period;
    long period_ns;
};

//!不能用qDebug()，会崩溃
void *simple_cyclic_task(void *data)
{
    struct period_info pinfo;
    pinfo.period_ns = SampleDuration * 1000000000;
    clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));

    control_msgs::FollowJointTrajectoryGoal::_trajectory_type type = GH.getGoal()->trajectory;
    if(0 == type.points.size())
    {
        IsStop = true;
        emit m_driver->cacheSendFinished();
        return nullptr;
    }
    std::vector<double>::size_type count = 0;
    std::vector<float> m_Offsets = IDriver::m_Offsets;
    static struct timeval tv;
    static struct timeval pre = tv;
    while(!IsStop && count < type.points.size())
    {
        pinfo.next_period.tv_nsec += pinfo.period_ns;
        while (pinfo.next_period.tv_nsec >= 1000000000)
        {
            pinfo.next_period.tv_sec++;
            pinfo.next_period.tv_nsec -= 1000000000;
        }
        gettimeofday(&tv,nullptr);
        long long cha = (tv.tv_sec * 1000000 + tv.tv_usec) - (pre.tv_sec * 1000000 + pre.tv_usec) - SampleDuration * 1000000;
//        if(cha > 10 || cha < -10)
//            printf("pre:us:%ld now:us:%ld cha:%lld\n",(pre.tv_sec * 1000000 + pre.tv_usec),(tv.tv_sec * 1000000 + tv.tv_usec),cha);  //微秒
        pre = tv;

        for(size_t index = 0;index < 6;++index)
        {
            const auto &temp = type.points.at(count);
            if(JointsIdMap.contains(type.joint_names[index]))
            {
                int id = JointsIdMap.value(type.joint_names[index]);
                if(id == 1)
                    printf("电机ID:%u 位置:%f \n",id,Global::radianToAngle(temp.positions.at(index)) - m_Offsets[id - 1]);  //微秒
                planet_quick_setTargetPosition(0,id,Global::radianToAngle(temp.positions.at(index)) - m_Offsets[id - 1]);
            }
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo.next_period, nullptr);
        ++count;
    }
    emit m_driver->cacheSendFinished();
    return nullptr;
}

void tie_self_thread_to_cpu(int cpu_num)
{
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(cpu_num, &mask); //指定该线程使用的CPU
    if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0)
    {
        perror("pthread_setaffinity_np");
    }
}

CanWaySendTrajectory::CanWaySendTrajectory(IDriver *driver, QObject *parent) :
    QObject(parent)
{
    m_driver = driver;
}

void CanWaySendTrajectory::startSendTrajectory(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
    GH = gh;
    struct sched_param param;
    pthread_attr_t attr;

    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        qDebug()<<"mlockall failed: %m\n";
        return;
    }
    if (pthread_attr_init(&attr))
    {
        qDebug()<<"init pthread attributes failed";
        return;
    }
    if(pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN))
    {
        qDebug()<<"pthread setstacksize failed";
        return;
    }
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO))
    {
        qDebug()<<"pthread setschedpolicy failed";
        return;
    }
    param.sched_priority = 99;
    if (pthread_attr_setschedparam(&attr, &param))
    {
        qDebug()<<"pthread setschedparam failed";
        return;
    }
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED))
    {
        qDebug()<<"pthread setinheritsched failed";
        return;
    }
    tie_self_thread_to_cpu(3);
    IsStop = false;
    if (pthread_create(&thread, &attr, simple_cyclic_task, nullptr))
    {
        qDebug()<<"create pthread failed";
        return;
    }

    //    /* Join the thread and wait until it is done */
    //    ret = pthread_join(thread, NULL);
    //    if (ret)
    //        printf("join pthread failed: %m\n");
}

void CanWaySendTrajectory::stop()
{
    if(!IsStop)
    {
        IsStop = true;
        pthread_join(thread, nullptr);
    }
}
