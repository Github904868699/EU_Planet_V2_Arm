#ifndef IPLANNINGMANAGER_H
#define IPLANNINGMANAGER_H

#include "global.h"
#include <QThread>
#include <ros/publisher.h>
#include <tf2_ros/buffer.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <std_msgs/Bool.h>
#include <moveit/move_group_interface/move_group_interface.h>

class IPlanningManager : public QThread
{
    Q_OBJECT
public:
    static IPlanningManager *instance();
    //    规划前参数
    double getPlanningVelocityScalingFactor() const {return m_velocityFactor;}          //获得速度比例因子
    void setPlanningVelocityScalingFactor(double factor){m_velocityFactor = factor;}    //设置速度比例因子
    std::string getPlanningFrame() const {return m_planFram;}                           //获得规划坐标系名称
    void setPlanningFrame(const std::string &frame);                                    //设置规划坐标系
    ControlMode getControlWay() const;                                                   //获得控制模式
    void setControlWay(ControlMode mode);                                          //设置控制模式
    double getToolLength() const;                                                       //获得末端工具长度mm
    bool setToolLength(double length);                                                  //设置末端工具长度mm

    //    空间规划
    void setNamedTarget(const QString &name);                                           //回归指定名称的位姿
    bool addJointValueTarget(int index, double value);                                  //对某个轴
    bool setJointValueTarget(const std::vector<double> &joints);                        //关节空间规划(单位度)
    bool setJointValueTarget(int index, double value);                                  //关节空间规划(单位度)
    bool setJointValueTarget(const TargetPose &targetPose);                             //关节空间规划(单位度)
    bool trajectoryPlanning(const std::vector<double> &joints);                         //笛卡尔空间规划 画直线
    bool trajectoryPlanning(const std::vector<std::vector<double> > &jointsList);       //笛卡尔空间规划 画直线
    bool trajectoryPlanning(const TargetPose &targetPosed);                             //笛卡尔空间规划 画直线
    bool trajectoryPlanning(const std::vector<TargetPose> &targetPoses);                //笛卡尔空间规划 画直线
    bool arcPlanning(const std::vector<std::vector<double> > &jointsList);              //笛卡尔空间规划 画弧
    bool arcPlanning(const std::vector<TargetPose> &targetPoses);                       //笛卡尔空间规划 画弧
    void stop();                                                                        //停止所有规划

    //    获取信息
    TargetPose getCurrentPose(){return m_targetPos;}                                    //获得当前终端位姿
    std::vector<double> getCurrentJointValues(){return m_jointValues;}                  //获得当前各关节弧度
    std::vector<double> getCurrentVelocityValues(){return m_velValues;}                 //获得当前速度
    std::vector<double> getCurrentAccelerationValues(){return m_accValues;}             //获得当前加速度

    TargetPose JointsToPose(const std::string &frame, const std::vector<double> &joints);
    TargetPose JointsToPose(const std::vector<double> &joints);
    std::vector<TargetPose> JointsToPose(const std::string &frame, const std::vector<std::vector<double> > &jointsList);
    std::vector<TargetPose> JointsToPose(const std::vector<std::vector<double> > &jointsList);

signals:
    void planFailed();     //规划失败
    void currentPlanFrameChanged(const QString &frame);
private:
    IPlanningManager(QObject *parent = nullptr);
    ~IPlanningManager();
protected:
    virtual void run() override;
private:
    //    控制规划
    moveit::planning_interface::MoveGroupInterface *m_group;        //moveit
    std::string m_planFram;                                         //当前参考坐标系
    double m_velocityFactor = 1;                                    //速度比例因子（0-1）
    tf2_ros::Buffer m_buffer;                                       //保存坐标转换信息
    TargetPose m_targetPos;                                         //实时保存的终端位姿信息
    std::vector<double> m_jointValues = {0,0,0,0,0,0};              //实时保存6个轴的关节位置数据
    std::vector<double> m_velValues = {0,0,0,0,0,0};                //实时保存6个轴的关节速度
    std::vector<double> m_accValues = {0,0,0,0,0,0};                //实时保存6个轴的关节加速度数据
};

#endif /* IPLANNINGMANAGER_H */
