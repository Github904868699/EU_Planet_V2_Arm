#ifndef IMOTORCONTROL_H
#define IMOTORCONTROL_H

#include "global.h"
#include <QObject>
#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class IMotorControl : public QObject
{
    Q_OBJECT
public:
    static IMotorControl *instance();
    //    rviz
    void publishPlanningFrame(const std::string &planFram);                 //发布当前规划坐标系（仅用于rviz显示）
    void publishRobotTargetState(const std::vector<double> &joints);        //发布目标姿态（仅用于rviz显示）
    void publishRobotTargetState(const TargetPose &pose);                   //发布目标姿态（仅用于rviz显示）
    void publishCoordinateSystem(const std::string &poseFrame, const TargetPose &pose, const std::string &coordName);//发布坐标系 pose为相对于poseFrame的值
    void publishWayPoint(const std::string &poseFrame, int id, const TargetPose &pose);//发布三个路点，pose为相对于poseFrame的值
    //    motor
    void publishRequest(RequestCmd cmd) const;                              //请求命令
    void publishSetUserOutputState(unsigned states);                        //发布设置的用户输出端口状态
    void publishSetUserOutputState(int index, bool state);                  //发布设置的用户输出端口状态
    void publishSetMotorOffset(OffsetType type, int id, float offSet);      //发布设置一个电机的偏移
    unsigned getUserInputState() const;                                     //获取用户输入端口状态
    unsigned getUserOutputState() const;                                    //获取用户输出端口状态
    void publishSetGripper(int value);                                      //设置夹爪
    void setPlanningResult(double result);                                  //每次规划前
signals:
    void motorPlanControlFinished();                                //电机规划运动结束信号
    void userInputPortStateUpdated(unsigned);                       //用户输入数字端口状态更新
    void userOutputPortStateUpdated(unsigned);                      //用户输出数字端口状态更新
    void motorInitStateUpdated(bool);                               //电机初始化状态更新
    void motorOffsetUpdated(const std::vector<float> &);            //电机偏移数据更新
    void motorBandTypeBrakeUpdated(bool);                           //电机抱闸状态更新
private:
    explicit IMotorControl(QObject *parent = nullptr);
    void delMotorControlResult(const control_msgs::FollowJointTrajectoryActionResult &result);
    void delResponseCmd(const std_msgs::UInt8 &msg);
    void delUserInputState(const std_msgs::UInt32 &msg);
    void delUserOutputState(const std_msgs::UInt32 &msg);
    void delErrorMsg(const std_msgs::String &msg);
    void delMotorOffset(const std_msgs::Float32MultiArray &msg);

private:
    ros::NodeHandle m_node;
    robot_state::RobotStatePtr m_robotState;        //用于发布多个机器人姿态

    ros::Publisher m_euMotorCtlPub;                 //发布电机控制话题（使能、失能。。。。。）
    ros::Publisher m_euTargetRobotModelStatePub;    //发布机器人模型目标姿态
    ros::Publisher m_setUserOutputStatePub;         //发布设置的用户输出状态
    ros::Publisher m_setOffsetPub;                  //发布设置电机偏移值
    QMap<int, ros::Publisher> m_PointMarkMap;       //(3个)用于发布标定坐标系时的路点
    ros::Publisher m_markerPub;
    ros::Publisher m_setGripper;                    //设置夹爪

    ros::Subscriber m_motorCtlResultSub;            //订阅规划控制结束话题
    ros::Subscriber m_UserInputStateSub;            //订阅用户输入状态
    ros::Subscriber m_UserOutputStateSub;           //订阅用户输出状态
    ros::Subscriber m_errorSub;                     //订阅发送的错误
    ros::Subscriber m_recordTrajSub;                //订阅录制轨迹
    ros::Subscriber m_offSetSub;                    //订阅电机偏移值
    ros::Subscriber m_responseSub;                  //订阅初始化是否成功状态

    double m_planResult = 1;
    unsigned m_userInputPortState;                  //保存当前用户输入端口状态
    unsigned m_userOutputPortState;                 //保存当前用户输出端口状态
};

#endif // IMOTORCONTROL_H
