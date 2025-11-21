#ifndef P_ROBOTCONTROL_H
#define P_ROBOTCONTROL_H

#include "robotcontrol.h"
#include "global.h"
#include <QStandardItemModel>

extern RobotEventCallback RobotEventFunc;
extern RoportInputPortStateUpdatedCallback InputPortStateUpdatedFunc;
extern RobotOutputPortStateUpdatedCallback OutportStateUpdatedFunc;
extern RobotOffsetUpdatedCallback RobotOffsetUpdatedFunc;

static inline TargetPose RobotTargetPoseToTargetPose(const RobotTargetPose &pose)
{
    TargetPose result;
    result.x = pose.x;
    result.y = pose.y;
    result.z = pose.z;
    result.roll = pose.roll;
    result.pitch = pose.pitch;
    result.yaw = pose.yaw;
    return result;
}

static inline std::vector<TargetPose> RobotTargetPoseToTargetPose(const std::vector<RobotTargetPose> &poses)
{
    std::vector<TargetPose> result;
    for(auto it = poses.begin();it != poses.end();++it)
        result.push_back(RobotTargetPoseToTargetPose(*it));
    return result;
}

static inline RobotTargetPose TargetPoseToRobotTargetPose(const TargetPose &pose)
{
    RobotTargetPose result;
    result.x = pose.x;
    result.y = pose.y;
    result.z = pose.z;
    result.roll = pose.roll;
    result.pitch = pose.pitch;
    result.yaw = pose.yaw;
    return result;
}

static inline std::vector<RobotTargetPose> TargetPoseToRobotTargetPose(const std::vector<TargetPose> &poses)
{
    std::vector<RobotTargetPose> result;
    for(auto it = poses.begin();it != poses.end();++it)
        result.push_back(TargetPoseToRobotTargetPose(*it));
    return result;
}

void setNamedTarget(const QString &name);                                           //回归指定名称的位姿
bool addJointValueTarget(int index, double value);                                  //对某个轴
TargetPose JointsToPose(const std::string &frame, const std::vector<double> &joints);
TargetPose JointsToPose(const std::vector<double> &joints);
std::vector<TargetPose> JointsToPose(const std::string &frame, const std::vector<std::vector<double> > &jointsList);
std::vector<TargetPose> JointsToPose(const std::vector<std::vector<double> > &jointsList);
void publishPlanningFrame(const std::string &planFram);                 //发布当前规划坐标系（仅用于rviz显示）
void publishRobotTargetState(const std::vector<double> &joints);        //发布目标姿态（仅用于rviz显示）
void publishRobotTargetState(const TargetPose &pose);                   //发布目标姿态（仅用于rviz显示）
void publishCoordinateSystem(const std::string &poseFrame, const TargetPose &pose, const std::string &coordName);//发布坐标系 pose为相对于poseFrame的值
void publishWayPoint(const std::string &poseFrame, int id, const TargetPose &pose);//发布三个路点，pose为相对于poseFrame的值
void publishRequest(RequestCmd cmd);                              //请求命令
unsigned getUserInputState();                                     //获取用户输入端口状态
unsigned getUserOutputState();                                    //获取用户输出端口状态
void publishRequest(RequestCmd cmd);                              //请求命令
QStandardItemModel *model();                                                            //坐标系信息列表
QStringList listCalibrateMethod();                                                      //列出标定方法
bool addCoordinateSystem(const CoordinateInfo &info);                                   //添加用户坐标系
bool removeCoordinateSystem(const QString &coordName);                                  //移除用户坐标系
bool alterCoordinateSystem(const QString &coordName, const CoordinateInfo &info);       //修改用户坐标系
bool queryCoordinateSystemInfo(const QString &coordName, CoordinateInfo &info);         //查询用户坐标系
QStringList listCoordinateSystem();
#endif // P_ROBOTCONTROL_H
