#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <vector>
#include <string>

#ifndef EXTERNFUNC
#ifdef WIN32
#define EXTERNFUNC __declspec(dllexport)
#else
#define EXTERNFUNC
#endif
#endif

#define ROBOT_SUCCESS 0
#define ROBOT_SETJOINTFAILED 1

typedef struct _RobotTargetPose             //目标姿态
{
    double x = 0;                           //单位mm
    double y = 0;                           //单位mm
    double z = 0;                           //单位mm
    double roll = 0;                        //单位度
    double pitch = 0;                       //单位度
    double yaw = 0;                         //单位度
}RobotTargetPose;

enum RobotWorkingMode                       //工机械臂作模式
{
    RobotWorkingMode_Actual,                //控制真实机械臂
    RobotWorkingMode_Simulation             //控制仿真机械臂
};

enum RobotEventType                         //事件类型
{
    RobotEvent_Error,                       //错误信息
    RobotEvent_RobotStartupDone,            //机械臂上电初始化完成
    RobotEvent_RobotShutdownDone,           //机械臂掉电关机完成
    RobotEvent_RobotBrakeOpenDone,          //机械臂打开抱闸完成
    RobotEvent_RobotBrakeCloseDone,         //机械臂关闭抱闸完成
    RobotEvent_RobotArriveTargetPos,        //机械臂轨迹运动到位
    RobotEvent_RobotPlanFailed,             //机械臂规划失败
    RobotEvent_PlanFrameChanged,            //机械臂参考坐标系改变，RobotEventCallback第二个参数是坐标系名称
    RobotEvent_PlanFrameInfoUpdated,        //机械臂参考坐标系信息更新（添加/删除/修改坐标系）
};

typedef void (*RobotEventCallback)(RobotEventType type, const std::string &info);       //事件回调函数
typedef void (*RoportInputPortStateUpdatedCallback)(unsigned state);                    //输入端口状态刷新回调函数
typedef void (*RobotOutputPortStateUpdatedCallback)(unsigned state);                    //输出端口状态刷新回调函数
typedef void (*RobotOffsetUpdatedCallback)(const std::vector<float> &offsets);          //电机偏移值更新回调函数

//回调函数
int robotSetRobotEventCallback(RobotEventCallback callback);                            //设置事件回调函数
int robotSetInputPortStateUpdatedCallback(RoportInputPortStateUpdatedCallback callback);//设置输入端口状态刷新回调函数
int robotSetOutputPortStateUpdatedCallback(RobotOutputPortStateUpdatedCallback callback);//设置输出端口状态刷新回调函数
int robotSetRobotOffsetUpdatedCallback(RobotOffsetUpdatedCallback callback);            //设置电机偏移值更新回调函数
//初始化相关
int robotInitDLL();                                                                     //初始化，所有接口调用前调用该接口
int robotFreeDLL();                                                                     //释放资源，软件关闭前调用该接口
int robotStartUp();                                                                     //上电初始化
int robotShutdown();                                                                    //掉电关机
int robotOpenBrake();                                                                   //打开抱闸
int robotCloseBrake();                                                                  //关闭抱闸
int robotClearOffset(int index);                                                        //清除偏移值
int robotSetOffset(int index, double offset);                                           //设置关节偏移量
//夹爪和IO控制
int robotSetGriper(int angle);                                                          //设置夹爪角度(单位度，范围0～180度)
int robotSetOutputPortState(unsigned states);                                           //设置输出端口状态，所有
int robotSetOutputPortState(int index, bool state);                                     //设置输出端口状态，单个
//运动规划控制
int robotGetWorkingMode(RobotWorkingMode &mode);                                        //获得机械臂的工作模式
int robotSetWorkingMode(RobotWorkingMode mode);                                         //设置机械臂的工作模式
int robotGetVelocityFactor(double &factor);                                             //获得机械臂运动速度因子(范围0-1,1为最大速度)
int robotSetVelocityFactor(double factor);                                              //设置机械臂运动速度因子
int robotGetPlanningFrame(std::string &frame);                                          //获得机械臂规划的参考坐标系
int robotSetPlanningFrame(const std::string &frame);                                    //设置机械臂规划的参考坐标系
int robotGetToolLength(double &length);                                                 //获得机械臂末端的工具长度
int robotSetToolLength(double &length);                                                 //设置机械臂末端的工具长度，该设置需要重启所有节点
int robotGetTerminalPose(RobotTargetPose &pose);                                        //获得终端位姿
int robotGetJointAngles(std::vector<double> &angles);                                   //获得关节角度
int robotJointMove(const std::vector<double> &angles);                                  //关节规划，给定6个关节角度
int robotJointMove(int index, double angle);                                            //关节规划，给定1个关节角度
int robotJointMove(const RobotTargetPose &pose);                                        //关节规划，给定末端位姿
int robotLineMove(const std::vector<double> &angles);                                   //直线规划，给定6个关节角度
int robotLineMove(const std::vector<std::vector<double> > &anglesList);                 //直线规划，给定多个轨迹点(每个轨迹点包含6个关节角度)
int robotLineMove(const RobotTargetPose &pose);                                         //直线规划，给定末端位姿
int robotLineMove(const std::vector<RobotTargetPose> &poses);                           //直线规划，给定多个轨迹点
int robotCircleMove(const std::vector<std::vector<double> > &anglesList);               //圆规划，给定三个轨迹点(每个轨迹点包含6个关节角度)，规划三个点的外接圆
int robotCircleMove(const std::vector<RobotTargetPose> &poses);                         //圆规划，给定三个轨迹点，规划三个点的外接圆
int robotStop();                                                                        //停止运动

#endif // ROBOTCONTROL_H
