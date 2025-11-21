#include "iplanningmanager.h"
#include "linethread.h"
#include "arcthread.h"
#include "jointthread.h"
#include "imotorcontrol.h"
#include <QMap>
#include <QDomComment>
#include <QFile>
#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "appevent.h"
#include <QTextStream>

static const QMap<QString,std::vector<double>> m_namedPoses = {{"Home",{0,0,0,0,0,0}},//关节初始姿态
                                                               {"Init",{0,0,-90,0,-90,0}}};

//                                                               {"Init",{-0.00172,-7.291862,-75.694718,21.596727,-89.999982,-0.000458}}};

IPlanningManager::IPlanningManager(QObject *parent) :
    QThread(parent)
{
    connect(LineThread::instance(),SIGNAL(planFailed()),this,SIGNAL(planFailed()));
    connect(ArcThread::instance(),SIGNAL(planFailed()),this,SIGNAL(planFailed()));
    connect(JointThread::instance(),SIGNAL(planFailed()),this,SIGNAL(planFailed()));

    m_group = new moveit::planning_interface::MoveGroupInterface(Arm_Group);//释放资源崩溃
    m_group->allowReplanning(true);
    m_group->setGoalJointTolerance(0.0001);
    m_group->setGoalPositionTolerance(0.0001);
    m_group->setGoalOrientationTolerance(0.0001);
    m_group->setMaxAccelerationScalingFactor(0.1);
    m_group->setMaxVelocityScalingFactor(1);
    m_group->setPlanningTime(5);
    m_group->startStateMonitor();
    IMotorControl::instance();//先初始化
    setPlanningFrame(World_Frame);

    this->start();
}

IPlanningManager::~IPlanningManager()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    this->wait();
}

IPlanningManager *IPlanningManager::instance()
{
    static IPlanningManager data;
    return &data;
}

void IPlanningManager::setPlanningFrame(const std::string &frame)
{
    m_planFram = frame;
    m_group->setPoseReferenceFrame(m_planFram);
    IMotorControl::instance()->publishPlanningFrame(m_planFram);
    emit currentPlanFrameChanged(QString::fromStdString(frame));
}

void IPlanningManager::setNamedTarget(const QString &name)
{
    setJointValueTarget(m_namedPoses.value(name));
}

bool IPlanningManager::addJointValueTarget(int index, double value)
{
    std::vector<double> angles = getCurrentJointValues();
    angles[index] += value;
    return setJointValueTarget(angles);
}

bool IPlanningManager::setJointValueTarget(const std::vector<double> &joints)
{
    //    电机进入操作模式
    IMotorControl::instance()->publishRequest(RequestCmd_Enter);

    if(!m_group->setJointValueTarget(Global::angleToRadian(joints)))
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><joint> 关节目标设置失败");
        //        QLOG_ERROR() << "<planning><joint> 关节目标设置失败";
        return false;
    }
    JointThread::instance()->setPara(m_group,m_velocityFactor,SampleDuration);
    JointThread::instance()->start();
    return true;
}

bool IPlanningManager::setJointValueTarget(int index, double value)
{
    std::vector<double> angles = getCurrentJointValues();
    angles[index] = value;
    return setJointValueTarget(angles);
}

bool IPlanningManager::setJointValueTarget(const TargetPose &targetPose)
{
    //    电机进入操作模式
    IMotorControl::instance()->publishRequest(RequestCmd_Enter);

    if(!m_group->setJointValueTarget(Global::targetPoseTo_Geometry_msgs_Pose(targetPose)))
    {
        emit planFailed();
        emit AppEvent::instance()->errorOccured("<planning><joint> 关节目标设置失败");
        //        QLOG_ERROR() << "<planning><joint> 关节目标设置失败";
        return false;
    }
    JointThread::instance()->setPara(m_group,m_velocityFactor,SampleDuration);
    JointThread::instance()->start();
    return true;
}

bool IPlanningManager::trajectoryPlanning(const std::vector<double> &joints)
{
    return trajectoryPlanning(JointsToPose(joints));
}

bool IPlanningManager::trajectoryPlanning(const std::vector<std::vector<double> > &jointsList)
{
    return trajectoryPlanning(JointsToPose(jointsList));
}

bool IPlanningManager::trajectoryPlanning(const TargetPose &targetPosed)
{
    //    std::vector<TargetPose> wayPoints = {getCurrentPose(),targetPosed};
    std::vector<TargetPose> wayPoints = {targetPosed};
    return trajectoryPlanning(wayPoints);
}

bool IPlanningManager::trajectoryPlanning(const std::vector<TargetPose> &targetPoses)
{
    //    电机进入操作模式
    IMotorControl::instance()->publishRequest(RequestCmd_Enter);

    LineThread::instance()->setPara(m_group,targetPoses,m_velocityFactor,SampleDuration);
    LineThread::instance()->start();
    return true;
}

bool IPlanningManager::arcPlanning(const std::vector<std::vector<double> > &jointsList)
{
    return arcPlanning(JointsToPose(jointsList));
}

bool IPlanningManager::arcPlanning(const std::vector<TargetPose> &targetPoses)
{
    //    电机进入操作模式
    IMotorControl::instance()->publishRequest(RequestCmd_Enter);

    ArcThread::instance()->setPara(m_group,targetPoses,m_planFram,m_velocityFactor,SampleDuration);
    ArcThread::instance()->start();
    return true;
}

TargetPose IPlanningManager::JointsToPose(const std::string &frame, const std::vector<double> &joints)
{
    robot_model::RobotModelConstPtr kinematic_model = m_group->getRobotModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(Arm_Group);

    kinematic_state->setJointGroupPositions(joint_model_group,Global::angleToRadian(joints));
    geometry_msgs::Pose p = Eigen::toMsg(kinematic_state->getGlobalLinkTransform(Tool_Frame));
    geometry_msgs::PoseStamped point_laser;
    point_laser.header.frame_id = World_Frame;
    point_laser.header.stamp = ros::Time(0);
    point_laser.pose.position = p.position;
    point_laser.pose.orientation = p.orientation;

    geometry_msgs::PoseStamped point_base;
    try
    {
        point_base = m_buffer.transform(point_laser,frame);
    }
    catch(const std::exception& e)
    {
        emit AppEvent::instance()->errorOccured("程序异常，获取tf变换失败!");
        //        QLOG_WARN() << "程序异常，获取tf变换失败!";
    }
    return Global::geometry_msgs_PoseToTargetPose(point_base.pose);
}

TargetPose IPlanningManager::JointsToPose(const std::vector<double> &joints)
{
    return JointsToPose(m_planFram,joints);
}

std::vector<TargetPose> IPlanningManager::JointsToPose(const std::string &frame, const std::vector<std::vector<double> > &jointsList)
{
    std::vector<TargetPose> poseList;
    for(size_t index = 0;index < jointsList.size();++index)
        poseList.push_back(JointsToPose(frame, jointsList.at(index)));
    return poseList;
}

std::vector<TargetPose> IPlanningManager::JointsToPose(const std::vector<std::vector<double> > &jointsList)
{
    return JointsToPose(m_planFram,jointsList);
}

void IPlanningManager::stop()
{
    m_group->stop();
    IMotorControl::instance()->setPlanningResult(1);
    IMotorControl::instance()->publishRequest(RequestCmd_Stop);
}

QDomDocument readFromXML(const QString &filePath)
{
    QFile file(filePath);
    if(!file.open(QFile::ReadOnly))
    {
        //        Logging::instance()->appendLog(Logging::LoggingType_Error,"demo.launch打开失败");
        return QDomDocument();
    }
    QDomDocument doc;
    if(!doc.setContent(&file))
    {
        file.close();
        //        Logging::instance()->appendLog(Logging::LoggingType_Error,"demo.launch打开失败");
        return QDomDocument();
    }
    file.close();
    return doc;
}

bool writeToXML(const QDomDocument &doc, const QString &filePath)
{
    QFile file(filePath);
    if(!file.open(QFile::WriteOnly|QFile::Truncate)) //可以用QIODevice，Truncate表示清空原来的内容
        return false;
    QTextStream out_stream(&file); //输出到文件
    doc.save(out_stream,2);     //缩进4格
    file.close();
    return true;
}

double IPlanningManager::getToolLength() const
{
    std::string urdf_file = ros::package::getPath("eu_description") + "/urdf/eu_description.urdf";
    //    QString filePath = QCoreApplication::applicationDirPath() + "/../../src/aubo_description/urdf/aubo_i5.urdf.xacro";
    QDomDocument doc = readFromXML(QString::fromStdString(urdf_file));
    if(doc.isNull()) return false;

    QDomElement rootElement = doc.documentElement(); //返回根元素
    QDomNodeList node2List = rootElement.elementsByTagName("joint");
    for(int i = 0;i < node2List.count();++i)
    {
        QDomElement eleMent = node2List.at(i).toElement();
        if("j7" == eleMent.attribute("name"))
        {
            QDomElement vOrign = eleMent.elementsByTagName("origin").at(0).toElement();
            return vOrign.attribute("xyz").split(" ").back().toDouble() * 1000.0;
        }
    }
    return 0.0;
}

bool IPlanningManager::setToolLength(double length)
{
    //    ros::NodeHandle nh;
    //    std::string urdf_string;
    //    nh.getParam(RobotDescription, urdf_string);
    //    urdf::Model model;
    //    std::string urdf_file = ros::package::getPath("eu_description") + "/urdf/eu_description.urdf";
    //    if (!model.initFile(urdf_file))
    //    {
    //        ROS_ERROR("Failed to parse URDF file");
    //        return false;
    //    }
    //    ROS_INFO("Successfully parsed URDF file");
    //    urdf::LinkSharedPtr link;
    //    model.getLink(Tool_Frame, link);
    //    if (link)
    //    {
    //        urdf::Box *geometry = new urdf::Box();
    //        geometry->dim.x = 0;
    //        geometry->dim.y = 0;
    //        geometry->dim.z = length / 1000.0;
    //        link.get()->visual.get()->geometry.reset(geometry);
    //    }
    ////    std::string modified_urdf_string;
    ////    model.exportXml(modified_urdf_string);
    ////    urdf::ModelInterface::
    ////    qDebug()<<QString::fromStdString(modified_urdf_string);
    //    robot_state_publisher::RobotStatePublisher robot_state_publisher(nh);
    //    sensor_msgs::JointState joint_state;
    //    joint_state.header.stamp = ros::Time::now();
    //    joint_state.name.push_back(Tool_Frame);
    //    joint_state.position.push_back(0.1);
    //    robot_state_publisher.publishTransforms();
    //    robot_state_publisher.publishTransforms(joint_state, model,);
    //    return true;
    //    QString filePath = QCoreApplication::applicationDirPath() + "/../../src/aubo_description/urdf/aubo_i5.urdf.xacro";
    std::string urdf_file = ros::package::getPath("eu_description") + "/urdf/eu_description.urdf";
    QDomDocument doc = readFromXML(QString::fromStdString(urdf_file));
    if(doc.isNull()) return false;
    std::string desc;
    ros::param::get(RobotDescription,desc);
    doc.setContent(QString::fromStdString(desc));
    if(doc.isNull()) return false;
    QDomElement rootElement = doc.documentElement(); //返回根元素

    QDomNodeList node1List = rootElement.elementsByTagName("link");
    for(int i = 0;i < node1List.count();++i)
    {
        QDomElement eleMent = node1List.at(i).toElement();
        //        qDebug()<<"**"<<eleMent.attribute("name");
        if("Empty_Link" == eleMent.attribute("name"))
        {
            //            qDebug()<<"找到了";
            QDomElement vOrign = eleMent.elementsByTagName("visual").at(0).toElement().elementsByTagName("origin").at(0).toElement();
            QDomElement vBox = eleMent.elementsByTagName("visual").at(0).toElement().elementsByTagName("geometry").at(0).toElement().elementsByTagName("box").at(0).toElement();
            QDomElement cOrign = eleMent.elementsByTagName("collision").at(0).toElement().elementsByTagName("origin").at(0).toElement();
            QDomElement cBox = eleMent.elementsByTagName("collision").at(0).toElement().elementsByTagName("geometry").at(0).toElement().elementsByTagName("box").at(0).toElement();
            vOrign.setAttribute("xyz",QString("0 0 %1").arg(-length / 1000.0 / 2.0));
            vBox.setAttribute("size",QString("0.01 0.01 %1").arg(length / 1000.0));
            cOrign.setAttribute("xyz",QString("0 0 %1").arg(-length / 1000.0 / 2.0));
            cBox.setAttribute("size",QString("0.01 0.01 %1").arg(length / 1000.0));
        }
    }
    QDomNodeList node2List = rootElement.elementsByTagName("joint");
    for(int i = 0;i < node2List.count();++i)
    {
        QDomElement eleMent = node2List.at(i).toElement();
        if("j7" == eleMent.attribute("name"))
        {
            QDomElement vOrign = eleMent.elementsByTagName("origin").at(0).toElement();
            vOrign.setAttribute("xyz",QString("0 0 %1").arg(length / 1000.0));
        }
    }
    return writeToXML(doc,QString::fromStdString(urdf_file));
}

ControlMode IPlanningManager::getControlWay() const
{
    return ControlMode_Simulation;
}

void IPlanningManager::setControlWay(ControlMode mode)
{
    if(ControlMode_Actual == mode)
        IMotorControl::instance()->publishRequest(RequestCmd_ActualArm);
    else if(ControlMode_Simulation == mode)
        IMotorControl::instance()->publishRequest(RequestCmd_Simulation);
}

void IPlanningManager::run()
{
    ros::Rate loop_rate(10);
    // 将专用线程标志设置为true
    m_buffer.setUsingDedicatedThread(true);
    tf2_ros::TransformListener listener(m_buffer);
    std::string endLink = m_group->getEndEffectorLink();
    while (ros::ok())
    {
        moveit::core::RobotStatePtr current_state = m_group->getCurrentState(10);
        if(current_state.get())
        {
            const moveit::core::JointModelGroup* model = current_state.get()->getJointModelGroup(Arm_Group);
            current_state.get()->copyJointGroupPositions(model,m_jointValues);
            current_state.get()->copyJointGroupVelocities(model,m_velValues);
            current_state.get()->copyJointGroupAccelerations(model,m_accValues);
            m_jointValues = Global::radianToAngle(m_jointValues);
            geometry_msgs::Pose p;
            const robot_model::LinkModel* lm = current_state.get()->getLinkModel(endLink);
            if(lm)
                p = tf2::toMsg(current_state.get()->getGlobalLinkTransform(lm));

            // 4.生成一个坐标点(相对于子级坐标系)
            geometry_msgs::PoseStamped point_laser;
            point_laser.header.frame_id = World_Frame;
            point_laser.header.stamp = ros::Time(0);
            point_laser.pose = p;
            //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
            try
            {
                geometry_msgs::PoseStamped point_base = m_buffer.transform(point_laser,m_planFram);
                m_targetPos = Global::geometry_msgs_PoseToTargetPose(point_base.pose);
            }
            catch(const std::exception& e)
            {
                emit AppEvent::instance()->errorOccured("程序异常，获取tf变换失败!");
                //                QLOG_WARN() << "程序异常,获取tf变换失败!";
            }
        }
        loop_rate.sleep();
    }
}
