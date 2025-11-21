#include "idriver.h"
#include <QSettings>
#include <QCoreApplication>

QMap<int, float> IDriver::motorPosMap;
QMap<int, QMap<int, bool>> IDriver::dataResult;
std::vector<float> IDriver::m_Offsets = {0,0,0,0,0,0};
QMutex IDriver::motorPosMutex;

IDriver::IDriver(QObject *parent) : QObject(parent)
{
    m_configIni = new QSettings(QCoreApplication::applicationDirPath() + "/config/global.ini",QSettings::IniFormat,this);
    m_configIni->beginGroup("MotorOffset");
    for(std::vector<float>::size_type index = 0;index < 6;++index)
        m_Offsets[index] = m_configIni->value(QString("Joint%1").arg(index),0).toFloat();
    m_configIni->sync();
    m_configIni->endGroup();
}

std::vector<float> IDriver::getPositionOffset() const
{
    return m_Offsets;
}

void IDriver::setPositionOffset(const std::vector<float> &offsets)
{
    m_Offsets = offsets;
    m_configIni->beginGroup("MotorOffset");
    for(std::vector<float>::size_type index = 0;index < 6;++index)
        m_configIni->setValue(QString("Joint%1").arg(index),m_Offsets[index]);
    m_configIni->sync();
    m_configIni->endGroup();
}

void IDriver::setPositionOffset(int id, float offset)
{
    m_Offsets[static_cast<std::vector<float>::size_type>(id - 1)] = offset;
    return setPositionOffset(m_Offsets);
}
