#include <QMessageBox>
#include <QApplication>
#include "subscribe.h"
#include "canwaysendtrajectory.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    ros::init(argc, argv, "eu_driver_planet");
    if(!ros::master::check())
    {
        QMessageBox::information(nullptr,"提示","主机不存在(http://192.168.137.37:11311/)",QMessageBox::Yes);
        return 0;
    }

    Subscribe subscribe;

    ros::spin();
    return app.exec();
}
