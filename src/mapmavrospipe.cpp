#include "include/mapmavrospipe.h"
#include <QDebug>
#include <QQuickItem>
#include <iostream>
#include <sstream>

MapMavrosPipe::MapMavrosPipe(QObject *parent) : QThread(parent),
    mavrosControl()
{

}

void MapMavrosPipe::waypointsPipeToROS(const QString &v)
{
    qDebug() << "Waypoints";
    std::cout << v.toStdString();
    mavrosControl.loadFromString(v.toStdString());
}

void MapMavrosPipe::homeLocationPipeToROS(const QString &h)
{
    qDebug() << "Home Location";
    std::cout << h.toStdString();
    std::stringstream s(h.toStdString());
    double la;
    s >> la;
    double lo;
    s >> lo;

    mavrosControl.setHome(la,lo);
}
