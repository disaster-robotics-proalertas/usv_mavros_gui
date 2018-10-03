#ifndef MAPMAVROSPIPE_H
#define MAPMAVROSPIPE_H

#include <QThread>
#include <QGeoCoordinate>
#include <QVariant>

#include "include/waypointcontrol.h"

class MapMavrosPipe : public QThread
{
    Q_OBJECT
public:
    explicit MapMavrosPipe(QObject *parent = nullptr);

    WaypointControl mavrosControl;

protected:
    QGeoCoordinate home;
signals:

public slots:
    void waypointsPipeToROS(const QString &v);
    void homeLocationPipeToROS(const QString &h);
};

#endif // MAPMAVROSPIPE_H
