TARGET = qml_location_mapviewer
TEMPLATE = app

QT += qml network quick positioning location
CONFIG += qtquickcompiler
SOURCES += src/main.cpp \
    src/mapmavrospipe.cpp \
    src/waypointcontrol.cpp

RESOURCES += \
    mapviewer.qrc

OTHER_FILES +=mapviewer.qml \
    helper.js \
    map/MapComponent.qml \
    map/MapSliders.qml \
    map/Marker.qml \
    map/CircleItem.qml \
    map/RectangleItem.qml \
    map/PolylineItem.qml \
    map/PolygonItem.qml \
    map/ImageItem.qml \
    map/MiniMap.qml \
    menus/ItemPopupMenu.qml \
    menus/MainMenu.qml \
    menus/MapPopupMenu.qml \
    menus/MarkerPopupMenu \
    forms/Geocode.qml \
    forms/GeocodeForm.ui.qml\
    forms/Message.qml \
    forms/MessageForm.ui.qml \
    forms/ReverseGeocode.qml \
    forms/ReverseGeocodeForm.ui.qml \
    forms/RouteCoordinate.qml \
    forms/Locale.qml \
    forms/LocaleForm.ui.qml \
    forms/RouteAddress.qml \
    forms/RouteAddressForm.ui.qml \
    forms/RouteCoordinateForm.ui.qml \
    forms/RouteList.qml \
    forms/RouteListDelegate.qml \
    forms/RouteListHeader.qml

target.path = $$[QT_INSTALL_EXAMPLES]/location/mapviewer
INSTALLS += target

HEADERS += \
    include/mapmavrospipe.h \
    include/waypointcontrol.h

DISTFILES += \
    menus/MarkerPopupMenu.qml \
    map/HomeMarker.qml
