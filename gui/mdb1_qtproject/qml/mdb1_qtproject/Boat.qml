// import QtQuick 1.0 // to target S60 5th Edition or Maemo 5
import QtQuick 1.1

Item {
    width: 300
    height: 300

    property alias rudder: rudder.rotation

    Image {
        id: boat
        x: 1
        y: 0
        width: 300
        height: 300
        source: "radar_boat.png"
        Behavior on rotation { SmoothedAnimation { velocity: 150 } }
    }

    Image {
        id: rudder
        x: 131
        y: 188
        width: 42
        height: 42
        source: "radar_rudder.png"
        Behavior on rotation { SmoothedAnimation { velocity: 150 } }
    }

}
