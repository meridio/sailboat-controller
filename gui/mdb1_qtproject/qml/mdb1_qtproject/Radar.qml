// import QtQuick 1.0 // to target S60 5th Edition or Maemo 5
import QtQuick 1.1

Item {
    id: item1
    width: 300
    height: 360

    property alias green: green.rotation
    property alias red: red.rotation
    property alias nogo: nogo.rotation
    property alias boat: boat.rotation
    property alias rudder: boat.rudder

    Boat {
        id: boat
        x: 0
        y: 30
        anchors.verticalCenterOffset: 0
        anchors.horizontalCenterOffset: 0
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        Behavior on rotation { SmoothedAnimation { velocity: 150 } }
    }


    Image {
        id: rings
        x: 0
        y: 20
        z: 1
        width: 300
        height: 300
        opacity: 0.250
        source: "radar_rings.png"
    }

    Image {
        id: letters
        x: 0
        y: 20
        z: 2
        width: 300
        height: 300
        source: "radar_letters.png"
    }

    Image {
        id: nogo
        x: 0
        y: 20
        z: 3
        width: 300
        height: 300
        opacity: 0.200
        visible: true
        source: "radar_nogo.png"
        Behavior on rotation { SmoothedAnimation { velocity: 150 } }
    }

    Image {
        id: red
        x: 0
        y: 20
        z: 4
        width: 300
        height: 300
        source: "radar_red.png"
        Behavior on rotation { SmoothedAnimation { velocity: 150 } }
    }

    Image {
        id: green
        x: 0
        y: 20
        z: 5
        rotation: 0
        width: 300
        height: 300
        source: "radar_green.png"
        Behavior on rotation { SmoothedAnimation { velocity: 150 } }
    }



}
