// import QtQuick 1.0 // to target S60 5th Edition or Maemo 5
import QtQuick 1.1
import QtMobility.location 1.2
import FileIO 1.0


Rectangle {
    id: container
    width: 1000
    height: 600
    focus: true

    property int elapsedSec:0;

    Map {
        id: map
        anchors.fill: parent
        plugin : Plugin {
            name : "nokia"
        }



        // DEFAULT ZOOM AND CENTER
        zoomLevel: 5
        center: Coordinate {latitude: 54.911886; longitude: 9.782295}


        //TARGET POINT
        MapCircle {
             id: targetPointMarker
             center: Coordinate {latitude: 54.9124628; longitude: 9.7835719}
             radius: 20
             color: "green"
             border.width: 3
             border.color: "white"
         }

        //START POINT
        MapCircle {
             id: startPointMarker
             center: Coordinate {latitude: 54.911886; longitude: 9.782295}
             radius: 5
             color: "orange"
             border.width: 2
             border.color: "white"
             visible: false
         }

        //BOAT MARKER
        MapCircle {
             id: boatMarker
             center: Coordinate {latitude: 54.911886; longitude: 9.782295}
             radius: 3
             color: "blue"
             border.width: 2
             border.color: "white"
         }


        // POLYLINE
        MapPolyline {
             id: polyline
             border {color: "blue"; width: 3}
         }


        //MOUSE INTERACTION
        MapMouseArea {
             property int lastX : -1
             property int lastY : -1
             property int dragged: 0

             onPressed : {
                 lastX = mouse.x
                 lastY = mouse.y
                 dragged=0
             }
             onReleased : {
                 lastX = -1
                 lastY = -1
                 if(dragged==0) {
                     targetPointMarker.center = mouse.coordinate
                     //write to file
                     myFile.source="/tmp/sailboat/Point_End_Lat"
                     myFile.write(roundVal(targetPointMarker.center.latitude,1000000));
                     myFile.source="/tmp/sailboat/Point_End_Lon"
                     myFile.write(roundVal(targetPointMarker.center.longitude,1000000));
                 }
             }
             onPositionChanged: {
                 if (mouse.button == Qt.LeftButton) {
                     if ((lastX != -1) && (lastY != -1)) {
                         var dx = mouse.x - lastX
                         var dy = mouse.y - lastY
                         map.pan(-dx, -dy)
                     }
                     lastX = mouse.x
                     lastY = mouse.y
                     dragged=1
                 }
             }
             onDoubleClicked: {
                 map.center = mouse.coordinate
                 map.zoomLevel += 1
                 lastX = -1
                 lastY = -1
            }
            //onClicked: {
            //    targetPointMarker.center = mouse.coordinate
            //}
         }

         //mouse_area
    } //map



    Radar{
        id: myradar
        x: 450
        y: 194
        anchors.right: parent.right
        anchors.rightMargin: 0
        anchors.verticalCenter: parent.verticalCenter

    }


    /*
     *   TOP PANEL
     */
    Rectangle {
        id: top_panel
        height: 68
        color: "#dc000000"
        anchors.top: parent.top
        anchors.topMargin: -70
        anchors.left: parent.left
        anchors.leftMargin: 0
        anchors.right: parent.right
        anchors.rightMargin: 0
        z: 3
        property int ishidden: 1


        Row {
            id: top_row
            spacing: 5
            anchors.fill: parent

            Item {
                id: e1
                x: 105
                width: 150
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0
                anchors.top: parent.top
                anchors.topMargin: 0

                Text {
                    id: e1_label
                    x: 18
                    y: 5
                    color: "#c8c8c8"
                    text: qsTr("Navigation System")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.family: "DejaVu Sans"
                    font.bold: false
                    font.pixelSize: 12
                }

                Text {
                    id: navsys
                    x: 48
                    y: 29
                    color: "#ffffff"
                    text: qsTr("IDLE")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    horizontalAlignment: Text.AlignHCenter
                    font.bold: false
                    font.pixelSize: 24
                }
            }

            Item {
                id: e2
                x: 105
                width: 150
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0
                anchors.top: parent.top
                anchors.topMargin: 0

                Text {
                    id: e2_label
                    x: 48
                    y: 5
                    color: "#c8c8c8"
                    text: qsTr("Start point")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.family: "DejaVu Sans"
                    font.bold: false
                    font.pixelSize: 10
                }

                Text {
                    id: start_lat
                    x: 34
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("0.000000 N")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    horizontalAlignment: Text.AlignHCenter
                    font.bold: false
                    font.pixelSize: 14
                }

                Text {
                    id: start_lon
                    x: 35
                    y: 43
                    color: "#c8c8c8"
                    text: qsTr("0.000000 E")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 14
                }
            }


            Item {
                id: e3
                x: 105
                width: 150
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0
                anchors.top: parent.top
                anchors.topMargin: 0

                Text {
                    id: e3_label
                    x: 48
                    y: 5
                    color: "#c8c8c8"
                    text: qsTr("Current point")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.family: "DejaVu Sans"
                    font.bold: false
                    font.pixelSize: 10
                }

                Text {
                    id: curr_lat
                    x: 34
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("0.000000 N")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    horizontalAlignment: Text.AlignHCenter
                    font.bold: false
                    font.pixelSize: 14
                }

                Text {
                    id: curr_lon
                    x: 35
                    y: 43
                    color: "#c8c8c8"
                    text: qsTr("0.000000 E")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 14
                }
            }

            Item {
                id: e4
                x: 105
                width: 150
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0
                anchors.top: parent.top
                anchors.topMargin: 0

                Text {
                    id: e4_label
                    x: 48
                    y: 5
                    color: "#c8c8c8"
                    text: qsTr("Target point")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.family: "DejaVu Sans"
                    font.bold: false
                    font.pixelSize: 10
                }

                Text {
                    id: target_lat
                    x: 34
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("0.000000 N")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    horizontalAlignment: Text.AlignHCenter
                    font.bold: false
                    font.pixelSize: 14
                }

                Text {
                    id: target_lon
                    x: 35
                    y: 44
                    color: "#c8c8c8"
                    text: qsTr("0.000000 E")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 14
                }
            }

            Item {
                id: e5
                x: 675
                width: 105
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0
                anchors.top: parent.top
                anchors.topMargin: 0

                Text {
                    id: e5_label
                    x: 31
                    y: 6
                    color: "#c8c8c8"
                    text: qsTr("Distance")
                    font.pixelSize: 10
                }

                Text {
                    id: distanceVal
                    x: 46
                    y: 23
                    color: "#c8c8c8"
                    text: qsTr("0000")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22
                }

                Text {
                    id: text5
                    x: 35
                    y: 51
                    color: "#b4b4b4"
                    text: qsTr("meters")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 9
                }
            }

            Item {
                id: e6
                x: 675
                width: 105
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0
                anchors.top: parent.top
                anchors.topMargin: 0

                Text {
                    id: labelDuration
                    x: 32
                    y: 6
                    color: "#c8c8c8"
                    text: qsTr("Duration")
                    font.pixelSize: 10
                }

                Text {
                    id: durationText
                    x: 21
                    y: 23
                    color: "#c8c8c8"
                    text: qsTr("00:00")
                    horizontalAlignment: Text.AlignHCenter
                    font.pixelSize: 22
                }

                Text {
                    id: text6
                    x: 31
                    y: 51
                    color: "#b4b4b4"
                    text: qsTr("min : sec")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 9
                }
            }


            Item {
                id: e7
                x: 105
                width: 150
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 0
                anchors.top: parent.top
                anchors.topMargin: 0

                Text {
                    id: e7_label
                    x: 29
                    y: 5
                    color: "#c8c8c8"
                    text: qsTr("Manual Control")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.family: "DejaVu Sans"
                    font.bold: false
                    font.pixelSize: 12
                }

                Text {
                    id: mancon
                    x: 51
                    y: 29
                    color: "#ffffff"
                    text: qsTr("OFF")
                    anchors.horizontalCenterOffset: 0
                    anchors.horizontalCenter: parent.horizontalCenter
                    horizontalAlignment: Text.AlignHCenter
                    font.bold: false
                    font.pixelSize: 24
                }
            }


        } //top_row

        Behavior on anchors.topMargin { SmoothedAnimation { velocity: 150 } }

    } //top_panel




    /*
     *   BOTTOM PANEL
     */
    Rectangle {
        id: bottom
        x: 644
        y: 412
        height: 68
        color: "#dc000000"
        anchors.right: parent.right
        anchors.rightMargin: 0
        anchors.left: parent.left
        anchors.leftMargin: 0
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 0

        Row {
            id: row_bottom
            anchors.rightMargin: 6
            anchors.leftMargin: 6
            spacing: 10
            anchors.fill: parent



            Item {
                id: w1
                width: 80
                height: parent.height

                Text {
                    id: w1_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Wind Speed")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: wind_speed
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    //font.family: "URW Gothic L"
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w1_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("m/s")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }


            Item {
                id: w2
                width: 80
                height: parent.height

                Text {
                    id: w2_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Wind Direction")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: wind_angle
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w2_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }

                Image {
                    id: arrow_wind
                    x: 0
                    y: 0
                    width: 60
                    height: 60
                    visible: false
                    rotation: -3
                    opacity: 0.250
                    z: -1
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    source: "arrow2.png"
                }
            }


            Item {
                id: w3
                width: 80
                height: parent.height

                Text {
                    id: w3_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Heading")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: heading
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w3_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }


                Image {
                    id: arrow_heading
                    x: 0
                    y: 0
                    width: 60
                    height: 60
                    visible: false
                    rotation: 70
                    opacity: 0.250
                    z: -1
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    source: "arrow2.png"
                }
            }

            Item {
                id: w4
                width: 80
                height: parent.height

                Text {
                    id: w4_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Deviation")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: deviation
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w4_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }

            Item {
                id: w5
                width: 80
                height: parent.height

                Text {
                    id: w5_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Variation")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: variation
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w5_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }


            Item {
                id: w6
                width: 80
                height: parent.height

                Text {
                    id: w6_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Rate of Turn")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: rate
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w6_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg/s")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }



            Item {
                id: w7
                width: 80
                height: parent.height

                Text {
                    id: w7_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Yaw")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: yaw
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w7_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }



            Item {
                id: w8
                width: 80
                height: parent.height

                Text {
                    id: w8_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Pitch")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: pitch
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w8_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }



            Item {
                id: w9
                width: 80
                height: parent.height

                Text {
                    id: w9_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("Roll")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: roll
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w9_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }


            Item {
                id: w10
                width: 80
                height: parent.height

                Text {
                    id: w10_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("COG")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: cog
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w10_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("deg")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }



            Item {
                id: w11
                width: 80
                height: parent.height

                Text {
                    id: w11_label
                    x: 10
                    y: 9
                    color: "#c8c8c8"
                    text: qsTr("SOG")
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 10
                }

                Text {
                    id: sog
                    x: 15
                    y: 25
                    color: "#c8c8c8"
                    text: qsTr("--")
                    anchors.horizontalCenterOffset: 0
                    horizontalAlignment: Text.AlignHCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    font.pixelSize: 22

                    Text {
                        id: w11_unit
                        x: 13
                        y: 26
                        color: "#b4b4b4"
                        text: qsTr("Km/h")
                        anchors.horizontalCenterOffset: 0
                        anchors.horizontalCenter: parent.horizontalCenter
                        font.pixelSize: 9
                    }
                }
            }





        } //row_bottom

    } //bottom







    //KEYBOARD INTERACTION
    Keys.onPressed: {
         //console.log("Key pressed: "+event.key);


         // MAP ZOOM IN [+][43]
         if (event.key === Qt.Key_Plus) {
             if(map.zoomLevel<20) map.zoomLevel += 1;
             event.accepted = true;
         }

         // MAP ZOOM OUT [-][45]
         if (event.key === Qt.Key_Minus) {
             if(map.zoomLevel>3) map.zoomLevel -= 1;
             event.accepted = true;
         }

         // MAP ZOOM MAX AND CENTER ON THE BOAT [9][57]
         if (event.key === Qt.Key_9) {
             map.zoomLevel = 18;
             //map.pan(boatMarker.center.latitude,boatMarker.center.longitude);
             //map.center = boatMarker.center;
             //map.zoomLevel += 1
             map.center.latitude=boatMarker.center.latitude;
             map.center.longitude=boatMarker.center.longitude;
             event.accepted = true;
         }

         // SHOW ARROWS [A][65]
         if (event.key === Qt.Key_A) {
             if(arrow_wind.visible) {arrow_wind.visible=false; arrow_heading.visible=false;}
             else  {arrow_wind.visible=true; arrow_heading.visible=true;}

         }

         // SHOW UPPER PANEL [ESC][16777216]
         if (event.key === Qt.Key_Escape) {
             if (top_panel.ishidden) {
                top_panel.anchors.topMargin= 0;
                top_panel.ishidden=0;
             }
             else {
                top_panel.anchors.topMargin= -70;
                top_panel.ishidden=1;
             }
         }

         // START/STOP NAVIGATION SYSTEM [S][83] (to_do: don't check item visibility, check file value)
         if (event.key === Qt.Key_S) {
             if(!startPointMarker.visible)
                {
                    // TURN ON
                    myFile.source="/tmp/sailboat/Point_Start_Lat"
                    myFile.write(boatMarker.center.latitude);
                    myFile.source="/tmp/sailboat/Point_Start_Lon"
                    myFile.write(boatMarker.center.longitude);
                    startPointMarker.center.latitude=boatMarker.center.latitude;
                    startPointMarker.center.longitude=boatMarker.center.longitude;
                    startPointMarker.visible=true;

                    myFile.source="/tmp/sailboat/Navigation_System"
                    myFile.write("1");
                    navsys.text="ON"

                    elapsedSec=0;
                }
             else
                {
                    // TURN OFF
                    startPointMarker.visible=false;
                    myFile.source="/tmp/sailboat/Point_Start_Lat"
                    myFile.write("0");
                    myFile.source="/tmp/sailboat/Point_Start_Lon"
                    myFile.write("0");
                    myFile.source="/tmp/sailboat/Navigation_System"
                    myFile.write("0");
                    navsys.text="IDLE"
                }
            }


         // START/STOP MANUAL CONTROL [M][77] (to_do: don't check item visibility, check file value)
         if (event.key === Qt.Key_M) {

             if(!mcslider1.visible)
                {
                    // TURN ON

                    // set Manual_Control_Rudder = Rudder_Feedback
                    rudder_rct.text=roundVal(myradar.rudder,0.2);
                    sail_rct.text=0;

                    // check max values
                    if(rudder_rct.text >  60) {rudder_rct.text =  60;}
                    if(rudder_rct.text < -60) {rudder_rct.text = -60;}
                    if(sail_rct.text >  60) {sail_rct.text =  60;}
                    if(sail_rct.text < -60) {sail_rct.text = -60;}


                    myFile.source="/tmp/sailboat/Manual_Control"
                    myFile.write("1");

                    myFile.source="/tmp/sailboat/Manual_Control_Rudder"
                    myFile.write(rudder_rct.text);

                    mcslider1.visible=true;
                    mcslider2.visible=true;
                    mancon.text="ON"
                }
             else
                {
                    // TURN OFF
                    mcslider1.visible=false;
                    mcslider2.visible=false;
                    myFile.source="/tmp/sailboat/Manual_Control"
                    myFile.write("0");
                    mancon.text="OFF"
                }

            }

        // MANUAL CONTROL: ARROW UP
        if (event.key === Qt.Key_Up) {
            if(sail_rct.text < 60) {
                sail_rct.text=(sail_rct.text*1)+5;
                if(sail_rct.text > 60) sail_rct.text=60;
                // WRITE TO FILE
            }
        }

        // MANUAL CONTROL: ARROW DOWN
        if (event.key === Qt.Key_Down) {
            if(sail_rct.text > -60) {
                sail_rct.text=(sail_rct.text*1)-5;
                if(sail_rct.text < -60) sail_rct.text=-60;
                // WRITE TO FILE
            }
        }

        // MANUAL CONTROL: ARROW RIGTH
        if (event.key === Qt.Key_Right) {
            if(rudder_rct.text < 60) {
                rudder_rct.text=(rudder_rct.text*1)+5;
                if(rudder_rct.text > 60) rudder_rct.text=60;
                // WRITE TO FILE
                myFile.source="/tmp/sailboat/Manual_Control_Rudder"
                myFile.write(rudder_rct.text);
            }
        }


        // MANUAL CONTROL: ARROW DOWN
        if (event.key === Qt.Key_Left) {
            if(rudder_rct.text > -60) {
                rudder_rct.text=(rudder_rct.text*1)-5;
                if(rudder_rct.text < -60) rudder_rct.text=-60;
                // WRITE TO FILE
                myFile.source="/tmp/sailboat/Manual_Control_Rudder"
                myFile.write(rudder_rct.text);
            }
        }

/*
        // CLEAR POLYLINE [C][..]
        if (event.key === Qt.Key_C) {
            for (var index = 0; index < polyline.path.length; index++)  {
                    console.log("Index, latitude:" + index + " , " + polyline.path[index].latitude);
                    polyline.removeCoordinate(polyline.path[index]);
                }
        }
*/

     }






    // FILE I/O
    FileIO {
        id: myFile
        source: "/home/mix/Downloads/my_file.txt"
        onError: console.log(msg)
    }



    Component.onCompleted: {
        //console.log( "WRITE: "+ myFile.write("valore"));
        //console.log( "READ: " + myFile.read());
        // wait for the component to be completed, then start the main loop
        mainloop.running=true;
    }



    // MAIN LOOP
    Timer {
            id: mainloop;
            interval: 1000; running: false; repeat: true
            onTriggered: {

                // check for U200 files existence
                myFile.source="/tmp/u200/Heading"
                if(myFile.read()) {

                    myFile.source="/tmp/u200/Wind_Speed"
                    //if(myFile.read() != 0) { wind_speed.text=myFile.read(); } else { wind_speed.text="--"; }
                    wind_speed.text=roundVal(myFile.read(),100); if(wind_speed.text==0) { wind_speed.text="--"; }

                    myFile.source="/tmp/u200/Wind_Angle"
                    wind_angle.text=roundVal(myFile.read(),10); if(wind_angle.text==0) { wind_angle.text="--"; }

                    myFile.source="/tmp/u200/Heading"
                    heading.text=roundVal(myFile.read(),10); if(heading.text==0) { heading.text="--"; }

                    myFile.source="/tmp/u200/Deviation"
                    deviation.text=roundVal(myFile.read(),100); if(deviation.text==0) { deviation.text="--"; }

                    myFile.source="/tmp/u200/Variation"
                    variation.text=roundVal(myFile.read(),100); if(variation.text==0) { variation.text="--"; }

                    myFile.source="/tmp/u200/Rate"
                    rate.text=roundVal(myFile.read(),100); if(rate.text==0) { rate.text="--"; }

                    myFile.source="/tmp/u200/Yaw"
                    yaw.text=roundVal(myFile.read(),100); if(yaw.text==0) { yaw.text="--"; }

                    myFile.source="/tmp/u200/Pitch"
                    pitch.text=roundVal(myFile.read(),100); if(pitch.text==0) { pitch.text="--"; }

                    myFile.source="/tmp/u200/Roll"
                    roll.text=roundVal(myFile.read(),100); if(roll.text==0) { roll.text="--"; }

                    myFile.source="/tmp/u200/COG"
                    cog.text=roundVal(myFile.read(),10); if(cog.text==0) { cog.text="--"; }

                    myFile.source="/tmp/u200/SOG"
                    sog.text=roundVal(myFile.read(),10); if(sog.text==0) { sog.text="--"; }



                    //MAP: update BoatMarker position
                    myFile.source="/tmp/u200/Latitude"
                    boatMarker.center.latitude=myFile.read();
                    myFile.source="/tmp/u200/Longitude"
                    boatMarker.center.longitude=myFile.read();

                    //TopPanel: Start position info
                    myFile.source="/tmp/sailboat/Point_Start_Lat"
                    start_lat.text=roundVal(myFile.read(),1000000)+" N";
                    myFile.source="/tmp/sailboat/Point_Start_Lon"
                    start_lon.text=roundVal(myFile.read(),1000000)+" E";

                    //TopPanel: Current position info
                    curr_lat.text=roundVal(boatMarker.center.latitude,1000000)+" N";
                    curr_lon.text=roundVal(boatMarker.center.longitude,1000000)+" E";

                    //TopPanel: Target position info
                    target_lat.text=roundVal(targetPointMarker.center.latitude,1000000)+" N";
                    target_lon.text=roundVal(targetPointMarker.center.longitude,1000000)+" E";

                    //TopPanel: Distance
                    distanceVal.text=roundVal(
                                        targetDistance( boatMarker.center.latitude,
                                                        boatMarker.center.longitude,
                                                        targetPointMarker.center.latitude,
                                                        targetPointMarker.center.longitude)
                                    ,1);



                    //Radar: Wind
                    if(wind_angle.text!="--") myradar.red=wind_angle.text;
                    if(wind_angle.text!="--") myradar.nogo=wind_angle.text;

                    //Radar: Boat
                    if(heading.text!="--") myradar.boat=heading.text;

                    //Radar: CALCULATE direction to TARGET POINT
                    myradar.green=targetHeading(boatMarker.center.latitude,
                                                boatMarker.center.longitude,
                                                targetPointMarker.center.latitude,
                                                targetPointMarker.center.longitude);


                    // BottomPanel: rotate arrow_heading and arrow_wind
                    if(wind_angle.text!="--") arrow_wind.rotation=wind_angle.text;
                    if(heading.text!="--") arrow_heading.rotation=heading.text;


                    //DRAW POLYLINE if NavSys ON
                    if(startPointMarker.visible) {
                    polyline.addCoordinate(boatMarker.center);
                    }


                }
                else
                {
                    console.log( "waiting for /tmp/u200 to be populated.. ");
                }


                // check for SAILBOAT files existence
                myFile.source="/tmp/sailboat/Navigation_System"
                if(myFile.read()) {

                    myFile.source="/tmp/sailboat/Navigation_System"


                    myFile.source="/tmp/sailboat/Manual_Control"


                    myFile.source="/tmp/sailboat/Rudder_Feedback"
                    myradar.rudder=myFile.read();


                }
                else
                {
                    console.log( "waiting for /tmp/sailboat to be populated.. ");
                }


                //STOPWATCH
                if(startPointMarker.visible) {
                    elapsedSec=elapsedSec+1;
                    durationText.text=calcDuration(elapsedSec);
                }


            }
    }

    Rectangle {
        id: mcslider1
        x: 710
        y: 463
        width: 132
        height: 34
        color: "#dc000000"
        visible: false

        Text {
            id: rudder_label
            x: 11
            y: 8
            color: "#c8c8c8"
            text: qsTr("Rudder")
            font.bold: false
            font.pixelSize: 14
        }

        Text {
            id: rudder_rct
            x: 77
            y: 6
            color: "#ffffff"
            text: qsTr("0")
            font.bold: true
            horizontalAlignment: Text.AlignRight
            font.pixelSize: 18
        }

        Text {
            id: rudder_unit
            x: 111
            y: 6
            color: "#ffffff"
            text: qsTr("°")
            font.bold: true
            font.pixelSize: 18
        }
    }

    Rectangle {
        id: mcslider2
        x: 860
        y: 463
        width: 132
        height: 34
        color: "#dc000000"
        visible: false

        Text {
            id: sail_label
            x: 14
            y: 8
            color: "#c8c8c8"
            text: qsTr("Sail")
            font.bold: false
            font.pixelSize: 14
        }

        Text {
            id: sail_rct
            x: 79
            y: 6
            width: 20
            height: 22
            color: "#ffffff"
            text: qsTr("0")
            font.bold: true
            horizontalAlignment: Text.AlignRight
            font.pixelSize: 18
        }

        Text {
            id: sail_unit
            x: 101
            y: 6
            color: "#ffffff"
            text: qsTr("°")
            font.bold: true
            font.pixelSize: 18
        }
    }


    function calcDuration(elap)
        {
            var res,min,sec;
            min=Math.floor(elap / 60);
            sec=elap - (min * 60);

            if (sec<10) { sec="0"+sec; };
            if (min<10) { min="0"+min; };

            res=min+":"+sec;
            return res;
        }


    function roundVal(val,pre)
        {
            var res;
            res=Math.round(val*pre)/pre;
            return res;
        }


    function targetHeading(boat_lat,boat_lon,target_lat,target_lon)
        {
            //var w=Math.sqrt(Math.pow((sx2-sx1),2)+Math.pow((sy2-sy1),2));
            //console.debug("W: "+w);
            var x,y;
            x=target_lon-boat_lon;
            y=target_lat-boat_lat;

            var res=Math.atan2(x,y)*180/Math.PI;
            return res;
        }

    function targetDistance(boat_lat,boat_lon,target_lat,target_lon)
        {
            //var w=Math.sqrt(Math.pow((sx2-sx1),2)+Math.pow((sy2-sy1),2));
            //console.debug("W: "+w);

            var earthRadius=6371*1000; // meters
/*
            var dlon = boat_lon - target_lon;
            var dlat = boat_lat - target_lat;
            var a = Math.pow((Math.sin(dlat/2)),2) + Math.cos(target_lat) * Math.cos(boat_lat) * Math.pow((Math.sin(dlon/2)),2);
            var c = 2 * Math.atan2( Math.sqrt(a), Math.sqrt(1-a) );
            var res = earthRadius * c;
*/
        /*
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = (sin(dlat/2))^2 + cos(lat1) * cos(lat2) * (sin(dlon/2))^2
        c = 2 * atan2( sqrt(a), sqrt(1-a) )
        d = R * c (where R is the radius of the Earth)
        */
/*
        var R = 6371; // km
        var dLat = (lat2-lat1).toRad();
        var dLon = (lon2-lon1).toRad();
        var lat1 = lat1.toRad();
        var lat2 = lat2.toRad();

        var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                Math.sin(dLon/2) * Math.sin(dLon/2) * Math.cos(lat1) * Math.cos(lat2);
        var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
        var d = R * c;
*/
        //1:target
        //2:boat

            var dLat = (boat_lat-target_lat)* Math.PI / 180;
            var dLon = (boat_lon-target_lon)* Math.PI / 180;
            var lat1 = target_lat * Math.PI / 180;
            var lat2 = boat_lat * Math.PI / 180;

            var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                    Math.sin(dLon/2) * Math.sin(dLon/2) * Math.cos(lat1) * Math.cos(lat2);
            var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
            var res = earthRadius * c;


            return res;
        }


}
