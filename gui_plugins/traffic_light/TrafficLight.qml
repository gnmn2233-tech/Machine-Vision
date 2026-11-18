import QtQuick 2.9
import QtQuick.Layouts 1.3

Rectangle {
  color: "transparent"
  implicitWidth: 210
  implicitHeight: 58

  RowLayout {
    anchors.centerIn: parent
    spacing: 16

    // 红灯按钮：亮 = 当前就是红灯
    Rectangle {
      width: 38; height: 38; radius: 19
      color: TrafficLight.lightColor === "red" ? "#ff2118" : "#4a4a4a"
      border.width: TrafficLight.lightColor === "red" ? 3 : 0
      border.color: "#ffffff"
      MouseArea {
        anchors.fill: parent
        cursorShape: Qt.PointingHandCursor
        onClicked: TrafficLight.SetRed()
      }
    }

    // 绿灯按钮
    Rectangle {
      width: 38; height: 38; radius: 19
      color: TrafficLight.lightColor === "green" ? "#16e83c" : "#4a4a4a"
      border.width: TrafficLight.lightColor === "green" ? 3 : 0
      border.color: "#ffffff"
      MouseArea {
        anchors.fill: parent
        cursorShape: Qt.PointingHandCursor
        onClicked: TrafficLight.SetGreen()
      }
    }

    Text {
      text: TrafficLight.lightColor === "red" ? "STOP" : "GO"
      color: "#2a2a2a"
      font.pixelSize: 17
      font.bold: true
    }
  }
}
