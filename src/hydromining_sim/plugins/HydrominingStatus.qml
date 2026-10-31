import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
Rectangle {
  color: "#d0222a32"
  anchors.fill: parent
  Layout.minimumWidth: 300
  Layout.minimumHeight: 150
  Text {
    anchors.fill: parent
    anchors.margins: 12
    text: HydrominingStatus.status
    color: "white"
    font.pixelSize: 15
    wrapMode: Text.WordWrap
  }
}
