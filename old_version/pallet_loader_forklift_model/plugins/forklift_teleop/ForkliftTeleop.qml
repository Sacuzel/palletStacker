import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3

ColumnLayout {
  anchors.fill: parent
  anchors.margins: 10
  spacing: 6

  Label {
    text: "Forklift controls"
    font.bold: true
    font.pixelSize: 16
  }
  Label { text: "↑ / ↓   Forward / reverse" }
  Label { text: "← / →   Counterclockwise / clockwise" }
  Label { text: "Shift + ↑ / ↓   Forks up / down" }
  Label { text: "Space   Emergency stop" }
  Label {
    Layout.fillWidth: true
    wrapMode: Text.WordWrap
    text: "Click the 3D viewport once before driving. Controls are press-and-hold."
  }
  Item { Layout.fillHeight: true }
}
