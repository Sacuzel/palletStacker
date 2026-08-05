import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Rectangle {
  color: "transparent"
  anchors.fill: parent

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 10
    spacing: 8

    Label {
      Layout.fillWidth: true
      text: "Forklift keyboard control"
      font.bold: true
      font.pixelSize: 16
    }

    Label {
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      text: ForkliftTeleop.controlsText
    }

    Label {
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      text: ForkliftTeleop.statusText
      font.bold: true
    }

    Rectangle {
      Layout.fillWidth: true
      height: 1
      color: "#808080"
    }

    GridLayout {
      Layout.fillWidth: true
      columns: 2
      columnSpacing: 12
      rowSpacing: 5

      Label { text: "Linear velocity" }
      Label {
        Layout.alignment: Qt.AlignRight
        text: ForkliftTeleop.linearVelocity.toFixed(2) + " m/s"
        font.family: "monospace"
      }

      Label { text: "Turn velocity" }
      Label {
        Layout.alignment: Qt.AlignRight
        text: ForkliftTeleop.angularVelocity.toFixed(2) + " rad/s"
        font.family: "monospace"
      }

      Label { text: "Fork target" }
      Label {
        Layout.alignment: Qt.AlignRight
        text: ForkliftTeleop.forkTarget.toFixed(3) + " m"
        font.family: "monospace"
      }
    }

    Button {
      Layout.alignment: Qt.AlignHCenter
      text: "Stop chassis"
      onClicked: ForkliftTeleop.Stop()
    }

    Label {
      Layout.fillWidth: true
      Layout.fillHeight: true
      verticalAlignment: Text.AlignBottom
      wrapMode: Text.WordWrap
      text: "Fork control uses a position target. Releasing an arrow key " +
            "keeps the last target, so the forks hold their selected height."
      opacity: 0.8
    }
  }
}
