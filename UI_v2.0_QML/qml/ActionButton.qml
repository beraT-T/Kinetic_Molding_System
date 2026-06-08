import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Button {
    id: btn
    property color color: "#3b82f6"
    Layout.fillWidth: true
    implicitHeight: 44

    background: Rectangle {
        radius: 8
        color: btn.enabled ? btn.color : Qt.darker(btn.color, 2.0)
        opacity: btn.enabled ? (btn.pressed ? 0.8 : 1.0) : 0.4
    }
    contentItem: Text {
        text: btn.text
        color: "white"
        font.bold: true
        font.pixelSize: 13
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        wrapMode: Text.WordWrap
    }
}
