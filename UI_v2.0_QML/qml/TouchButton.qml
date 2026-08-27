import QtQuick
import QtQuick.Controls.Basic
import "Theme.js" as Theme

// Satir icinde kullanilan, icerige gore genisleyen dokunma butonu.
Button {
    id: b
    property color color: Theme.actionPrimary
    property bool dense: false          // bakim ekranlari icin yogun mod
    implicitHeight: dense ? Theme.touchDense : Theme.touchMin

    background: Rectangle {
        radius: Theme.radiusSm
        color: b.enabled ? (b.pressed ? Qt.darker(b.color, 1.3) : b.color)
                         : Theme.panelHi
        border.color: b.enabled ? "transparent" : Theme.border
        border.width: 1
    }
    contentItem: Text {
        text: b.text
        color: b.enabled ? "#ffffff" : Theme.textDim
        font.bold: true
        font.pixelSize: b.dense ? Theme.fsBody : Theme.fsButton
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        leftPadding: 20; rightPadding: 20
    }
}
