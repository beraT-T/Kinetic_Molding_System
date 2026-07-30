import QtQuick
import QtQuick.Controls.Basic
import "Theme.js" as Theme

// Icerige gore genisleyen, sabit dokunma yukseklikli buton (header / satir ici).
// ActionButton fillWidth'tir (grid hucreleri icin); bu ise satir icinde kullanilir.
Button {
    id: b
    property color color: Theme.accent
    implicitHeight: Theme.btnHeight

    background: Rectangle {
        radius: Theme.radiusSm
        color: b.enabled ? b.color : Qt.darker(b.color, 2.0)
        opacity: b.enabled ? (b.pressed ? 0.8 : 1.0) : 0.4
    }
    contentItem: Text {
        text: b.text
        color: "white"
        font.bold: true
        font.pixelSize: Theme.fsButton
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        leftPadding: 18; rightPadding: 18
    }
}
