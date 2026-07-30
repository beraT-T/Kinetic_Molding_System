import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

Button {
    id: btn
    property color color: Theme.accent
    // opsiyonel alt metin (or: "tum eksenler sifira")
    property string subText: ""
    Layout.fillWidth: true
    implicitHeight: Theme.btnHeight

    background: Rectangle {
        radius: Theme.radiusSm
        color: btn.enabled ? btn.color : Qt.darker(btn.color, 2.0)
        opacity: btn.enabled ? (btn.pressed ? 0.8 : 1.0) : 0.4
    }
    // NOT: contentItem implicitWidth'i tanimli olmali; aksi halde GridLayout
    // fillWidth sutunlarini yanlis dagitir (bir buton digerlerini sifira sikistirir).
    // Metin yatay+dikey ortalanir (Column butonun tam ortasinda).
    contentItem: Item {
        implicitWidth: col.implicitWidth
        implicitHeight: col.implicitHeight
        Column {
            id: col
            anchors.centerIn: parent
            spacing: 2
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: btn.text
                color: "white"
                font.bold: true
                font.pixelSize: Theme.fsButtonLg
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                visible: btn.subText.length > 0
                text: btn.subText
                color: "white"
                opacity: 0.8
                font.pixelSize: Theme.fsSmall
            }
        }
    }
}
