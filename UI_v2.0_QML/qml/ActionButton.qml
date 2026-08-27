import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// Grid icinde genisleyen birincil aksiyon butonu.
// Devre disi stili: opacity ile solmak yerine NOTR dolgu + okunur metin
// (endustriyel HMI'de "buton neden basmiyor?" en yaygin operator sikayeti;
// sebep subText ile yazilir).
Button {
    id: btn
    property color color: Theme.actionPrimary
    property string subText: ""
    property string disabledReason: ""
    Layout.fillWidth: true
    implicitHeight: Theme.btnHeight

    background: Rectangle {
        radius: Theme.radiusSm
        color: btn.enabled ? (btn.pressed ? Qt.darker(btn.color, 1.3) : btn.color)
                           : Theme.panelHi
        border.color: btn.enabled ? "transparent" : Theme.border
        border.width: 1
    }
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
                color: btn.enabled ? "#ffffff" : Theme.textDim
                font.bold: true
                font.pixelSize: Theme.fsButtonLg
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                visible: text.length > 0
                text: (!btn.enabled && btn.disabledReason.length) ? btn.disabledReason : btn.subText
                color: btn.enabled ? "#ffffff" : Theme.warn
                opacity: btn.enabled ? 0.85 : 1.0
                font.pixelSize: Theme.fsSmall
            }
        }
    }
}
