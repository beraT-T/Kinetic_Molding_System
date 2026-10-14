import QtQuick
import QtQuick.Controls.Basic
import "Theme.js" as Theme

// Satir icinde kullanilan, icerige gore genisleyen dokunma butonu.
// v2.2: devre disiyken SEBEP ikinci satirda yazilir (endustriyel HMI'de "buton
// neden basmiyor?" en yaygin operator sikayeti). Uzun sebep buton genisligine
// kisaltilir; tam metin ilgili grubun sebep satirinda / durum seridinde gorunur.
Button {
    id: b
    property color color: Theme.actionPrimary
    property bool dense: false          // bakim ekranlari icin yogun mod
    property string disabledReason: ""
    readonly property bool showReason: !b.enabled && b.disabledReason.length > 0
    implicitHeight: dense ? Theme.touchDense : Theme.touchMin
    implicitWidth: Math.max(lbl.implicitWidth + 40, implicitHeight)

    background: Rectangle {
        radius: Theme.radiusSm
        color: b.enabled ? (b.pressed ? Qt.darker(b.color, 1.3) : b.color)
                         : Theme.panelHi
        border.color: b.enabled ? "transparent" : Theme.border
        border.width: 1
    }
    contentItem: Item {
        Column {
            anchors.centerIn: parent
            width: parent.width
            spacing: 0
            Text {
                id: lbl
                width: parent.width
                text: b.text
                color: b.enabled ? "#ffffff" : Theme.textDim
                font.bold: true
                font.pixelSize: b.dense ? Theme.fsBody : Theme.fsButton
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
            }
            Text {
                width: parent.width
                visible: b.showReason
                text: b.disabledReason
                color: Theme.warn
                font.pixelSize: Theme.fsSmall
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
            }
        }
    }
}
