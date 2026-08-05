import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// Dokunmatik olculu modal uyari/onay penceresi.
// show(baslik, mesaj, kabulMetni, redMetni, renk) ile acilir.
// redMetni bos ise tek butonlu bilgi penceresi olur.
Item {
    id: root
    anchors.fill: parent
    visible: false
    z: 2000

    property string title: ""
    property string message: ""
    property string acceptText: "Tamam"
    property string rejectText: ""
    property color accentColor: Theme.accent

    signal accepted()
    signal rejected()

    function show(t, m, okText, cancelText, col) {
        root.title = t || ""
        root.message = m || ""
        root.acceptText = okText || "Tamam"
        root.rejectText = cancelText || ""
        root.accentColor = col || Theme.accent
        root.visible = true
    }

    // NOT: QML 8-hane hex = #AARRGGBB (alpha once)
    Rectangle {
        anchors.fill: parent
        color: "#cc000000"
        MouseArea { anchors.fill: parent }   // arkadaki dokunuslari yut
    }

    Rectangle {
        anchors.centerIn: parent
        width: Math.min(parent.width * 0.6, 720)
        height: content.implicitHeight + 48
        radius: Theme.radius
        color: Theme.panel
        border.color: root.accentColor
        border.width: 2

        ColumnLayout {
            id: content
            anchors.centerIn: parent
            width: parent.width - 48
            spacing: 18

            // baslik seridi
            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gapMin
                Rectangle {
                    width: 14; height: 14; radius: 7
                    color: root.accentColor
                    Layout.alignment: Qt.AlignVCenter
                }
                Text {
                    Layout.fillWidth: true
                    text: root.title
                    color: Theme.text
                    font.bold: true
                    font.pixelSize: Theme.fsTitle
                    wrapMode: Text.WordWrap
                }
            }

            Text {
                Layout.fillWidth: true
                text: root.message
                color: Theme.textDim
                font.pixelSize: Theme.fsButton
                wrapMode: Text.WordWrap
                lineHeight: 1.25
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gapMin
                Item { Layout.fillWidth: true }
                TouchButton {
                    text: root.rejectText
                    color: Theme.panelHi
                    visible: root.rejectText.length > 0
                    onClicked: { root.visible = false; root.rejected() }
                }
                TouchButton {
                    text: root.acceptText
                    color: root.accentColor
                    onClicked: { root.visible = false; root.accepted() }
                }
            }
        }
    }
}
