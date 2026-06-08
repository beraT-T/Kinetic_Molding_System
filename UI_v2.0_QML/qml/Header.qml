import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

Rectangle {
    height: 64
    color: Theme.panel
    border.color: Theme.border
    border.width: 1

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 16
        anchors.rightMargin: 16
        spacing: 14

        // baslik
        Column {
            spacing: 0
            Text { text: "ADAPTIF KALIP"; color: Theme.text; font.bold: true; font.pixelSize: 18 }
            Text { text: "Kontrol Sistemi v2.0"; color: Theme.textDim; font.pixelSize: 11 }
        }

        Item { Layout.fillWidth: true }

        // demo modu
        RowLayout {
            spacing: 6
            Text { text: "Demo"; color: Theme.textDim; font.pixelSize: 12 }
            Switch {
                checked: app.demoMode
                onToggled: app.setDemoMode(checked)
            }
        }

        // port secimi
        ComboBox {
            id: portCombo
            Layout.preferredWidth: 200
            visible: !app.demoMode
            model: app.ports
            displayText: currentIndex >= 0 ? currentText : "Port sec"
        }
        Button {
            text: "⟳"
            visible: !app.demoMode
            onClicked: app.refreshPorts()
            ToolTip.text: "Portlari yenile"; ToolTip.visible: hovered
        }

        // baglan / kes
        Button {
            text: app.connected ? "Baglantiyi Kes" : "Baglan"
            onClicked: {
                if (app.connected) app.disconnectPort()
                else app.connectPort(app.demoMode ? "DEMO" : portCombo.currentText)
            }
            background: Rectangle {
                radius: 8
                color: app.connected ? Theme.red : Theme.green
            }
            contentItem: Text {
                text: parent.text; color: "white"; font.bold: true; font.pixelSize: 13
                horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                leftPadding: 14; rightPadding: 14; topPadding: 8; bottomPadding: 8
            }
        }

        // tara
        Button {
            text: "Agi Tara"
            enabled: app.connected
            onClicked: app.scan()
            background: Rectangle { radius: 8; color: Theme.accent; opacity: parent.enabled ? 1 : 0.4 }
            contentItem: Text {
                text: parent.text; color: "white"; font.bold: true; font.pixelSize: 13
                horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                leftPadding: 14; rightPadding: 14; topPadding: 8; bottomPadding: 8
            }
        }

        // durum
        Row {
            spacing: 8
            Rectangle {
                width: 12; height: 12; radius: 6
                anchors.verticalCenter: parent.verticalCenter
                color: app.connected ? Theme.green : Theme.red
            }
            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: app.connected ? ("Bagli · " + app.activeSlaves.length + " slave") : "Bagli degil"
                color: Theme.textDim; font.pixelSize: 12
            }
        }
    }
}
