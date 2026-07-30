import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

Rectangle {
    height: Theme.btnHeight + 24
    color: Theme.panel
    border.color: Theme.border
    border.width: 1

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 16
        anchors.rightMargin: 16
        spacing: Theme.gapMin

        // baslik
        Column {
            spacing: 0
            Text { text: "ADAPTIF KALIP"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
            Text { text: "Kontrol Sistemi v2.0"; color: Theme.textDim; font.pixelSize: Theme.fsSmall }
        }

        Item { Layout.fillWidth: true }

        // port secimi (gercek donanim; demo bayrakli calisirken gizli)
        ComboBox {
            id: portCombo
            Layout.preferredWidth: 240
            Layout.preferredHeight: Theme.inputHeight
            visible: !app.demoMode
            model: app.ports
            font.pixelSize: Theme.fsButton
            displayText: currentIndex >= 0 ? currentText : "Port sec"
        }
        TouchButton {
            text: "Yenile"
            color: Theme.panelHi
            visible: !app.demoMode
            onClicked: app.refreshPorts()
        }

        // baglan / kes (tek toggle - panelde yer kazandirir)
        TouchButton {
            text: app.connected ? "Baglantiyi Kes" : "Baglan"
            color: app.connected ? Theme.red : Theme.green
            onClicked: {
                if (app.connected) app.disconnectPort()
                else app.connectPort(app.demoMode ? "DEMO" : portCombo.currentText)
            }
        }

        // modulleri bul (tara)
        TouchButton {
            text: "Modulleri Bul"
            color: Theme.accent
            enabled: app.connected
            onClicked: app.scan()
        }

        // durum
        Row {
            spacing: 8
            Rectangle {
                width: 14; height: 14; radius: 7
                anchors.verticalCenter: parent.verticalCenter
                color: app.connected ? Theme.green : Theme.red
            }
            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: app.connected ? ("Bagli · " + app.activeSlaves.length + " modul") : "Bagli degil"
                color: Theme.textDim; font.pixelSize: Theme.fsBody
            }
        }
    }
}
