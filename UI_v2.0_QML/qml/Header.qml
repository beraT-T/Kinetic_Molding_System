import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// Baslik cubugu. Durum/alarm bilgisi burada DEGIL, altindaki StatusStrip'te
// (ISA-101: anormal durum tek ve sabit bir yerde gosterilir; tekrar etmez).
Rectangle {
    height: Theme.btnHeight + 22
    color: Theme.panel
    border.color: Theme.border
    border.width: 1

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 16
        anchors.rightMargin: 16
        spacing: Theme.gapMin

        Column {
            spacing: 0
            Text {
                text: "ADAPTIF KALIP"
                color: Theme.text; font.bold: true
                font.pixelSize: Theme.fsTitle
                font.letterSpacing: 1.5
            }
            Text {
                text: "Kontrol Sistemi v2.0"
                color: Theme.textDim; font.pixelSize: Theme.fsSmall
            }
        }

        Item { Layout.fillWidth: true }

        // --- baglanti kurulumu (gunde bir kez kullanilir) ---
        ComboBox {
            id: portCombo
            Layout.preferredWidth: 260
            Layout.preferredHeight: Theme.inputHeight
            visible: !app.demoMode
            enabled: !app.busy
            model: app.ports
            font.pixelSize: Theme.fsButton
            displayText: currentIndex >= 0 ? currentText : "Port sec"
        }
        TouchButton {
            text: "Yenile"
            color: Theme.actionNeutral
            visible: !app.demoMode
            enabled: !app.busy
            onClicked: app.refreshPorts()
        }
        TouchButton {
            text: app.connected ? "Baglantiyi Kes" : "Baglan"
            color: app.connected ? Theme.actionNeutral : Theme.actionPrimary
            enabled: !app.busy
            onClicked: {
                if (app.connected) app.disconnectPort()
                else app.connectPort(app.demoMode ? "DEMO" : portCombo.currentText)
            }
        }
        TouchButton {
            text: "Modulleri Bul"
            color: Theme.actionPrimary
            enabled: app.connected && !app.busy
            onClicked: app.scan()
        }
    }
}
