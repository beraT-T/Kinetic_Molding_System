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
                text: "ADAPTİF KALIP"
                color: Theme.text; font.bold: true
                font.pixelSize: Theme.fsTitle
                font.letterSpacing: 1.5
            }
            Text {
                text: "Kontrol Sistemi v2.2" + (app.demoMode ? " · SİMÜLASYON" : "")
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
            enabled: !app.connected
            model: app.ports
            font.pixelSize: Theme.fsButton
            displayText: currentIndex >= 0 ? currentText : "Port seç"
        }
        TouchButton {
            text: "Portları yenile"
            color: Theme.actionNeutral
            visible: !app.demoMode
            enabled: !app.connected
            disabledReason: "önce bağlantıyı kesin"
            onClicked: app.refreshPorts()
        }
        TouchButton {
            text: app.connected ? "Bağlantıyı kes" : "Bağlan"
            color: app.connected ? Theme.actionNeutral : Theme.actionPrimary
            enabled: app.connected ? app.linkBlockReason === ""
                                   : (app.demoMode || portCombo.currentIndex >= 0)
            disabledReason: app.connected ? app.linkBlockReason : "port seçin"
            onClicked: {
                if (app.connected) app.disconnectPort()
                else app.connectPort(app.demoMode ? "SIM" : portCombo.currentText)
            }
        }
        TouchButton {
            text: "Modülleri bul"
            color: Theme.actionPrimary
            enabled: app.scanBlockReason === ""
            disabledReason: app.scanBlockReason
            onClicked: app.scan()
        }
    }
}
