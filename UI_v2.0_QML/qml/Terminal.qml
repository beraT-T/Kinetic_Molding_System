import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// Haberlesme gunlugu + ham komut girisi (Seviye-4 bilgi).
// v2.2: terminal de AYNI hareket yetkisinden gecer (backend sendRaw): PING/STAT/GETPOS/GETID
// serbest, ARR/ALL/MOV/HOME yetki kontrollu, SETID kilitli, taninmayan fiil reddedilir.
// Reddedilen komutun sebebi girisin altinda gosterilir.
Rectangle {
    id: root
    property var logModel
    property bool showInput: true
    property bool dense: false          // Bakim ekrani: kisa baslik, butonlar giris satirinda
    property string rejectText: ""      // son reddedilen komutun sebebi
    color: "#050a14"
    radius: Theme.radius
    border.color: Theme.border

    function submit() {
        if (!cmdField.text.length) return
        var r = app.sendRaw(cmdField.text)
        root.rejectText = r
        if (r === "") cmdField.text = ""
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.gapMin
            Rectangle { width: 8; height: 8; radius: 4; color: Theme.textFaint }
            Text { text: "HABERLEŞME GÜNLÜĞÜ"; color: Theme.textDim; font.bold: true; font.pixelSize: Theme.fsSmall }
            Item { Layout.fillWidth: true }
            TouchButton {
                visible: !root.dense
                text: "Günlüğü temizle"
                color: Theme.panelHi
                onClicked: root.logModel.clear()
            }
        }

        ListView {
            id: list
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            model: root.logModel
            spacing: 1
            delegate: Text {
                width: list.width
                text: line
                wrapMode: Text.WrapAnywhere
                font.family: "monospace"
                font.pixelSize: Theme.fsSmall
                color: line.indexOf("ALARM") >= 0 ? Theme.alarm
                     : (line.indexOf("HATA") >= 0 || line.indexOf("REDDEDİLDİ") >= 0
                        || line.indexOf("UYARI") >= 0) ? Theme.warn
                     : line.indexOf("→") >= 0 ? Theme.text
                     : Theme.textDim
            }
            onCountChanged: positionViewAtEnd()
            ScrollBar.vertical: ScrollBar { width: 16 }
        }

        RowLayout {
            visible: root.showInput
            Layout.fillWidth: true
            spacing: Theme.gapMin
            TextField {
                id: cmdField
                Layout.fillWidth: true
                Layout.preferredHeight: root.dense ? Theme.touchDense : Theme.inputHeight
                placeholderText: "Komut (ör. STAT:01, PING:01)"
                color: Theme.text
                font.family: "monospace"
                font.pixelSize: Theme.fsButton
                background: Rectangle { radius: Theme.radiusSm; color: "#0b1322"; border.color: Theme.border }
                onAccepted: root.submit()
            }
            TouchButton {
                text: "Gönder"
                dense: root.dense
                color: Theme.actionPrimary
                onClicked: root.submit()
            }
            TouchButton {
                visible: root.dense
                dense: true
                text: "Günlüğü temizle"
                color: Theme.panelHi
                onClicked: root.logModel.clear()
            }
        }
        Text {
            Layout.fillWidth: true
            visible: root.showInput && root.rejectText.length > 0
            text: "Gönderilmedi: " + root.rejectText
            color: Theme.warn
            font.pixelSize: Theme.fsSmall
            wrapMode: Text.WordWrap
            maximumLineCount: 2
            elide: Text.ElideRight
        }
    }
}
