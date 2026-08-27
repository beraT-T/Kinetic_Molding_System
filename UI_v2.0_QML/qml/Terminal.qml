import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

Rectangle {
    id: root
    property var logModel
    property bool showInput: true
    color: "#050a14"
    radius: Theme.radius
    border.color: Theme.border

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.gapMin
            Rectangle { width: 8; height: 8; radius: 4; color: Theme.textFaint }
            Text { text: "SERIAL MONITOR"; color: Theme.textDim; font.bold: true; font.pixelSize: Theme.fsSmall }
            Item { Layout.fillWidth: true }
            TouchButton {
                text: "Temizle"
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
                color: line.indexOf("→") >= 0 ? Theme.text
                     : line.indexOf("←") >= 0 ? Theme.textDim
                     : line.indexOf("HATA") >= 0 ? Theme.alarm
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
                Layout.preferredHeight: Theme.inputHeight
                placeholderText: "Komut (or: PING:01, ARR:01:..., STAT:01)"
                color: Theme.text
                font.family: "monospace"
                font.pixelSize: Theme.fsButton
                background: Rectangle { radius: Theme.radiusSm; color: "#0b1322"; border.color: Theme.border }
                onAccepted: { if (text.length) { app.sendRaw(text); text = "" } }
            }
            TouchButton {
                text: "Gonder"
                color: Theme.actionPrimary
                onClicked: { if (cmdField.text.length) { app.sendRaw(cmdField.text); cmdField.text = "" } }
            }
        }
    }
}
