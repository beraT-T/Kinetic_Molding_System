import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

Rectangle {
    id: root
    property var logModel
    property bool showInput: true
    color: "#050a14"
    radius: 10
    border.color: Theme.border

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            Rectangle { width: 8; height: 8; radius: 4; color: Theme.green }
            Text { text: "SERIAL MONITOR"; color: Theme.textDim; font.bold: true; font.pixelSize: 11 }
            Item { Layout.fillWidth: true }
            Button {
                text: "Temizle"
                onClicked: root.logModel.clear()
                background: Rectangle { radius: 6; color: Theme.panelHi }
                contentItem: Text { text: parent.text; color: Theme.textDim; font.pixelSize: 11
                    leftPadding:8; rightPadding:8; topPadding:3; bottomPadding:3 }
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
                font.pixelSize: 12
                color: line.indexOf("→") >= 0 ? Theme.accent2
                     : line.indexOf("←") >= 0 ? Theme.green
                     : line.indexOf("HATA") >= 0 ? Theme.red
                     : Theme.textDim
            }
            onCountChanged: positionViewAtEnd()
            ScrollBar.vertical: ScrollBar {}
        }

        RowLayout {
            visible: root.showInput
            Layout.fillWidth: true
            spacing: 6
            TextField {
                id: cmdField
                Layout.fillWidth: true
                placeholderText: "Komut (or: PING:01, ARR:01:..., STAT:01)"
                color: Theme.text
                font.family: "monospace"
                background: Rectangle { radius: 6; color: "#0b1322"; border.color: Theme.border }
                onAccepted: { if (text.length) { app.sendRaw(text); text = "" } }
            }
            Button {
                text: "Gonder"
                onClicked: { if (cmdField.text.length) { app.sendRaw(cmdField.text); cmdField.text = "" } }
                background: Rectangle { radius: 6; color: Theme.accent }
                contentItem: Text { text: parent.text; color: "white"; font.bold: true; font.pixelSize: 12
                    leftPadding:14; rightPadding:14; topPadding:8; bottomPadding:8 }
            }
        }
    }
}
