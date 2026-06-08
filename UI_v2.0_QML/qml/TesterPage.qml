import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

Item {
    id: page
    property var logModel
    signal applyVals(var arr)     // 9 deger -> sliderlara uygula

    function fill(v) { var a = []; for (var i = 0; i < 9; i++) a.push(v); return a }

    // STAT cevabi gelince, secili slave ise sliderlari guncelle
    Connections {
        target: app
        function onStatusReceived(sid, toks) {
            if (sid !== app.currentSlaveId) return
            var arr = []
            for (var i = 0; i < 9; i++) {
                var t = (i < toks.length) ? toks[i] : "I0"
                arr.push(parseInt(t.substring(1)) || 0)
            }
            page.applyVals(arr)
        }
    }

    // canli durum poll
    Timer {
        id: poll
        interval: 1200; repeat: true; running: livesw.checked && app.connected
        onTriggered: app.requestStatus(app.currentSlaveId)
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 16

        // ---------- SOL: 16 slave panosu ----------
        Rectangle {
            Layout.preferredWidth: 360
            Layout.fillHeight: true
            radius: 12
            color: Theme.panel
            border.color: Theme.border
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 14
                spacing: 12
                Text { text: "Moduller (16 Slave)"; color: Theme.text; font.bold: true; font.pixelSize: 15 }
                GridLayout {
                    Layout.fillWidth: true
                    columns: 4
                    rowSpacing: 8; columnSpacing: 8
                    Repeater {
                        model: 16
                        delegate: Rectangle {
                            id: cell
                            required property int index
                            Layout.fillWidth: true
                            height: 60
                            radius: 8
                            property int sid: index + 1
                            property bool sel: app.currentSlaveId === sid
                            property bool online: app.activeSlaves.indexOf(sid) >= 0
                            color: cell.sel ? Qt.rgba(0.23,0.51,0.96,0.35)
                                 : cell.online ? Qt.rgba(0.13,0.77,0.37,0.18) : Theme.panelHi
                            border.width: cell.sel ? 2 : 1
                            border.color: cell.sel ? Theme.accent : (cell.online ? Theme.green : Theme.border)
                            Column {
                                anchors.centerIn: parent
                                spacing: 2
                                Text { text: "ID " + cell.sid; color: Theme.text; font.bold: true; font.pixelSize: 13
                                    anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: cell.online ? "online" : "offline"
                                    color: cell.online ? Theme.green : Theme.textDim; font.pixelSize: 10
                                    anchors.horizontalCenter: parent.horizontalCenter }
                            }
                            MouseArea { anchors.fill: parent; onClicked: app.currentSlaveId = cell.sid }
                        }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Text { text: "Canli durum"; color: Theme.textDim; font.pixelSize: 12 }
                    Item { Layout.fillWidth: true }
                    Switch { id: livesw }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    radius: 8
                    color: Theme.panelHi
                    Column {
                        anchors.fill: parent; anchors.margins: 10; spacing: 4
                        Text { text: "Klavuz"; color: Theme.text; font.bold: true; font.pixelSize: 12 }
                        Text { text: "• Soldan modul sec\n• 3x3 grid'den motorlari sur\n• 'Canli durum' STAT poll eder\n• Terminalden ham komut girilebilir"
                            color: Theme.textDim; font.pixelSize: 11; lineHeight: 1.3 }
                    }
                }
            }
        }

        // ---------- SAG: motor kontrol + terminal ----------
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 14

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 430
                radius: 12
                color: Theme.panel
                border.color: Theme.border
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 12
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "Modul Kontrol: ID " + app.currentSlaveId; color: Theme.text; font.bold: true; font.pixelSize: 16 }
                        Item { Layout.fillWidth: true }
                        Button {
                            text: "Tumunu Sifirla"
                            enabled: app.connected
                            onClicked: { app.allToValue(app.currentSlaveId, 0); page.applyVals(page.fill(0)) }
                            background: Rectangle { radius: 6; color: Theme.orange; opacity: parent.enabled?1:0.4 }
                            contentItem: Text { text: parent.text; color: "white"; font.bold: true; font.pixelSize: 12
                                leftPadding:12; rightPadding:12; topPadding:7; bottomPadding:7 }
                        }
                        Button {
                            text: "Test 300mm"
                            enabled: app.connected
                            onClicked: { app.allToValue(app.currentSlaveId, 300); page.applyVals(page.fill(300)) }
                            background: Rectangle { radius: 6; color: Theme.accent; opacity: parent.enabled?1:0.4 }
                            contentItem: Text { text: parent.text; color: "white"; font.bold: true; font.pixelSize: 12
                                leftPadding:12; rightPadding:12; topPadding:7; bottomPadding:7 }
                        }
                        Button {
                            text: "Home Hepsi"
                            enabled: app.connected
                            onClicked: { app.homeSlave(app.currentSlaveId); page.applyVals(page.fill(0)) }
                            background: Rectangle { radius: 6; color: Theme.green; opacity: parent.enabled?1:0.4 }
                            contentItem: Text { text: parent.text; color: "white"; font.bold: true; font.pixelSize: 12
                                leftPadding:12; rightPadding:12; topPadding:7; bottomPadding:7 }
                        }
                    }

                    GridLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        columns: 3
                        rowSpacing: 12; columnSpacing: 12
                        Repeater {
                            model: 9
                            delegate: Rectangle {
                                required property int index
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                radius: 10
                                color: Theme.panelHi
                                border.color: Theme.border
                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 8
                                    spacing: 4
                                    Text { text: "Motor " + (index + 1); color: Theme.textDim; font.pixelSize: 12; font.bold: true
                                        Layout.alignment: Qt.AlignHCenter }
                                    Slider {
                                        id: s
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true
                                        orientation: Qt.Vertical
                                        from: 0; to: 600; value: 0
                                        Layout.alignment: Qt.AlignHCenter
                                        onPressedChanged: if (!pressed && app.connected)
                                            app.moveMotor(app.currentSlaveId, index + 1, Math.round(value))
                                        Connections {
                                            target: page
                                            function onApplyVals(arr) { s.value = arr[index] }
                                        }
                                    }
                                    Text { text: Math.round(s.value) + " mm"; color: Theme.accent2; font.bold: true; font.pixelSize: 13
                                        Layout.alignment: Qt.AlignHCenter }
                                    Button {
                                        text: "Home"
                                        Layout.fillWidth: true
                                        enabled: app.connected
                                        onClicked: { app.homeMotor(app.currentSlaveId, index + 1); s.value = 0 }
                                        background: Rectangle { radius: 5; color: Theme.orange; opacity: parent.enabled?0.85:0.3 }
                                        contentItem: Text { text: parent.text; color: "white"; font.pixelSize: 10; font.bold: true
                                            horizontalAlignment: Text.AlignHCenter; topPadding:4; bottomPadding:4 }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            Terminal {
                Layout.fillWidth: true
                Layout.fillHeight: true
                logModel: page.logModel
            }
        }
    }
}
