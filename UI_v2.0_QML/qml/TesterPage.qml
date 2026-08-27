import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

Item {
    id: page
    property var logModel
    signal applyVals(var arr)       // 9 hedef mm -> spinbox
    signal applyStates(var letters) // 9 durum harfi -> durum cipi

    function fill(v) { var a = []; for (var i = 0; i < 9; i++) a.push(v); return a }

    // STAT cevabi gelince, secili modul ise spinbox + durum cipini guncelle
    Connections {
        target: app
        function onStatusReceived(sid, toks) {
            if (sid !== app.currentSlaveId) return
            var nums = [], letters = []
            for (var i = 0; i < 9; i++) {
                var t = (i < toks.length) ? toks[i] : "I0"
                letters.push(t.substring(0, 1))
                nums.push(parseInt(t.substring(1)) || 0)
            }
            page.applyVals(nums)
            page.applyStates(letters)
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

        // ---------- SOL: 16 modul panosu ----------
        Rectangle {
            Layout.preferredWidth: 380
            Layout.minimumWidth: 380
            Layout.maximumWidth: 380
            Layout.fillHeight: true
            radius: Theme.radius
            color: Theme.panel
            border.color: Theme.border
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 14
                spacing: Theme.gapMin
                Text { text: "Moduller"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                GridLayout {
                    id: modGrid
                    Layout.fillWidth: true
                    columns: 4
                    rowSpacing: Theme.gapMin; columnSpacing: Theme.gapMin
                    // karolar KARE: hucre genisligi = (grid - 3*bosluk)/4
                    readonly property real cellSize:
                        Math.max(Theme.touchMin, (width - 3 * columnSpacing) / 4)
                    Repeater {
                        model: 16
                        delegate: Rectangle {
                            id: cell
                            required property int index
                            Layout.fillWidth: true
                            Layout.preferredHeight: modGrid.cellSize
                            radius: Theme.radiusSm
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
                                Text { text: "Modul " + cell.sid; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsBody
                                    anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: cell.online ? "bagli" : "bagli degil"
                                    color: cell.online ? Theme.green : Theme.textDim; font.pixelSize: Theme.fsSmall
                                    anchors.horizontalCenter: parent.horizontalCenter }
                            }
                            TapHandler { onTapped: app.currentSlaveId = cell.sid }
                        }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Text { text: "Canli Durum"; color: Theme.textDim; font.pixelSize: Theme.fsBody }
                    Item { Layout.fillWidth: true }
                    Switch { id: livesw }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    radius: Theme.radiusSm
                    color: Theme.panelHi
                    Column {
                        anchors.fill: parent; anchors.margins: 10; spacing: 4
                        Text { text: "Klavuz"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsBody }
                        Text { text: "• Soldan modul sec\n• Motor kutusuna deger yaz + Enter\n• 'Canli Durum' surekli izler\n• Terminalden ham komut girilebilir"
                            color: Theme.textDim; font.pixelSize: Theme.fsSmall; lineHeight: 1.3 }
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
                Layout.fillHeight: true
                Layout.minimumHeight: 430
                radius: Theme.radius
                color: Theme.panel
                border.color: Theme.border
                clip: true
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: Theme.gapMin
                    // referans uyarisi (Tester'da engelleme yok, yalniz uyari)
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 36
                        radius: Theme.radiusSm
                        color: Theme.red
                        visible: app.connected && !app.allHomed
                        Text {
                            anchors.centerIn: parent
                            text: "Referans alinmadi - pozisyonlar guvenilir degil. Once Home yapin."
                            color: "white"; font.bold: true; font.pixelSize: Theme.fsSmall
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: Theme.gapMin
                        Text { text: "Modul " + app.currentSlaveId; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                        Item { Layout.fillWidth: true }
                        TouchButton {
                            text: "Tumunu Sifirla"; color: Theme.orange
                            enabled: app.connected
                            onClicked: { app.allToValue(app.currentSlaveId, 0); page.applyVals(page.fill(0)) }
                        }
                        TouchButton {
                            text: "Test 300 mm"; color: Theme.accent
                            enabled: app.connected
                            onClicked: { app.allToValue(app.currentSlaveId, 300); page.applyVals(page.fill(300)) }
                        }
                        TouchButton {
                            text: "Home Hepsi"; color: Theme.green
                            enabled: app.connected
                            onClicked: { app.homeSlave(app.currentSlaveId); page.applyVals(page.fill(0)) }
                        }
                    }

                    GridLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        columns: 3
                        rowSpacing: Theme.gapMin; columnSpacing: Theme.gapMin
                        Repeater {
                            model: 9
                            delegate: Rectangle {
                                id: mcell
                                required property int index
                                property string st: "I"
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                radius: Theme.radiusSm
                                color: Theme.panelHi
                                border.color: mcell.st === "F" ? Theme.red : Theme.border
                                border.width: mcell.st === "F" ? 3 : 1

                                Connections {
                                    target: page
                                    function onApplyStates(letters) { mcell.st = letters[index] || "I" }
                                }

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 10
                                    spacing: 6

                                    // baslik + durum cipi
                                    RowLayout {
                                        Layout.fillWidth: true
                                        Text { text: "Motor " + (index + 1); color: Theme.text; font.pixelSize: Theme.fsBody; font.bold: true }
                                        Text { text: "mm · yaz + Enter"; color: Theme.textDim; font.pixelSize: Theme.fsSmall }
                                        Item { Layout.fillWidth: true }
                                        Rectangle {
                                            radius: Theme.radiusSm
                                            implicitHeight: 26
                                            implicitWidth: stLbl.implicitWidth + 18
                                            color: Theme.stateColor(mcell.st)
                                            Text { id: stLbl; anchors.centerIn: parent
                                                text: Theme.stateLabel(mcell.st)
                                                color: "white"; font.bold: true; font.pixelSize: Theme.fsSmall }
                                        }
                                    }

                                    Item { Layout.fillHeight: true; Layout.fillWidth: true }

                                    // Hedef mm girisi: degeri yaz + Enter ile gonder (MOV).
                                    // +/- stepper sadece degeri ayarlar, gondermez (kazara hareket yok).
                                    SpinBox {
                                        id: sp
                                        Layout.fillWidth: true
                                        Layout.preferredHeight: Theme.inputHeight
                                        from: 0; to: 600; stepSize: 10; editable: true
                                        value: 0
                                        font.pixelSize: Theme.fsValue
                                        contentItem: TextInput {
                                            text: sp.displayText
                                            color: Theme.accent2
                                            font.pixelSize: Theme.fsValue
                                            font.bold: true
                                            horizontalAlignment: Qt.AlignHCenter
                                            verticalAlignment: Qt.AlignVCenter
                                            readOnly: !sp.editable
                                            validator: sp.validator
                                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                                            selectByMouse: true
                                            onAccepted: {
                                                sp.value = sp.valueFromText(text, sp.locale)
                                                if (app.connected)
                                                    app.moveMotor(app.currentSlaveId, index + 1, sp.value)
                                            }
                                        }
                                        Connections {
                                            target: page
                                            function onApplyVals(arr) { sp.value = arr[index] }
                                        }
                                    }
                                    Item { Layout.fillHeight: true; Layout.fillWidth: true }

                                    Button {
                                        text: "Home"
                                        Layout.fillWidth: true
                                        implicitHeight: Theme.touchMin
                                        enabled: app.connected
                                        onClicked: { app.homeMotor(app.currentSlaveId, index + 1); sp.value = 0 }
                                        background: Rectangle { radius: Theme.radiusSm; color: Theme.orange; opacity: parent.enabled ? 0.9 : 0.3 }
                                        contentItem: Text { text: parent.text; color: "white"; font.pixelSize: Theme.fsButton; font.bold: true
                                            horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            Terminal {
                Layout.fillWidth: true
                Layout.preferredHeight: 140
                logModel: page.logModel
            }
        }
    }
}
