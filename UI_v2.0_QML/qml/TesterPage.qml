import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

Item {
    id: page
    property var logModel
    // Secili modulun SON OLCULEN durumu (backend modul deposu; kimlik + yas).
    // moduleRevision her STAT/tick'te artar -> binding yeniden hesaplanir.
    property var info: { app.moduleRevision; return app.moduleInfo(app.currentSlaveId) }

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
                            // NORMAL durum notr kalir (yesil dekoratif kullanilmaz);
                            // secim mavi cerceve, bagli olmayan sonuk.
                            color: cell.sel ? Qt.rgba(0.29,0.44,0.65,0.35) : Theme.panelHi
                            opacity: cell.online || cell.sel ? 1.0 : 0.55
                            border.width: cell.sel ? 3 : 1
                            border.color: cell.sel ? Theme.selection : Theme.border
                            Column {
                                anchors.centerIn: parent
                                spacing: 2
                                Text { text: cell.sid; color: Theme.text; font.bold: true
                                    font.pixelSize: Theme.fsTitle; font.family: Theme.fontMono
                                    anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: cell.online ? "bagli" : "yok"
                                    color: Theme.textDim; font.pixelSize: Theme.fsSmall
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
                        // ANORMAL / on-kosul eksik -> AMBER (kirmizi yalniz gercek ariza)
                        color: Theme.warn
                        visible: app.connected && !app.allHomed
                        Text {
                            anchors.centerIn: parent
                            text: "⚠  REFERANS ALINMADI - pozisyonlar guvenilir degil, once Home yapin"
                            color: "#101214"; font.bold: true; font.pixelSize: Theme.fsBody
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: Theme.gapMin
                        Text { text: "Modul " + app.currentSlaveId; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                        Item { Layout.fillWidth: true }
                        TouchButton {
                            text: "Test 300 mm"; color: Theme.actionNeutral
                            enabled: app.testerMoveBlockReason === ""
                            onClicked: app.allToValue(app.currentSlaveId, 300)
                        }
                        TouchButton {
                            text: "Home Hepsi"; color: Theme.actionWarn
                            enabled: app.testerHomeBlockReason === ""
                            onClicked: app.homeSlave(app.currentSlaveId)
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
                                readonly property var mi: page.info.motors[index]
                                readonly property string st: mi.known ? mi.letter : ""
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                radius: Theme.radiusSm
                                color: Theme.panelHi
                                border.color: mcell.st === "F" ? Theme.alarm : Theme.border
                                border.width: mcell.st === "F" ? 3 : 1

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 10
                                    spacing: 6

                                    // baslik + durum cipi (renk + METIN birlikte - WCAG 1.4.1)
                                    RowLayout {
                                        Layout.fillWidth: true
                                        Text { text: "MOTOR " + (index + 1); color: Theme.text
                                            font.pixelSize: Theme.fsBody; font.bold: true }
                                        Item { Layout.fillWidth: true }
                                        Rectangle {
                                            radius: Theme.radiusSm
                                            implicitHeight: 30
                                            implicitWidth: stLbl.implicitWidth + 20
                                            color: mcell.st === "F" ? Theme.alarm
                                                 : mcell.st === "H" ? Theme.warn : Theme.panel
                                            border.color: Theme.border
                                            border.width: (mcell.st === "F" || mcell.st === "H") ? 0 : 1
                                            Text { id: stLbl; anchors.centerIn: parent
                                                text: (mcell.st === "F" ? "⚠ " : "") + Theme.stateLabel(mcell.st)
                                                color: (mcell.st === "F" || mcell.st === "H") ? "#101214" : Theme.textDim
                                                font.bold: true; font.pixelSize: Theme.fsSmall }
                                        }
                                    }

                                    Item { Layout.fillHeight: true; Layout.fillWidth: true }

                                    // Hedef mm girisi + Home YAN YANA (dikey alan kazanci).
                                    // Deger yaz + Enter -> MOV. +/- stepper sadece ayarlar, gondermez.
                                    RowLayout {
                                        Layout.fillWidth: true
                                        spacing: Theme.gapMin
                                        SpinBox {
                                            id: sp
                                            Layout.fillWidth: true
                                            Layout.preferredHeight: Theme.touchDense
                                            from: 0; to: 600; stepSize: 10; editable: true
                                            value: 0
                                            font.pixelSize: Theme.fsButton
                                            contentItem: TextInput {
                                                text: sp.displayText
                                                color: Theme.text
                                                font.pixelSize: Theme.fsValue
                                                font.bold: true
                                                font.family: Theme.fontMono
                                                horizontalAlignment: Qt.AlignHCenter
                                                verticalAlignment: Qt.AlignVCenter
                                                readOnly: !sp.editable
                                                validator: sp.validator
                                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                                selectByMouse: true
                                                onAccepted: {
                                                    sp.value = sp.valueFromText(text, sp.locale)
                                                    app.moveMotor(app.currentSlaveId, index + 1, sp.value)
                                                }
                                            }
                                        }
                                        TouchButton {
                                            text: "Home"
                                            dense: true
                                            color: Theme.actionWarn
                                            enabled: app.testerHomeBlockReason === ""
                                            onClicked: app.homeMotor(app.currentSlaveId, index + 1)
                                        }
                                    }

                                    Text { text: "mm  ·  deger yaz + Enter"
                                        color: Theme.textFaint; font.pixelSize: Theme.fsSmall
                                        Layout.alignment: Qt.AlignHCenter }

                                    Item { Layout.fillHeight: true; Layout.fillWidth: true }
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
