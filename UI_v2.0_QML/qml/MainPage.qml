import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtQuick.Dialogs
import "Theme.js" as Theme

Item {
    id: page
    property var logModel
    property string stlName: ""

    FileDialog {
        id: fileDialog
        title: "STL dosyasi sec"
        nameFilters: ["STL dosyalari (*.stl)"]
        onAccepted: {
            page.stlName = selectedFile.toString().split("/").pop()
            app.loadStl(selectedFile)
        }
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 16

        // ---------- SOL: STL + 3D + terminal ----------
        ColumnLayout {
            Layout.preferredWidth: 520
            Layout.fillHeight: true
            spacing: 14

            // dosya
            Rectangle {
                Layout.fillWidth: true
                height: 92
                radius: 12
                color: Theme.panel
                border.color: Theme.border
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 12
                    Column {
                        Layout.fillWidth: true
                        spacing: 4
                        Text { text: "STL Modeli"; color: Theme.text; font.bold: true; font.pixelSize: 14 }
                        Text {
                            text: page.stlName.length ? page.stlName : "Dosya secilmedi"
                            color: page.stlName.length ? Theme.accent2 : Theme.textDim
                            font.pixelSize: 12; elide: Text.ElideMiddle; width: 300
                        }
                    }
                    Button {
                        text: "STL Sec + Hesapla"
                        onClicked: fileDialog.open()
                        background: Rectangle { radius: 8; color: Theme.accent }
                        contentItem: Text { text: parent.text; color: "white"; font.bold: true; font.pixelSize: 13
                            leftPadding:14; rightPadding:14; topPadding:10; bottomPadding:10 }
                    }
                }
            }

            // 3D
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 320
                radius: 12
                color: Theme.panel
                border.color: Theme.border
                clip: true
                Column {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 6
                    Text { text: "3D Onizleme (mold yuzeyi)"; color: Theme.textDim; font.bold: true; font.pixelSize: 12 }
                    Surface3D {
                        width: parent.width
                        height: parent.height - 24
                    }
                }
            }

            // terminal
            Terminal {
                Layout.fillWidth: true
                Layout.fillHeight: true
                logModel: page.logModel
            }
        }

        // ---------- SAG: heatmap + aksiyonlar ----------
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 14

            // istatistik seridi
            Rectangle {
                Layout.fillWidth: true
                height: 64
                radius: 12
                color: Theme.panel
                border.color: Theme.border
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 24
                    Repeater {
                        model: [
                            { k: "Min",  v: app.stats && app.stats.min  !== undefined ? app.stats.min  + " mm" : "—" },
                            { k: "Max",  v: app.stats && app.stats.max  !== undefined ? app.stats.max  + " mm" : "—" },
                            { k: "Ort",  v: app.stats && app.stats.mean !== undefined ? app.stats.mean + " mm" : "—" },
                            { k: "Std",  v: app.stats && app.stats.std  !== undefined ? app.stats.std  + " mm" : "—" }
                        ]
                        delegate: Column {
                            Text { text: modelData.k; color: Theme.textDim; font.pixelSize: 11 }
                            Text { text: modelData.v; color: Theme.text; font.bold: true; font.pixelSize: 18 }
                        }
                    }
                }
            }

            // heatmap
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: 12
                color: Theme.panel
                border.color: Theme.border
                Column {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 8
                    Text { text: "12 × 12 Pozisyon Haritasi (144 motor)"; color: Theme.text; font.bold: true; font.pixelSize: 13 }
                    Heatmap {
                        width: parent.width
                        height: parent.height - 28
                    }
                }
            }

            // aksiyon cubugu
            Rectangle {
                Layout.fillWidth: true
                height: 120
                radius: 12
                color: Theme.panel
                border.color: Theme.border
                GridLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    columns: 4
                    columnSpacing: 10
                    rowSpacing: 10

                    RowLayout {
                        Layout.columnSpan: 4
                        spacing: 10
                        Text { text: "Hedef Slave:"; color: Theme.textDim; font.pixelSize: 13 }
                        SpinBox {
                            from: 1; to: 16
                            value: app.currentSlaveId
                            onValueModified: app.currentSlaveId = value
                        }
                        Text {
                            text: "Aktif: [" + app.activeSlaves.join(", ") + "]"
                            color: Theme.textDim; font.pixelSize: 12
                        }
                    }

                    ActionButton { text: "Bu Slave'e Gonder (ARR)"; color: Theme.accent
                        enabled: app.connected && app.gridData.length > 0
                        onClicked: app.sendArrayToSlave(app.currentSlaveId) }
                    ActionButton { text: "Tum Aktiflere Gonder"; color: Theme.purple
                        enabled: app.connected && app.gridData.length > 0
                        onClicked: app.sendArrayActive() }
                    ActionButton { text: "Slave Home"; color: Theme.orange
                        enabled: app.connected
                        onClicked: app.homeSlave(app.currentSlaveId) }
                    ActionButton { text: "Durum (STAT)"; color: Theme.green
                        enabled: app.connected
                        onClicked: app.requestStatus(app.currentSlaveId) }
                }
            }
        }
    }
}
