import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtQuick.Dialogs
import "Theme.js" as Theme

Item {
    id: page
    property var logModel

    // yuklu STL adi (app.stlFileUrl'den turetilir)
    function stlName() {
        var u = app.stlFileUrl.toString()
        if (!u.length) return ""
        return decodeURIComponent(u.split("/").pop())
    }

    // Native QML dosya secici (kullanici tercihi)
    FileDialog {
        id: fileDialog
        title: "STL dosyasi sec"
        nameFilters: ["STL dosyalari (*.stl)"]
        onAccepted: app.loadStl(selectedFile)
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 16

        // ---------- SOL: STL + 3D + terminal ----------
        // Sabit genislikli kontrol sutunu. NOT: RowLayout icinde ic-ice
        // ColumnLayout sadece preferredWidth ile genisligini tutmaz (fillWidth
        // kardesini sifira sikistirir); min=max ile sabitlenir.
        ColumnLayout {
            Layout.preferredWidth: 520
            Layout.minimumWidth: 520
            Layout.maximumWidth: 520
            Layout.fillHeight: true
            spacing: 14

            // dosya
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: Theme.btnHeight + 28
                radius: Theme.radius
                color: Theme.panel
                border.color: Theme.border
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: Theme.gapMin
                    Column {
                        Layout.fillWidth: true
                        spacing: 4
                        Text { text: "STL Modeli"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsBody }
                        Text {
                            text: page.stlName().length ? page.stlName() : "Dosya secilmedi"
                            color: page.stlName().length ? Theme.accent2 : Theme.textDim
                            font.pixelSize: Theme.fsSmall; elide: Text.ElideMiddle; width: 280
                        }
                    }
                    TouchButton {
                        text: "STL Sec + Hesapla"
                        color: Theme.accent
                        onClicked: fileDialog.open()
                    }
                }
            }

            // 3D (STL / Kalip gorunumu Surface3D icinde)
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 340
                radius: Theme.radius
                color: Theme.panel
                border.color: Theme.border
                clip: true
                Column {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 6
                    Text { text: "3D Onizleme"; color: Theme.textDim; font.bold: true; font.pixelSize: Theme.fsSmall }
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
                Layout.preferredHeight: 68
                radius: Theme.radius
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
                            Text { text: modelData.k; color: Theme.textDim; font.pixelSize: Theme.fsSmall }
                            Text { text: modelData.v; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                        }
                    }
                }
            }

            // heatmap
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: Theme.radius
                color: Theme.panel
                border.color: Theme.border
                Column {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 8
                    Text { text: "12 × 12 Pozisyon Haritasi (144 motor)"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsBody }
                    Heatmap {
                        width: parent.width
                        height: parent.height - 28
                    }
                }
            }

            // aksiyon cubugu
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: Theme.inputHeight + Theme.btnHeight + 48
                radius: Theme.radius
                color: Theme.panel
                border.color: Theme.border
                GridLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    columns: 4
                    columnSpacing: Theme.gapMin
                    rowSpacing: Theme.gapMin

                    RowLayout {
                        Layout.columnSpan: 4
                        spacing: Theme.gapMin
                        Text { text: "Hedef Modul:"; color: Theme.textDim; font.pixelSize: Theme.fsBody }
                        SpinBox {
                            from: 1; to: 16
                            value: app.currentSlaveId
                            onValueModified: app.currentSlaveId = value
                            Layout.preferredHeight: Theme.inputHeight
                            font.pixelSize: Theme.fsButton
                        }
                        Text {
                            text: "Bagli: [" + app.activeSlaves.join(", ") + "]"
                            color: Theme.textDim; font.pixelSize: Theme.fsSmall
                        }
                    }

                    ActionButton { text: "Secili Module Uygula"; color: Theme.accent
                        enabled: app.connected && app.gridData.length > 0
                        onClicked: app.sendArrayToSlave(app.currentSlaveId) }
                    ActionButton { text: "Tum Modullere Uygula"; color: Theme.purple
                        enabled: app.connected && app.gridData.length > 0
                        onClicked: app.sendArrayActive() }
                    ActionButton { text: "Home"; color: Theme.orange
                        enabled: app.connected
                        onClicked: app.homeSlave(app.currentSlaveId) }
                    ActionButton { text: "Basla"; color: Theme.green
                        enabled: app.connected
                        onClicked: app.requestStatus(app.currentSlaveId) }
                }
            }
        }
    }
}
