import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// Tam ekran, buyuk satirli, dokunmatik STL dosya secici.
// Dosya listesini controller.listStlFiles(dir) verir (QML'de FolderListModel yok).
Item {
    id: root
    anchors.fill: parent
    visible: false
    property string currentDir: ""
    property var entries: []
    signal picked(string path)

    function openAt(dir) {
        var dirs = app.stlSearchDirs()
        root.currentDir = (dir && dir.length) ? dir : (dirs.length ? dirs[0].path : "/")
        reload()
        root.visible = true
    }
    function reload() { root.entries = app.listStlFiles(root.currentDir) }
    function goUp() { root.currentDir = app.parentDir(root.currentDir); reload() }

    z: 1000   // tum arayuzun uzerinde

    // karartma arka plan; arkadaki dokunuslari yutar
    // NOT: QML 8-hane hex = #AARRGGBB (alpha once). "#cc000000" = %80 siyah.
    Rectangle {
        anchors.fill: parent
        color: "#cc000000"
        MouseArea { anchors.fill: parent }
    }

    Rectangle {
        anchors.centerIn: parent
        width: parent.width * 0.82
        height: parent.height * 0.86
        radius: Theme.radius
        color: Theme.panel
        border.color: Theme.border

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 18
            spacing: Theme.gapMin

            // baslik + kapat
            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gapMin
                Text { text: "STL Dosyasi Sec"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                Item { Layout.fillWidth: true }
                TouchButton { text: "Kapat"; color: Theme.panelHi; onClicked: root.visible = false }
            }

            // ust klasor + hizli klasorler
            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gapMin
                TouchButton { text: "Ust Klasor"; color: Theme.panelHi; onClicked: root.goUp() }
                Repeater {
                    model: app.stlSearchDirs()
                    delegate: TouchButton {
                        required property var modelData
                        text: modelData.name
                        color: Theme.bg2
                        onClicked: { root.currentDir = modelData.path; root.reload() }
                    }
                }
                Item { Layout.fillWidth: true }
            }

            // gecerli yol
            Text {
                Layout.fillWidth: true
                text: root.currentDir
                color: Theme.textDim
                font.pixelSize: Theme.fsSmall
                elide: Text.ElideMiddle
            }

            // dosya/klasor listesi
            ListView {
                id: list
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.entries
                spacing: 6
                ScrollBar.vertical: ScrollBar { width: 16 }

                delegate: Rectangle {
                    id: row
                    required property var modelData
                    width: ListView.view.width
                    height: Theme.rowHeight
                    radius: Theme.radiusSm
                    color: tap.pressed ? Theme.panelHi : Theme.bg2
                    border.color: Theme.border

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 16; anchors.rightMargin: 16
                        spacing: Theme.gapMin

                        // tip etiketi (emoji yok - Pi font guvenli)
                        Rectangle {
                            width: 62; height: 30; radius: Theme.radiusSm
                            color: row.modelData.isDir ? Theme.accent : Theme.green
                            Text { anchors.centerIn: parent
                                text: row.modelData.isDir ? "KLASOR" : "STL"
                                color: "white"; font.bold: true; font.pixelSize: Theme.fsSmall }
                        }
                        Text {
                            Layout.fillWidth: true
                            text: row.modelData.name
                            color: Theme.text
                            font.pixelSize: Theme.fsButton
                            font.bold: !row.modelData.isDir
                            elide: Text.ElideMiddle
                        }
                        Text { visible: !row.modelData.isDir; text: row.modelData.size
                            color: Theme.textDim; font.pixelSize: Theme.fsSmall }
                        Text { visible: !row.modelData.isDir; text: row.modelData.mtime
                            color: Theme.textDim; font.pixelSize: Theme.fsSmall }
                    }

                    TapHandler {
                        id: tap
                        onTapped: {
                            if (row.modelData.isDir) { root.currentDir = row.modelData.path; root.reload() }
                            else { root.picked(row.modelData.path); root.visible = false }
                        }
                    }
                }

                // bos klasor bilgisi
                Text {
                    anchors.centerIn: parent
                    visible: list.count === 0
                    text: "Bu klasorde .stl dosyasi yok"
                    color: Theme.textDim
                    font.pixelSize: Theme.fsBody
                }
            }
        }
    }
}
