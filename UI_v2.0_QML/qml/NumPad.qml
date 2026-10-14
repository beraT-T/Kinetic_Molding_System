import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// Uygulama ici SAYISAL TUS TAKIMI (K13: Qt VirtualKeyboard eklenmez, yeni bagimlilik yok).
// Pi kiosk'ta dokunmatik klavye yok; "deger yaz + Enter" yerine bu kullanilir.
// Dogrulama backend'de (app.checkTarget: 10–590 mm, tam sayi). Eylem butonu hareketi
// BASLATIR; metni ne yapacagini acikca soyler (or. "Motor 3'ü 300 mm'ye götür").
// "Vazgeç" yalniz pencereyi kapatir (hicbir hareketi iptal etmez).
Item {
    id: root
    anchors.fill: parent
    visible: false
    z: 1950

    property string title: ""
    property string info: ""
    property string entry: ""
    property string actionPrefix: ""        // or. "Motor 3'ü"
    property string blockReason: ""         // hareket kapisi ("" = izinli)
    readonly property string err: app.checkTarget(entry)
    signal accepted(int value)

    function open(t, i, prefix) {
        root.title = t
        root.info = i || ""
        root.actionPrefix = prefix || ""
        root.entry = ""
        root.visible = true
    }
    function key(k) { if (root.entry.length < 3) root.entry += k }
    function back() { root.entry = root.entry.substring(0, root.entry.length - 1) }

    Rectangle {
        anchors.fill: parent
        color: "#cc000000"
        MouseArea { anchors.fill: parent }
    }

    Rectangle {
        anchors.centerIn: parent
        width: 600
        height: col.implicitHeight + 48
        radius: Theme.radius
        color: Theme.panel
        border.color: Theme.border
        border.width: 2

        ColumnLayout {
            id: col
            anchors.centerIn: parent
            width: parent.width - 48
            spacing: 12

            Text { text: root.title; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
            Text {
                Layout.fillWidth: true
                visible: root.info.length > 0
                text: root.info
                color: Theme.textDim; font.pixelSize: Theme.fsBody
                wrapMode: Text.WordWrap
            }
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: Theme.inputHeight
                radius: Theme.radiusSm
                color: Theme.panelHi
                border.color: root.entry.length && root.err.length ? Theme.warn : Theme.border
                border.width: 2
                Text {
                    anchors.centerIn: parent
                    text: (root.entry.length ? root.entry : "—") + " mm"
                    color: Theme.text; font.bold: true
                    font.pixelSize: Theme.fsValue; font.family: Theme.fontMono
                }
            }
            Text {
                Layout.fillWidth: true
                text: root.entry.length && root.err.length ? root.err
                      : ("Geçerli aralık " + app.safeMinMm + "–" + app.safeMaxMm + " mm")
                color: root.entry.length && root.err.length ? Theme.warn : Theme.textDim
                font.pixelSize: Theme.fsSmall
                wrapMode: Text.WordWrap
            }

            GridLayout {
                Layout.fillWidth: true
                columns: 3
                rowSpacing: 10; columnSpacing: 10
                Repeater {
                    model: ["1", "2", "3", "4", "5", "6", "7", "8", "9", "Sil", "0", "Temizle"]
                    delegate: TouchButton {
                        required property string modelData
                        Layout.fillWidth: true
                        text: modelData
                        color: (modelData === "Sil" || modelData === "Temizle")
                               ? Theme.actionNeutral : Theme.panelHi
                        onClicked: {
                            if (modelData === "Sil") root.back()
                            else if (modelData === "Temizle") root.entry = ""
                            else root.key(modelData)
                        }
                    }
                }
            }

            Text {
                Layout.fillWidth: true
                visible: root.blockReason.length > 0
                text: "Hareket kapalı: " + root.blockReason
                color: Theme.warn; font.pixelSize: Theme.fsSmall
                wrapMode: Text.WordWrap
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gapMin
                TouchButton {
                    text: "Vazgeç"
                    color: Theme.actionNeutral
                    onClicked: root.visible = false
                }
                TouchButton {
                    Layout.fillWidth: true
                    text: root.entry.length && !root.err.length
                          ? (root.actionPrefix + " " + root.entry + " mm'ye götür")
                          : "Hedef girin"
                    color: Theme.actionWarn
                    enabled: root.entry.length > 0 && root.err.length === 0 && root.blockReason.length === 0
                    onClicked: {
                        root.visible = false
                        root.accepted(parseInt(root.entry))
                    }
                }
            }
        }
    }
}
