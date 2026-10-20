import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// MOTOR PANELI: uygulama ici SAYISAL TUS TAKIMI + tek motor eylemleri
// (K13: Qt VirtualKeyboard eklenmez). Pi kiosk'ta dokunmatik klavye yok.
//
// Baglam SABITTIR (inceleme bulgu 1): panel acildigi andaki modul/motor/baglanti
// oturumu (sid, motor, session) saklanir; komut YALNIZ bu degerlerle gonderilir.
// Baglam degisirse sahibi (TesterPage) paneli kapatir; panel hicbir zaman bir modulu
// gosterip baska module komut gondermez.
//
// Dogrulama backend'de (app.checkTarget: 10–590 mm, tam sayi). Eylem butonlari ne
// yapacagini acikca soyler ("Motor 3'ü 300 mm'ye götür", "Motor 3'e referans al").
// Kapali eylemin SEBEBI panelde tam metin yazilir. "Vazgeç" yalniz paneli kapatir
// (hicbir hareketi iptal etmez).
Item {
    id: root
    anchors.fill: parent
    visible: false
    z: 1950

    property int sid: 0                     // acilis anindaki modul (sabit)
    property int motor: 0                   // acilis anindaki motor (sabit)
    property int session: -1                // acilis anindaki baglanti oturumu
    property string title: ""
    property string info: ""
    property string entry: ""
    property string actionPrefix: ""        // or. "Motor 3'ü"
    property string homeText: ""            // or. "Motor 3'e referans al"
    property string blockReason: ""         // hareket kapisi ("" = izinli)
    property string homeBlockReason: ""     // referans alma kapisi ("" = izinli)
    readonly property string err: app.checkTarget(entry)
    signal accepted(int value)
    signal homeRequested()

    function open(s, m, t, i, prefix, home) {
        root.sid = s
        root.motor = m
        root.session = app.sessionId
        root.title = t
        root.info = i || ""
        root.actionPrefix = prefix || ""
        root.homeText = home || ""
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
        id: box
        objectName: "padBox"
        anchors.centerIn: parent
        width: Math.min(parent.width - 32, 980)
        height: Math.min(parent.height - 16, body.implicitHeight + 40)
        radius: Theme.radius
        color: Theme.panel
        border.color: Theme.border
        border.width: 2

        RowLayout {
            id: body
            anchors.fill: parent
            anchors.margins: 20
            spacing: 20

            // ---- sol: tus takimi ----
            GridLayout {
                Layout.preferredWidth: 330
                Layout.minimumWidth: 330
                Layout.maximumWidth: 330
                Layout.alignment: Qt.AlignTop
                columns: 3
                rowSpacing: 10; columnSpacing: 10
                Repeater {
                    model: ["1", "2", "3", "4", "5", "6", "7", "8", "9", "Sil", "0", "Temizle"]
                    delegate: TouchButton {
                        required property string modelData
                        objectName: "padKey_" + modelData
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

            // ---- sag: baglam + giris + eylemler ----
            ColumnLayout {
                Layout.fillWidth: true
                Layout.alignment: Qt.AlignTop
                spacing: 12

                Text {
                    objectName: "padTitle"
                    Layout.fillWidth: true
                    text: root.title
                    color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle
                    wrapMode: Text.WordWrap
                }
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
                        text: "Hedef: " + (root.entry.length ? root.entry : "—") + " mm"
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
                TouchButton {
                    objectName: "padAccept"
                    Layout.fillWidth: true
                    text: root.entry.length && !root.err.length
                          ? (root.actionPrefix + " " + root.entry + " mm'ye götür")
                          : "Önce hedef girin"
                    color: Theme.actionWarn
                    enabled: root.entry.length > 0 && root.err.length === 0 && root.blockReason.length === 0
                    disabledReason: root.blockReason
                    onClicked: {
                        root.visible = false
                        root.accepted(parseInt(root.entry))
                    }
                }
                TouchButton {
                    objectName: "padHome"
                    Layout.fillWidth: true
                    visible: root.homeText.length > 0
                    text: root.homeText
                    color: Theme.actionWarn
                    enabled: root.homeBlockReason.length === 0
                    disabledReason: root.homeBlockReason
                    onClicked: {
                        root.visible = false
                        root.homeRequested()
                    }
                }
                // kapali eylemlerin TAM sebebi (kisaltilmaz)
                Text {
                    objectName: "padMoveReason"
                    Layout.fillWidth: true
                    visible: root.blockReason.length > 0
                    text: "Hareket kapalı: " + root.blockReason
                    color: Theme.warn; font.pixelSize: Theme.fsSmall
                    wrapMode: Text.WordWrap
                }
                Text {
                    objectName: "padHomeReason"
                    Layout.fillWidth: true
                    visible: root.homeText.length > 0 && root.homeBlockReason.length > 0
                             && root.homeBlockReason !== root.blockReason
                    text: "Referans alma kapalı: " + root.homeBlockReason
                    color: Theme.warn; font.pixelSize: Theme.fsSmall
                    wrapMode: Text.WordWrap
                }
                TouchButton {
                    objectName: "padCancel"
                    text: "Vazgeç"
                    color: Theme.actionNeutral
                    onClicked: root.visible = false
                }
            }
        }
    }
}
