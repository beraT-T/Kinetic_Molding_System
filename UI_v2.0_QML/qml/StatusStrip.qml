import QtQuick
import QtQuick.Layouts
import "Theme.js" as Theme

// Kalici durum/alarm seridi (ISA-101: anormal durum her ekrandan gorunur olmali).
// Normalde NOTR; yalniz anormal durumda renk alir. Renk tek basina bilgi tasimaz
// (WCAG 1.4.1) -> sembol + metin birlikte verilir.
Rectangle {
    id: root
    height: Theme.stripHeight

    // oncelik: ariza (kirmizi) > baglanti/on-kosul (amber) > normal (notr)
    readonly property bool hasFault: app.faultText.length > 0
    readonly property bool notConnected: !app.connected
    readonly property bool noModules: app.connected && app.activeSlaves.length === 0
    readonly property bool noRef: app.connected && app.activeSlaves.length > 0 && !app.allHomed

    readonly property int level: hasFault ? 2
                               : (notConnected || noModules || noRef) ? 1 : 0

    color: level === 2 ? Theme.alarm
         : level === 1 ? Theme.warn
         : Theme.bg2
    border.color: level === 0 ? Theme.border : "transparent"
    border.width: 1

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 16
        anchors.rightMargin: 16
        spacing: Theme.gapMin

        // sembol (renk korlugu icin renkten bagimsiz isaret)
        Text {
            text: root.level === 2 ? "⚠" : (root.level === 1 ? "⚠" : "✓")
            color: root.level === 0 ? Theme.textDim : "#101214"
            font.pixelSize: Theme.fsAlarm
            font.bold: true
        }

        Text {
            Layout.fillWidth: true
            elide: Text.ElideRight
            font.pixelSize: Theme.fsAlarm
            font.bold: root.level > 0
            color: root.level === 0 ? Theme.textDim : "#101214"
            text: root.hasFault      ? app.faultText
                : root.notConnected  ? "BAGLANTI YOK - cihaza baglanin"
                : root.noModules     ? "MODUL BULUNAMADI - 'Modulleri Bul'"
                : root.noRef         ? "REFERANS ALINMADI - islem oncesi home yapilacak"
                                     : "Anormal durum yok"
        }

        // ariza kaydini temizle (yalniz ariza varken)
        Rectangle {
            visible: root.hasFault
            implicitWidth: ackTxt.implicitWidth + 28
            implicitHeight: 40
            radius: Theme.radiusSm
            color: "#101214"
            Text {
                id: ackTxt
                anchors.centerIn: parent
                text: "ONAYLA"
                color: "#e8eaed"
                font.pixelSize: Theme.fsSmall
                font.bold: true
            }
            TapHandler { onTapped: app.clearFault() }
        }
    }
}
