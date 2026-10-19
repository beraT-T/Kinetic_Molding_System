import QtQuick
import QtQuick.Layouts
import "Theme.js" as Theme

// Kalici durum/alarm seridi (ISA-101: anormal durum her ekrandan gorunur olmali).
// Metin ve oncelik backend'de hesaplanir (app.statusInfo) - test edilebilir tek yer:
//   aktif ariza > aktif uyari > baglanti yok > giderildi/onay bekliyor > modul yok
//   > veri eski ("Güncel makine durumu bilinmiyor.") > hareket > referans yok > normal.
// "Uyarıyı gördüm" ONAYDIR, kaydi silmez; kayit yalniz taze gecerli durumla kapanir.
// Renk tek basina bilgi tasimaz (WCAG 1.4.1) -> sembol + metin birlikte.
Rectangle {
    id: root
    height: Theme.stripHeight
    signal openAlarms()

    readonly property var st: app.statusInfo
    readonly property int level: (st && st.level !== undefined) ? st.level : 1
    readonly property color ink: level === 0 ? Theme.textDim : "#101214"

    color: level === 2 ? Theme.alarm
         : level === 1 ? Theme.warn
         : Theme.bg2
    border.color: level === 0 ? Theme.border : "transparent"
    border.width: 1

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 16
        anchors.rightMargin: 4
        spacing: Theme.gapMin

        // sembol (renk korlugu icin renkten bagimsiz isaret)
        Text {
            text: root.level > 0 ? "⚠" : "●"
            color: root.ink
            font.pixelSize: Theme.fsAlarm
            font.bold: true
        }

        Text {
            Layout.fillWidth: true
            elide: Text.ElideRight
            font.pixelSize: Theme.fsAlarm
            font.bold: root.level > 0
            color: root.ink
            text: (root.st && root.st.text) ? root.st.text : ""
            TapHandler { onTapped: root.openAlarms() }
        }

        // alarm/uyari listesi (kayitlar burada ayrintili)
        Rectangle {
            visible: app.alarms.length > 0
            Layout.preferredHeight: Theme.touchMin
            implicitWidth: listTxt.implicitWidth + 40
            radius: Theme.radiusSm
            color: root.level === 0 ? Theme.actionNeutral : "#101214"
            Text {
                id: listTxt
                anchors.centerIn: parent
                text: "Alarmlar (" + app.alarms.length + ")"
                color: "#e8eaed"
                font.pixelSize: Theme.fsButton
                font.bold: true
            }
            TapHandler { onTapped: root.openAlarms() }
        }

        // ONAY: kaydi SILMEZ (ISA-18.2: onay != normale donus)
        Rectangle {
            objectName: "stripAck"
            visible: app.unackedAlarms > 0
            Layout.preferredHeight: Theme.touchMin
            implicitWidth: ackTxt.implicitWidth + 40
            radius: Theme.radiusSm
            color: "#101214"
            border.color: "#e8eaed"
            border.width: 2
            Text {
                id: ackTxt
                anchors.centerIn: parent
                text: "Uyarıyı gördüm"
                color: "#e8eaed"
                font.pixelSize: Theme.fsButton
                font.bold: true
            }
            TapHandler { onTapped: app.acknowledgeAlarms() }
        }
    }
}
