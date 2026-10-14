import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// BAKIM / MOTOR TESTI (Seviye-4 teknisyen ekrani; yogunluk icin touchDense bilincli sapma).
// v2.2:
//  - HEDEF ile OLCUM ayri: "Ölçülen" = modulun son STAT'i (backend modul deposu, yasi
//    gosterilir); "Hedef" = son KABUL EDILEN komut hedefi (backend). STAT hedefi ezmez.
//  - Secim degisince yalniz o modulun deposu gosterilir; veri yoksa "—" / "Durum bilinmiyor".
//  - Butun hareketler backend'in TEK yetki noktasindan gecer (testerMove/HomeBlockReason);
//    her hareket onayla veya eylemi acikca soyleyen butonla baslar.
//  - Hedef girisi uygulama ici sayisal tus takimi (NumPad); 10–590 mm (STL ile ayni marj).
Item {
    id: page
    property var logModel
    readonly property int sid: app.currentSlaveId
    property var info: { app.moduleRevision; return app.moduleInfo(page.sid) }

    function ageText(a) { return a < 0 ? "durum yok" : (a + " sn önce") }

    NumPad {
        id: pad
        objectName: "numPad"
        property int motor: 0
        blockReason: app.testerMoveBlockReason
        onAccepted: function(v) { app.moveMotor(page.sid, pad.motor, v) }
    }

    // Hareket baslatmadan once onay (fiziksel makine hareket edecek)
    AppDialog {
        id: confirm
        property string action: ""
        property int motor: 0
        function ask(what, m) {
            confirm.action = what
            confirm.motor = m || 0
            if (what === "all300")
                show("9 motoru 300 mm'ye götür",
                     "Modül " + page.sid + ": 9 motor 300 mm hedefine gidecek (kademeli, " +
                     "~75 sn'ye kadar).\nBu arayüzden hareket durdurulamıyor.",
                     "Götür", "Vazgeç", Theme.actionWarn)
            else if (what === "homeModule")
                show("Modül " + page.sid + Theme.dative(page.sid) + " referans al",
                     "Modül " + page.sid + ": 9 motor geri çekilip referans alacak " +
                     "(~150 sn'ye kadar).\nBu arayüzden hareket durdurulamıyor.",
                     "Referans al", "Vazgeç", Theme.actionWarn)
            else
                show("Motor " + m + Theme.dative(m) + " referans al",
                     "Modül " + page.sid + " / Motor " + m + " geri çekilip referans alacak." +
                     "\nBu arayüzden hareket durdurulamıyor.",
                     "Referans al", "Vazgeç", Theme.actionWarn)
        }
        onAccepted: {
            if (action === "all300") app.allToValue(page.sid, 300)
            else if (action === "homeModule") app.homeSlave(page.sid)
            else app.homeMotor(page.sid, confirm.motor)
        }
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 16

        // ---------- SOL: 16 modul panosu ----------
        Rectangle {
            Layout.preferredWidth: 404
            Layout.minimumWidth: 404
            Layout.maximumWidth: 404
            Layout.fillHeight: true
            radius: Theme.radius
            color: Theme.panel
            border.color: Theme.border
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 14
                spacing: 12
                Text { text: "Modüller"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                GridLayout {
                    id: modGrid
                    Layout.fillWidth: true
                    columns: 4
                    rowSpacing: 12; columnSpacing: 12
                    // karolar KARE: hucre genisligi = (grid - 3*bosluk)/4  (>= 80 px)
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
                            property int msid: index + 1
                            property bool sel: page.sid === msid
                            property var mi: { app.moduleRevision; return app.moduleInfo(msid) }
                            readonly property string tag:
                                !mi.known ? "yok"
                                : mi.alarm ? "ALARM"
                                : !mi.reachable ? "cevapsız"
                                : mi.moving ? "hareket"
                                : !mi.homed ? "ref. yok" : "bağlı"
                            // NORMAL durum notr; secim mavi cerceve; alarm kirmizi cerceve.
                            color: cell.sel ? Qt.rgba(0.29,0.44,0.65,0.35) : Theme.panelHi
                            opacity: mi.known || cell.sel ? 1.0 : 0.55
                            border.width: cell.sel || mi.alarm ? 3 : 1
                            border.color: mi.alarm ? Theme.alarm
                                        : cell.sel ? Theme.selection : Theme.border
                            Column {
                                anchors.centerIn: parent
                                spacing: 2
                                Text { text: cell.msid; color: Theme.text; font.bold: true
                                    font.pixelSize: Theme.fsTitle; font.family: Theme.fontMono
                                    anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: cell.tag
                                    color: cell.tag === "ALARM" ? Theme.alarm : Theme.textDim
                                    font.bold: cell.tag === "ALARM"
                                    font.pixelSize: Theme.fsSmall
                                    anchors.horizontalCenter: parent.horizontalCenter }
                            }
                            TapHandler { onTapped: app.currentSlaveId = cell.msid }
                        }
                    }
                }

                TouchButton {
                    Layout.fillWidth: true
                    text: "Seçili modülün durumunu sorgula"
                    color: Theme.actionNeutral
                    enabled: app.connected
                    disabledReason: "bağlantı yok"
                    onClicked: app.requestStatus(page.sid)
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    radius: Theme.radiusSm
                    color: Theme.panelHi
                    clip: true
                    Column {
                        anchors.fill: parent; anchors.margins: 10; spacing: 4
                        Text { text: "Kılavuz"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsBody }
                        Text {
                            width: parent.width
                            wrapMode: Text.WordWrap
                            text: "• Soldan modül seçin.\n" +
                                  "• Ölçülen = son durum sorgusu (yaşı üstte); Hedef = son kabul edilen komut.\n" +
                                  "• Hedef gir → tuş takımı → motor götürülür (" +
                                  app.safeMinMm + "–" + app.safeMaxMm + " mm).\n" +
                                  "• Makine sürekli izlenir; düğme yalnız seçili modülü hemen sorgular."
                            color: Theme.textDim; font.pixelSize: Theme.fsSmall; lineHeight: 1.2
                        }
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
                radius: Theme.radius
                color: Theme.panel
                border.color: Theme.border
                clip: true
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 10

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: Theme.gapMin
                        Column {
                            spacing: 2
                            Text { text: "Modül " + page.sid; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                            Text {
                                text: "Son ölçüm: " + page.ageText(page.info.ageS) +
                                      (page.info.known && page.info.stale ? " — güncel değil" : "") +
                                      "   ·   Referans: " +
                                      (page.info.homed ? "alındı (firmware bildirimi)" : "alınmadı")
                                color: page.info.known && page.info.stale ? Theme.warn : Theme.textDim
                                font.pixelSize: Theme.fsSmall
                            }
                        }
                        Item { Layout.fillWidth: true }
                        TouchButton {
                            text: "9 motoru 300 mm'ye götür"; color: Theme.actionWarn
                            enabled: app.testerMoveBlockReason === ""
                            disabledReason: app.testerMoveBlockReason
                            Layout.maximumWidth: 360
                            onClicked: confirm.ask("all300")
                        }
                        TouchButton {
                            text: "Modül " + page.sid + Theme.dative(page.sid) + " referans al"
                            color: Theme.actionWarn
                            enabled: app.testerHomeBlockReason === ""
                            disabledReason: app.testerHomeBlockReason
                            Layout.maximumWidth: 360
                            onClicked: confirm.ask("homeModule")
                        }
                    }

                    // neden kapali? (tam metin; buton altindaki satir kisaltilmis olabilir)
                    Text {
                        Layout.fillWidth: true
                        visible: app.testerMoveBlockReason.length > 0 || app.testerHomeBlockReason.length > 0
                        wrapMode: Text.WordWrap
                        maximumLineCount: 2
                        elide: Text.ElideRight
                        color: Theme.warn
                        font.pixelSize: Theme.fsSmall
                        text: (app.testerMoveBlockReason.length ? "Hareket kapalı: " + app.testerMoveBlockReason : "") +
                              (app.testerHomeBlockReason.length && app.testerHomeBlockReason !== app.testerMoveBlockReason
                               ? (app.testerMoveBlockReason.length ? "   ·   " : "") +
                                 "Referans alma kapalı: " + app.testerHomeBlockReason : "")
                    }

                    GridLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        columns: 3
                        rowSpacing: 12; columnSpacing: 12
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
                                    anchors.margins: 8
                                    spacing: 4

                                    // baslik + durum cipi (renk + METIN birlikte - WCAG 1.4.1)
                                    RowLayout {
                                        Layout.fillWidth: true
                                        Text { text: "MOTOR " + (mcell.index + 1); color: Theme.text
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
                                                color: (mcell.st === "F" || mcell.st === "H") ? "#101214"
                                                     : mcell.st === "" ? Theme.textFaint : Theme.textDim
                                                font.bold: true; font.pixelSize: Theme.fsSmall }
                                        }
                                    }

                                    Item { Layout.fillHeight: true }

                                    RowLayout {
                                        Layout.fillWidth: true
                                        spacing: 10
                                        Column {
                                            Layout.preferredWidth: 110
                                            Text { text: "Ölçülen"; color: Theme.textFaint; font.pixelSize: Theme.fsSmall }
                                            Text {
                                                text: mcell.mi.known ? (mcell.mi.mm + " mm") : "—"
                                                color: Theme.text; font.bold: true
                                                font.pixelSize: Theme.fsTitle; font.family: Theme.fontMono
                                            }
                                        }
                                        Column {
                                            Layout.preferredWidth: 110
                                            Text { text: "Hedef"; color: Theme.textFaint; font.pixelSize: Theme.fsSmall }
                                            Text {
                                                text: mcell.mi.target >= 0 ? (mcell.mi.target + " mm") : "—"
                                                color: Theme.textDim; font.bold: true
                                                font.pixelSize: Theme.fsTitle; font.family: Theme.fontMono
                                            }
                                        }
                                        Item { Layout.fillWidth: true }
                                        TouchButton {
                                            text: "Hedef gir"
                                            dense: true
                                            color: Theme.actionNeutral
                                            enabled: app.testerMoveBlockReason === ""
                                            onClicked: {
                                                pad.motor = mcell.index + 1
                                                pad.open("Modül " + page.sid + " / Motor " + (mcell.index + 1) + " hedefi",
                                                         "Ölçülen: " + (mcell.mi.known ? mcell.mi.mm + " mm" : "bilinmiyor") +
                                                         " (" + page.ageText(page.info.ageS) + ")",
                                                         "Motor " + (mcell.index + 1) + Theme.accusative(mcell.index + 1))
                                            }
                                        }
                                        TouchButton {
                                            text: "Ref."
                                            dense: true
                                            color: Theme.actionWarn
                                            enabled: app.testerHomeBlockReason === ""
                                            onClicked: confirm.ask("homeMotor", mcell.index + 1)
                                        }
                                    }
                                    Item { Layout.fillHeight: true }
                                }
                            }
                        }
                    }
                }
            }

            Terminal {
                Layout.fillWidth: true
                Layout.preferredHeight: 216
                dense: true
                logModel: page.logModel
            }
        }
    }
}
