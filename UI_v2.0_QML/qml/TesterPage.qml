import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// BAKIM / MOTOR TESTI (Seviye-4 teknisyen ekrani; yogunluk icin touchDense bilincli sapma).
// v2.2:
//  - HEDEF ile OLCUM ayri: "Ölçülen" = modulun son STAT'i (backend modul deposu, yasi
//    gosterilir); "Hedef" = son KABUL EDILEN komut hedefi (backend). STAT hedefi ezmez.
//  - Veri guncel dogrulanmis degilse (eski, hat sirasi belirsiz, eksik, baglanti yok)
//    "Ölçülen" yerine "Son bilinen" yazilir (son bilinen != guncel dogrulanmis).
//  - Butun hareketler backend'in TEK yetki noktasindan gecer (testerMove/HomeBlockReason);
//    her hareket onayla veya eylemi acikca soyleyen butonla baslar.
// Inceleme duzeltmesi (2026-10-19):
//  - Motor hucresi butonsuz; hucrenin TAMAMI dokunma hedefidir ve MOTOR PANELI'ni acar
//    (tus takimi + "Motor N'e referans al"). 1080p'de hucre icinde buton tasmasi yok.
//  - Panel ve onay penceresi acildigi andaki modul/motor/baglanti oturumunu SAKLAR ve
//    komutu yalniz onunla gonderir; secili modul, baglanti veya ekran degisirse pencere
//    KAPATILIR (komut gonderilmez, gunluge yazilir). Baslik bir modulu gosterip baska
//    module komut gitmez.
Item {
    id: page
    property var logModel
    readonly property int sid: app.currentSlaveId
    property var info: { app.moduleRevision; return app.moduleInfo(page.sid) }

    function ageText(a) { return a < 0 ? "durum yok" : (a + " sn önce") }

    // ---- baglam degisince acik pencereleri gecersiz kil ----
    function closeDialogs(why) {
        if (pad.visible) {
            pad.visible = false
            app.logUi("Motor paneli kapatıldı: " + why + " (komut gönderilmedi)")
        }
        if (confirm.visible) {
            confirm.visible = false
            app.logUi("Onay penceresi kapatıldı: " + why + " (komut gönderilmedi)")
        }
    }
    onVisibleChanged: if (!visible) closeDialogs("Bakım ekranından çıkıldı")
    onSidChanged: closeDialogs("seçili modül değişti")
    Connections {
        target: app
        function onSessionChanged() { page.closeDialogs("bağlantı değişti") }
    }
    function contextOk(s, sess) { return s === page.sid && sess === app.sessionId }

    function openMotor(m) {
        var mi = page.info.motors[m - 1]
        var meas = mi.known ? (mi.mm + " mm (" + (page.info.current ? "ölçülen" : "son bilinen") +
                               ", " + page.ageText(page.info.ageS) + ")") : "bilinmiyor"
        pad.open(page.sid, m, "Modül " + page.sid + " / Motor " + m,
                 "Konum: " + meas + "\nSon kabul edilen hedef: " +
                 (mi.target >= 0 ? mi.target + " mm" : "—") + "   ·   Durum: " +
                 Theme.stateLabel(mi.known ? mi.letter : ""),
                 "Motor " + m + Theme.accusative(m),
                 "Motor " + m + Theme.dative(m) + " referans al")
    }

    NumPad {
        id: pad
        objectName: "numPad"
        blockReason: app.testerMoveBlockReason
        homeBlockReason: app.testerHomeBlockReason
        onAccepted: function(v) {
            if (!page.contextOk(pad.sid, pad.session)) {
                app.logUi("Motor paneli bağlamı değişti; komut gönderilmedi")
                return
            }
            app.moveMotor(pad.sid, pad.motor, v)
        }
        onHomeRequested: confirm.ask("homeMotor", pad.sid, pad.motor)
    }

    // Hareket baslatmadan once onay (fiziksel makine hareket edecek). Modul/motor ve
    // baglanti oturumu ACILISTA sabitlenir.
    AppDialog {
        id: confirm
        objectName: "testerConfirm"
        property string action: ""
        property int csid: 0
        property int motor: 0
        property int session: -1
        function ask(what, s, m) {
            confirm.action = what
            confirm.csid = s
            confirm.motor = m || 0
            confirm.session = app.sessionId
            if (what === "all300")
                show("9 motoru 300 mm'ye götür",
                     "Modül " + s + ": 9 motor 300 mm hedefine gidecek (kademeli, " +
                     "~75 sn'ye kadar).\nBu arayüzden hareket durdurulamıyor.",
                     "Götür", "Vazgeç", Theme.actionWarn)
            else if (what === "homeModule")
                show("Modül " + s + Theme.dative(s) + " referans al",
                     "Modül " + s + ": 9 motor geri çekilip referans alacak " +
                     "(~150 sn'ye kadar).\nBu arayüzden hareket durdurulamıyor.",
                     "Referans al", "Vazgeç", Theme.actionWarn)
            else
                show("Modül " + s + " / Motor " + m + Theme.dative(m) + " referans al",
                     "Modül " + s + " / Motor " + m + " geri çekilip referans alacak." +
                     "\nBu, modül referansını tamamlamaz: motor hareketi için modülün tamamına " +
                     "referans alınmalıdır.\nBu arayüzden hareket durdurulamıyor.",
                     "Referans al", "Vazgeç", Theme.actionWarn)
        }
        onAccepted: {
            if (!page.contextOk(csid, session)) {
                app.logUi("Onay bağlamı değişti; komut gönderilmedi")
                return
            }
            if (action === "all300") app.allToValue(csid, 300)
            else if (action === "homeModule") app.homeSlave(csid)
            else app.homeMotor(csid, confirm.motor)
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
                spacing: 10
                Text { text: "Modüller"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                GridLayout {
                    id: modGrid
                    Layout.fillWidth: true
                    columns: 4
                    rowSpacing: 10; columnSpacing: 12
                    // karolar KARE: hucre genisligi = (grid - 3*bosluk)/4  (>= 80 px)
                    readonly property real cellSize:
                        Math.max(Theme.touchMin, (width - 3 * columnSpacing) / 4)
                    Repeater {
                        model: 16
                        delegate: Rectangle {
                            id: cell
                            objectName: "modTile_" + (index + 1)
                            required property int index
                            Layout.fillWidth: true
                            Layout.preferredHeight: modGrid.cellSize
                            radius: Theme.radiusSm
                            property int msid: index + 1
                            property bool sel: page.sid === msid
                            property var mi: { app.moduleRevision; return app.moduleInfo(msid) }
                            readonly property string tag:
                                mi.lost ? "cevapsız"
                                : !mi.known ? "yok"
                                : mi.alarm ? "ALARM"
                                : !mi.reachable ? "cevapsız"
                                : mi.moving ? "hareket"
                                : !mi.homed ? "referans yok" : "bağlı"
                            // NORMAL durum notr; secim mavi cerceve; alarm kirmizi cerceve.
                            color: cell.sel ? Qt.rgba(0.29,0.44,0.65,0.35) : Theme.panelHi
                            opacity: mi.known || mi.lost || cell.sel ? 1.0 : 0.55
                            border.width: cell.sel || mi.alarm ? 3 : 1
                            border.color: mi.alarm ? Theme.alarm
                                        : cell.sel ? Theme.selection : Theme.border
                            Column {
                                anchors.centerIn: parent
                                width: parent.width - 8
                                spacing: 0
                                Text { text: cell.msid; color: Theme.text; font.bold: true
                                    font.pixelSize: Theme.fsTitle; font.family: Theme.fontMono
                                    anchors.horizontalCenter: parent.horizontalCenter }
                                Text {
                                    objectName: "modTag_" + cell.msid
                                    width: parent.width
                                    horizontalAlignment: Text.AlignHCenter
                                    wrapMode: Text.WordWrap
                                    maximumLineCount: 2
                                    text: cell.tag
                                    color: cell.tag === "ALARM" ? Theme.alarm
                                         : cell.tag === "cevapsız" ? Theme.warn : Theme.textDim
                                    font.bold: cell.tag === "ALARM"
                                    font.pixelSize: Theme.fsSmall
                                }
                            }
                            TapHandler { onTapped: app.currentSlaveId = cell.msid }
                        }
                    }
                }

                TouchButton {
                    Layout.fillWidth: true
                    dense: true
                    text: "Seçili modülün durumunu sorgula"
                    color: Theme.actionNeutral
                    enabled: app.connected
                    disabledReason: "bağlantı yok"
                    onClicked: app.requestStatus(page.sid)
                }

                Rectangle {
                    objectName: "guideBox"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    radius: Theme.radiusSm
                    color: Theme.panelHi
                    clip: true
                    Column {
                        anchors.fill: parent; anchors.margins: 10; spacing: 4
                        Text { text: "Kılavuz"; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsBody }
                        Text {
                            objectName: "guideText"
                            width: parent.width
                            wrapMode: Text.WordWrap
                            text: "• Motora dokunun: hedef girin veya referans alın (" +
                                  app.safeMinMm + "–" + app.safeMaxMm + " mm).\n" +
                                  "• Ölçülen: güncel durum sorgusu. Son bilinen: güncel değil.\n" +
                                  "• Hedef: son kabul edilen komut."
                            color: Theme.textDim; font.pixelSize: Theme.fsSmall; lineHeight: 1.15
                        }
                    }
                }
            }
        }

        // ---------- SAG: motor kontrol + terminal ----------
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: Theme.radius
                color: Theme.panel
                border.color: Theme.border
                clip: true
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: Theme.gapMin
                        Column {
                            Layout.fillWidth: true
                            spacing: 2
                            Text { text: "Modül " + page.sid; color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle }
                            Text {
                                objectName: "testerAge"
                                width: parent.width
                                elide: Text.ElideRight
                                text: (page.info.current ? "Son ölçüm: " : "Son bilinen durum: ") +
                                      page.ageText(page.info.ageS) +
                                      (page.info.known && !page.info.current ? " — güncel değil" : "") +
                                      "   ·   Referans: " +
                                      (page.info.homed ? "alındı (firmware bildirimi)" : "alınmadı")
                                color: page.info.known && !page.info.current ? Theme.warn : Theme.textDim
                                font.pixelSize: Theme.fsSmall
                            }
                        }
                        TouchButton {
                            objectName: "btnAll300"
                            dense: true
                            text: "9 motoru 300 mm'ye götür"; color: Theme.actionWarn
                            enabled: app.testerMoveBlockReason === ""
                            disabledReason: app.testerMoveBlockReason
                            Layout.maximumWidth: 360
                            onClicked: confirm.ask("all300", page.sid)
                        }
                        TouchButton {
                            objectName: "btnHomeModule"
                            dense: true
                            text: "Modül " + page.sid + Theme.dative(page.sid) + " referans al"
                            color: Theme.actionWarn
                            enabled: app.testerHomeBlockReason === ""
                            disabledReason: app.testerHomeBlockReason
                            Layout.maximumWidth: 360
                            onClicked: confirm.ask("homeModule", page.sid)
                        }
                    }

                    // neden kapali? (tam metin; buton altindaki satir kisaltilmis olabilir)
                    Text {
                        objectName: "testerReason"
                        Layout.fillWidth: true
                        visible: app.testerMoveBlockReason.length > 0 || app.testerHomeBlockReason.length > 0
                        wrapMode: Text.WordWrap
                        maximumLineCount: 3
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
                        rowSpacing: 10; columnSpacing: 12
                        Repeater {
                            model: 9
                            // Hucre = tek dokunma hedefi (>= 80 px yukseklik). Butun eylemler
                            // motor panelinde, kapali olanlarin sebebiyle birlikte.
                            delegate: Rectangle {
                                id: mcell
                                objectName: "motorCell_" + (index + 1)
                                required property int index
                                readonly property var mi: page.info.motors[index]
                                readonly property string st: mi.known ? mi.letter : ""
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                Layout.minimumHeight: Theme.touchMin
                                radius: Theme.radiusSm
                                color: tap.pressed ? Qt.darker(Theme.panelHi, 1.25) : Theme.panelHi
                                border.color: mcell.st === "F" ? Theme.alarm : Theme.border
                                border.width: mcell.st === "F" ? 3 : 1

                                TapHandler { id: tap; onTapped: page.openMotor(mcell.index + 1) }

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 8
                                    spacing: 4

                                    // baslik + durum cipi (renk + METIN birlikte - WCAG 1.4.1)
                                    RowLayout {
                                        Layout.fillWidth: true
                                        Text { objectName: "motorTitle_" + (mcell.index + 1)
                                            text: "Motor " + (mcell.index + 1); color: Theme.text
                                            font.pixelSize: Theme.fsBody; font.bold: true }
                                        Text {
                                            visible: mcell.mi.referenced && !page.info.homed
                                            text: "· motor referanslı"
                                            color: Theme.textDim; font.pixelSize: Theme.fsSmall
                                        }
                                        Item { Layout.fillWidth: true }
                                        Rectangle {
                                            objectName: "motorState_" + (mcell.index + 1)
                                            radius: Theme.radiusSm
                                            implicitHeight: 28
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
                                        spacing: 16
                                        Column {
                                            Layout.preferredWidth: 140
                                            Text { objectName: "measLabel_" + (mcell.index + 1)
                                                text: page.info.current ? "Ölçülen" : "Son bilinen"
                                                color: page.info.current || !mcell.mi.known ? Theme.textFaint : Theme.warn
                                                font.pixelSize: Theme.fsSmall }
                                            Text {
                                                objectName: "measValue_" + (mcell.index + 1)
                                                text: mcell.mi.known ? (mcell.mi.mm + " mm") : "—"
                                                color: page.info.current ? Theme.text : Theme.textDim
                                                font.bold: true
                                                font.pixelSize: Theme.fsTitle; font.family: Theme.fontMono
                                            }
                                        }
                                        Column {
                                            Layout.preferredWidth: 140
                                            Text { text: "Hedef"; color: Theme.textFaint; font.pixelSize: Theme.fsSmall }
                                            Text {
                                                objectName: "targetValue_" + (mcell.index + 1)
                                                text: mcell.mi.target >= 0 ? (mcell.mi.target + " mm") : "—"
                                                color: Theme.textDim; font.bold: true
                                                font.pixelSize: Theme.fsTitle; font.family: Theme.fontMono
                                            }
                                        }
                                        Item { Layout.fillWidth: true }
                                        Text {
                                            objectName: "motorHint_" + (mcell.index + 1)
                                            text: "Dokunun ›"
                                            color: Theme.textFaint; font.pixelSize: Theme.fsSmall
                                            Layout.alignment: Qt.AlignBottom
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            Terminal {
                Layout.fillWidth: true
                Layout.preferredHeight: 190
                dense: true
                logModel: page.logModel
            }
        }
    }
}
