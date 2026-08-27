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

    // Hareket baslatmadan once onay (fiziksel makine hareket edecek)
    AppDialog {
        id: confirmDialog
        property string action: ""
        function ask(what) {
            confirmDialog.action = what
            if (what === "production") {
                // Referans uyarisi burada cikar (baglantida degil).
                // Tum moduller referans aldiysa home tekrarlanmaz.
                if (!app.allHomed)
                    show("Referans alinmadi",
                         "Bazi moduller bu oturumda henuz referans almadi.\n" +
                         "Once tum bagli modullerde home yapilacak, ardindan model " +
                         "sekli uygulanacak.\n\n" +
                         "Toplam sure yaklasik 5 dakika. Kalip alanini bosaltin.",
                         "Home Yap ve Basla", "Iptal", Theme.warn)
                else
                    show("Uretimi baslat",
                         "Tum moduller referans almis durumda; dogrudan model sekli " +
                         "uygulanacak (~150 sn).\n\nKalip alanini bosaltin.",
                         "Basla", "Iptal", Theme.actionStart)
            }
            else if (what === "home")
                show("Referans al",
                     "Tum bagli modullerde tum eksenler sifira inecek (~150 sn).",
                     "Home Yap", "Iptal", Theme.warn)
            else
                show("Secili module uygula",
                     "Modul " + app.currentSlaveId + " icin model sekli uygulanacak (~150 sn).\n" +
                     "Bu bir test islemidir; referans alinmamissa pozisyonlar hatali olabilir.",
                     "Uygula", "Iptal", Theme.actionNeutral)
        }
        onAccepted: {
            if (action === "production") app.startProduction()
            else if (action === "home") app.startHomeAll()
            else app.startSendSelected(app.currentSlaveId)
        }
    }

    // Yerlesim (ISA-101 H2): icerik ustte, AKSIYON CUBUGU en altta tam genislikte
    // ve her ekranda ayni yerde (kas hafizasi). Boylece kare harita da buyuyebilir.
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 14

    RowLayout {
        Layout.fillWidth: true
        Layout.fillHeight: true
        spacing: 16

        // ---------- SOL: STL + 3D + terminal ----------
        // Sabit genislikli kontrol sutunu. NOT: RowLayout icinde ic-ice
        // ColumnLayout sadece preferredWidth ile genisligini tutmaz (fillWidth
        // kardesini sifira sikistirir); min=max ile sabitlenir.
        ColumnLayout {
            Layout.fillWidth: true
            Layout.minimumWidth: 520
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
                        Text { text: "STL MODELI"; color: Theme.textDim; font.bold: true
                            font.pixelSize: Theme.fsSmall; font.letterSpacing: 1 }
                        Text {
                            text: page.stlName().length ? page.stlName() : "Dosya secilmedi"
                            color: page.stlName().length ? Theme.text : Theme.textFaint
                            font.pixelSize: Theme.fsBody; 
                            elide: Text.ElideMiddle; width: 250
                        }
                    }
                    // model istatistikleri (sayilar monospace - sutun hizasi)
                    Repeater {
                        model: [
                            { k: "MIN", v: app.stats && app.stats.min  !== undefined ? app.stats.min  : null },
                            { k: "MAX", v: app.stats && app.stats.max  !== undefined ? app.stats.max  : null },
                            { k: "ORT", v: app.stats && app.stats.mean !== undefined ? app.stats.mean : null }
                        ]
                        delegate: Column {
                            Layout.rightMargin: 20
                            Text { text: modelData.k; color: Theme.textFaint
                                font.pixelSize: Theme.fsSmall; font.letterSpacing: 1 }
                            Text { text: modelData.v !== null ? (modelData.v + " mm") : "--"
                                color: Theme.text; font.bold: true
                                font.pixelSize: Theme.fsValue
                                font.family: Theme.fontMono }
                        }
                    }
                    TouchButton {
                        text: "STL Sec"
                        color: Theme.actionPrimary
                        enabled: !app.busy
                        onClicked: fileDialog.open()
                    }
                }
            }

            // 3D onizleme + terminal YAN YANA (genis alani verimli kullanir)
            RowLayout {
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
                        anchors.margins: 10
                        spacing: 6
                        Text { text: "3D ONIZLEME"; color: Theme.textDim; font.bold: true
                            font.pixelSize: Theme.fsSmall; font.letterSpacing: 1 }
                        Surface3D {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                        }
                    }
                }

                // terminal (Seviye-4 bilgi: en dusuk oncelik, sabit dar sutun)
                Terminal {
                    Layout.preferredWidth: 440
                    Layout.minimumWidth: 440
                    Layout.maximumWidth: 440
                    Layout.fillHeight: true
                    logModel: page.logModel
                }
            }
        }

        // ---------- SAG: pozisyon haritasi (kare -> genislik yukseklige gore) ----------
        ColumnLayout {
            Layout.preferredWidth: 600
            Layout.minimumWidth: 600
            Layout.maximumWidth: 600
            Layout.fillHeight: true
            spacing: 14

            // heatmap + istatistikler (ayri serit yerine ayni panelde - yer kazanci)
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: Theme.radius
                color: Theme.panel
                border.color: Theme.border
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 10

                    Text {
                        Layout.fillWidth: true
                        text: "POZISYON HARITASI  ·  12 × 12  ·  144 MOTOR"
                        color: Theme.textDim; font.bold: true
                        font.pixelSize: Theme.fsSmall; font.letterSpacing: 1
                    }

                    // Harita KARE (12x12) + yaninda renk olcegi (legend)
                    Item {
                        id: mapArea
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Row {
                            anchors.centerIn: parent
                            spacing: Theme.gapMin
                            Heatmap {
                                id: hm
                                width: Math.max(120, Math.min(mapArea.width - 84, mapArea.height))
                                height: width
                            }
                            // renk olcegi: renk tek basina sayi okutmaz -> etiketli rampa
                            Item {
                                width: 68
                                height: hm.height
                                Rectangle {
                                    id: ramp
                                    width: 26; height: parent.height
                                    anchors.left: parent.left
                                    border.color: Theme.border; border.width: 1
                                    gradient: Gradient {
                                        GradientStop { position: 0.0;  color: Theme.heat(600, 600) }
                                        GradientStop { position: 0.25; color: Theme.heat(450, 600) }
                                        GradientStop { position: 0.5;  color: Theme.heat(300, 600) }
                                        GradientStop { position: 0.75; color: Theme.heat(150, 600) }
                                        GradientStop { position: 1.0;  color: Theme.heat(0, 600) }
                                    }
                                }
                                Repeater {
                                    model: [600, 450, 300, 150, 0]
                                    delegate: Text {
                                        x: ramp.width + 6
                                        y: (index / 4) * (ramp.height - font.pixelSize)
                                        text: modelData
                                        color: Theme.textDim
                                        font.pixelSize: Theme.fsSmall
                                        font.family: Theme.fontMono
                                    }
                                }
                            }
                        }
                    }
                }
            }

        }
    }

        // ---------- AKSIYON CUBUGU (en altta, tam genislik, sabit yer) ----------
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: Theme.inputHeight + Theme.btnHeightXl + 3 * Theme.gapMin + 12
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
                            enabled: !app.busy
                        }
                        Text {
                            text: "Bagli: [" + app.activeSlaves.join(", ") + "]"
                            color: Theme.textDim; font.pixelSize: Theme.fsSmall
                            font.family: Theme.fontMono
                        }
                    }

                    // Test: yalnizca secili module gonder (home zorunlu degil)
                    ActionButton {
                        text: "Secili Module Uygula"; subText: "test"
                        color: Theme.actionNeutral
                        enabled: app.connected && app.gridData.length > 0 && !app.busy
                        disabledReason: !app.connected ? "baglanti yok"
                                      : app.gridData.length === 0 ? "STL yuklu degil" : "islem suruyor"
                        onClicked: confirmDialog.ask("send")
                    }
                    // Sadece referans alma
                    ActionButton {
                        text: "Home"; subText: "tum moduller"
                        color: Theme.actionWarn
                        enabled: app.connected && !app.busy
                        disabledReason: !app.connected ? "baglanti yok" : "islem suruyor"
                        onClicked: confirmDialog.ask("home")
                    }
                    // Ana uretim akisi. IEC 60204-1: start AKTUATORU yesil olabilir
                    // (Theme'deki "yesili gostergede kullanma" kurali butona uygulanmaz).
                    // Tehlikeli hareketi baslatan tek kontrol -> en buyuk hedef (~25 mm).
                    ActionButton {
                        text: "BASLA"; subText: "referans + sekil uygula"
                        color: Theme.actionStart
                        Layout.columnSpan: 2
                        Layout.rowSpan: 1
                        implicitHeight: Theme.btnHeightXl
                        enabled: app.connected && app.gridData.length > 0 && !app.busy
                        disabledReason: !app.connected ? "baglanti yok"
                                      : app.gridData.length === 0 ? "once STL yukleyin" : "islem suruyor"
                        onClicked: confirmDialog.ask("production")
                    }
                }
        }
    }
}
