import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtQuick.Dialogs
import "Theme.js" as Theme

Item {
    id: page
    property var logModel

    function modList(a) { return (a && a.length) ? a.join(", ") : "—" }

    // Native QML dosya secici (kullanici tercihi)
    FileDialog {
        id: fileDialog
        title: "STL dosyası seç"
        nameFilters: ["STL dosyaları (*.stl)"]
        onAccepted: app.loadStl(selectedFile)
    }

    // Hareket baslatmadan once onay (fiziksel makine hareket edecek).
    // Onay metni: model adi, hedef modul listesi, motor sayisi (v2.2). "Vazgeç" yalniz
    // pencereyi kapatir; hicbir hareketi iptal etmez (bu arayuzden hareket durdurulamaz).
    AppDialog {
        id: confirmDialog
        property string action: ""
        function ask(what) {
            confirmDialog.action = what
            var p = app.productionPlan
            var mods = "Modüller: " + page.modList(p.modules) + " (" + p.motors + " motor)"
            var exp = p.expectedConfigured
                      ? ("Beklenen modüller (machine.json): " + page.modList(p.expected) + ".")
                      : ("Beklenen modül listesi YAPILANDIRILMADI: yalnız taramada bulunan modüller " +
                         "hareket eder. Takılı bütün modüllerin listede olduğunu kontrol edin.")
            var stop = "\nBu arayüzden hareket durdurulamıyor. Kalıp alanını boşaltın."
            if (what === "production") {
                if (p.needHome && p.needHome.length > 0)
                    show("Üretimi başlat: önce referans",
                         "Model: " + p.model + "\n" + mods + "\n" + exp + "\n\n" +
                         "Referansı olmayan modül var (" + page.modList(p.needHome) + "). Önce bütün " +
                         "listelenen modüllerde referans alınacak, ardından model şekli uygulanacak " +
                         "(toplam ~5 dk'ya kadar)." + stop,
                         "Referans al ve başla", "Vazgeç", Theme.warn)
                else
                    show("Üretimi başlat",
                         "Model: " + p.model + "\n" + mods + "\n" + exp + "\n\n" +
                         "Bütün modüllerde referans alma tamamlandı (firmware bildirimi); doğrudan " +
                         "model şekli uygulanacak (~150 sn'ye kadar)." + stop,
                         "Başla", "Vazgeç", Theme.actionStart)
            }
            else if (what === "home")
                show("Referans al",
                     mods + "\nBütün motorlar geri çekilip referans alacak (~150 sn'ye kadar)." + stop,
                     "Referans al", "Vazgeç", Theme.warn)
            else
                show("Seçili modüle uygula",
                     "Model: " + p.model + "\nModül: " + app.currentSlaveId + " (9 motor)\n" +
                     "Bu bir test işlemidir; yalnız bu modülün hedefleri uygulanır (~150 sn'ye kadar)." +
                     stop,
                     "Uygula", "Vazgeç", Theme.actionNeutral)
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
        // NOT: RowLayout icinde ic-ice ColumnLayout sadece preferredWidth ile
        // genisligini tutmaz (fillWidth kardesini sifira sikistirir); min=max ile sabitlenir.
        ColumnLayout {
            Layout.fillWidth: true
            Layout.minimumWidth: 520
            Layout.fillHeight: true
            spacing: 14

            // dosya + model durumu (yukleniyor / hazir / hata) + donusum bilgisi
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: Theme.btnHeight + 28 + (infoLine.visible ? infoLine.implicitHeight + 6 : 0)
                radius: Theme.radius
                color: Theme.panel
                border.color: app.modelState === "error" ? Theme.warn : Theme.border
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 6
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: Theme.gapMin
                        Column {
                            Layout.fillWidth: true
                            spacing: 4
                            Text { text: "STL MODELİ"; color: Theme.textDim; font.bold: true
                                font.pixelSize: Theme.fsSmall; font.letterSpacing: 1 }
                            Text {
                                width: parent.width
                                text: app.modelState === "loading" ? ("Hesaplanıyor: " + app.loadingName + " …")
                                    : app.modelState === "error" ? ("Yüklenemedi: " + app.loadingName)
                                    : app.modelState === "ready" ? app.modelName
                                    : "Dosya seçilmedi"
                                color: app.modelState === "ready" ? Theme.text
                                     : app.modelState === "idle" ? Theme.textFaint : Theme.warn
                                font.pixelSize: Theme.fsBody
                                font.bold: app.modelState === "error"
                                elide: Text.ElideMiddle
                            }
                        }
                        // model istatistikleri (sayilar monospace - sutun hizasi)
                        Repeater {
                            model: [
                                { k: "MİN", v: app.stats && app.stats.min  !== undefined ? app.stats.min  : null },
                                { k: "MAKS", v: app.stats && app.stats.max  !== undefined ? app.stats.max  : null },
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
                            text: "STL Seç"
                            color: Theme.actionPrimary
                            enabled: app.modelBlockReason === ""
                            disabledReason: app.modelBlockReason
                            onClicked: fileDialog.open()
                        }
                    }
                    // donusum seffafligi (3b): algoritma degismedi, yaptigi gosterilir
                    Text {
                        id: infoLine
                        Layout.fillWidth: true
                        visible: app.modelState === "ready" || app.modelState === "error"
                        wrapMode: Text.WordWrap
                        maximumLineCount: 2
                        elide: Text.ElideRight
                        font.pixelSize: Theme.fsSmall
                        readonly property var mi: app.modelInfo
                        readonly property int clipped: (mi.clipped_low || 0) + (mi.clipped_high || 0)
                        color: app.modelState === "error" || (mi.filled > 0 || clipped > 0) ? Theme.warn : Theme.textDim
                        text: app.modelState === "error"
                              ? ("Dosya STL olarak okunamadı veya hesaplanamadı; yüklü model yok. " + app.modelError)
                              : (((mi.filled > 0 || clipped > 0) ? "⚠ " : "") +
                                 "Model dışında kalıp en yüksek değerle doldurulan: " + (mi.filled || 0) +
                                 " hücre · " + app.safeMinMm + "–" + app.safeMaxMm + " mm sınırına kırpılan: " +
                                 clipped + " hücre · en büyük komşu farkı: " + (mi.max_neighbor || 0) +
                                 " mm (membran sınırı tanımlı değil)")
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
                        Text { text: "3D ÖNİZLEME — model hedefleri (ölçülen konum değildir)"
                            Layout.fillWidth: true
                            elide: Text.ElideRight
                            color: Theme.textDim; font.bold: true
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

        // ---------- SAG: model hedef haritasi (kare -> genislik yukseklige gore) ----------
        ColumnLayout {
            Layout.preferredWidth: 600
            Layout.minimumWidth: 600
            Layout.maximumWidth: 600
            Layout.fillHeight: true
            spacing: 14

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
                        text: "MODEL HEDEFLERİ — ölçülen konum değildir  ·  12 × 12"
                        color: Theme.textDim; font.bold: true
                        font.pixelSize: Theme.fsSmall; font.letterSpacing: 1
                        elide: Text.ElideRight
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
                        Text { text: "Hedef modül:"; color: Theme.textDim; font.pixelSize: Theme.fsBody }
                        SpinBox {
                            from: 1; to: 16
                            value: app.currentSlaveId
                            onValueModified: app.currentSlaveId = value
                            Layout.preferredHeight: Theme.inputHeight
                            font.pixelSize: Theme.fsButton
                            enabled: !app.busy
                        }
                        Text {
                            text: "Bulunan modüller: " + page.modList(app.activeSlaves) +
                                  (app.expectedModules.length
                                   ? ("   ·   Beklenen: " + page.modList(app.expectedModules))
                                   : "   ·   Beklenen liste yapılandırılmadı")
                            color: Theme.textDim; font.pixelSize: Theme.fsSmall
                            font.family: Theme.fontMono
                        }
                    }

                    // Test: yalnizca secili module gonder (referans gerekli)
                    ActionButton {
                        text: "Seçili modüle uygula"; subText: "test · referans gerekli"
                        color: Theme.actionNeutral
                        enabled: app.sendSelectedBlockReason === ""
                        disabledReason: app.sendSelectedBlockReason
                        onClicked: confirmDialog.ask("send")
                    }
                    // Sadece referans alma
                    ActionButton {
                        text: "Referans al"; subText: "bulunan bütün modüller"
                        color: Theme.actionWarn
                        enabled: app.homeAllBlockReason === ""
                        disabledReason: app.homeAllBlockReason
                        onClicked: confirmDialog.ask("home")
                    }
                    // Ana uretim akisi. IEC 60204-1: start AKTUATORU yesil olabilir
                    // (Theme'deki "yesili gostergede kullanma" kurali butona uygulanmaz).
                    // Tehlikeli hareketi baslatan tek kontrol -> en buyuk hedef (~25 mm).
                    ActionButton {
                        text: "BAŞLA"; subText: "referans + şekil uygula"
                        color: Theme.actionStart
                        Layout.columnSpan: 2
                        Layout.rowSpan: 1
                        implicitHeight: Theme.btnHeightXl
                        enabled: app.productionBlockReason === ""
                        disabledReason: app.productionBlockReason
                        onClicked: confirmDialog.ask("production")
                    }
                }
        }
    }
}
