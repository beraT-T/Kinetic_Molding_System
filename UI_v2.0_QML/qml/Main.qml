import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

ApplicationWindow {
    id: win
    visible: true
    width: 1366
    height: 820
    title: "Adaptif Kalıp Kontrol - UI v2.2" + (app.demoMode ? " — SİMÜLASYON" : "")
    color: Theme.bg

    Component.onCompleted: if (startFullscreen) win.showFullScreen()

    // ---- ortak log modeli ----
    ListModel { id: logModel }

    Connections {
        target: app
        function onLogMessage(msg) {
            logModel.append({ "line": msg })
            if (logModel.count > 800) logModel.remove(0, logModel.count - 800)
            // otomatik en alta kaydirma terminallerde yapilir
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        Header { Layout.fillWidth: true }

        // --demo: KALICI SIMULASYON bandi (sahte basari gercek islemle karismasin).
        // Renk hicbir durum/alarm anlamina kullanilmayan sonuk mor; metin acikca soyler.
        Rectangle {
            Layout.fillWidth: true
            visible: app.demoMode
            height: 44
            color: "#3b2f63"
            border.color: "#8a7bc8"
            border.width: 2
            Text {
                anchors.centerIn: parent
                text: "SİMÜLASYON (--demo) — donanım bağlı değil; durumlar ve hareketler sahte köprüden"
                color: "#ffffff"
                font.bold: true
                font.pixelSize: Theme.fsBody
                font.letterSpacing: 1
            }
        }

        // kalici durum/alarm seridi - her sekmede gorunur (ISA-101)
        StatusStrip { Layout.fillWidth: true; onOpenAlarms: alarmPanel.open() }

        // ---- sekme cubugu ----
        Rectangle {
            Layout.fillWidth: true
            height: Theme.touchMin + 8
            color: Theme.bg2
            RowLayout {
                anchors.left: parent.left
                anchors.leftMargin: 16
                anchors.verticalCenter: parent.verticalCenter
                spacing: Theme.gapMin
                Repeater {
                    model: ["STL Kontrol", "Bakım / Motor testi"]
                    delegate: Button {
                        objectName: "tab_" + index
                        text: modelData
                        checkable: true
                        checked: tabStack.currentIndex === index
                        onClicked: tabStack.currentIndex = index
                        implicitHeight: Theme.touchMin
                        background: Rectangle {
                            radius: Theme.radiusSm
                            color: parent.checked ? Theme.selection : "transparent"
                            border.color: parent.checked ? Theme.selection : Theme.border
                            border.width: 1
                        }
                        contentItem: Text {
                            text: parent.text
                            color: parent.checked ? "white" : Theme.textDim
                            font.bold: true
                            font.pixelSize: Theme.fsButton
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 22; rightPadding: 22
                        }
                    }
                }
            }
        }

        StackLayout {
            id: tabStack
            objectName: "tabStack"
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: 0
            MainPage   { objectName: "mainPage"; logModel: logModel }
            TesterPage { objectName: "testerPage"; logModel: logModel }
        }

    }

    // Hareket ilerlemesi: ekrani KAPATMAZ. Yerlesimden yer CALMAZ; aksiyon
    // cubugunun uzerine biner (butonlar zaten islem sirasinda devre disi).
    // Isi haritasi ve durum seridi canli izlenebilir kalir.
    BusyOverlay {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.leftMargin: 16
        anchors.rightMargin: 16
        anchors.bottomMargin: 16
        height: implicitHeight
    }

    // ---- alarm / uyari listesi ----
    AlarmPanel { id: alarmPanel; objectName: "alarmPanel" }

    // ---- bilgi/hata penceresi (islem sonucu) ----
    // Tek pencere + kisa KUYRUK: ayni mesaj tekrar gelirse yeniden acilmaz (pencere
    // yagmuru yok); acik ARIZA penceresi sonraki siradan bir mesajla EZILMEZ; ariza
    // mesaji siradakilerin onune gecer. Kuyruk sinirli (en eski ariza-disi mesaj atilir).
    AppDialog {
        id: infoDialog
        objectName: "infoDialog"
        property var current: null
        onAccepted: win.showNextResult()
    }
    property var pendingResults: []

    function _sameResult(a, b) { return a && b && a.title === b.title && a.message === b.message }
    function _showResult(r) {
        infoDialog.current = r
        infoDialog.show(r.title, r.message, "Kapat", "", r.color)
    }
    function showNextResult() {
        infoDialog.current = null
        if (win.pendingResults.length > 0) {
            var q = win.pendingResults.slice()
            var r = q.shift()
            win.pendingResults = q
            win._showResult(r)
        }
    }
    function postResult(ok, title, message) {
        // "Kapat": pencere yalniz kapanir (islemi etkilemez). Basari rengi NOTR
        // (yesil yalniz BASLA aktuatorunde); ariza kirmizi; diger red/uyari amber.
        var r = { "title": title, "message": message, "fault": title === "ARIZA",
                  "color": ok ? Theme.actionPrimary
                              : (title === "ARIZA" ? Theme.alarm : Theme.warn) }
        if (!infoDialog.visible) { win._showResult(r); return }
        if (win._sameResult(infoDialog.current, r)) return
        var q = win.pendingResults.slice()
        for (var i = 0; i < q.length; i++) if (win._sameResult(q[i], r)) return
        if (r.fault && !(infoDialog.current && infoDialog.current.fault)) {
            if (infoDialog.current) q.unshift(infoDialog.current)
            win.pendingResults = q.slice(0, 6)
            win._showResult(r)
            return
        }
        if (r.fault) q.unshift(r); else q.push(r)
        while (q.length > 6) {
            var drop = -1
            for (var j = q.length - 1; j >= 0; j--) if (!q[j].fault) { drop = j; break }
            q.splice(drop >= 0 ? drop : q.length - 1, 1)
        }
        win.pendingResults = q
    }

    // Islem bitince sonucu goster (basarili / ariza / zaman asimi)
    // NOT: referans uyarisi baglantida DEGIL, BASLA'ya basinca cikar (MainPage).
    Connections {
        target: app
        function onOperationFinished(ok, title, message) { win.postResult(ok, title, message) }
    }
}
