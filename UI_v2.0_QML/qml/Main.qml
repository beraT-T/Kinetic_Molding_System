import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

ApplicationWindow {
    id: win
    visible: true
    width: 1366
    height: 820
    title: "Adaptif Kalip Kontrol - UI v2.0"
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

        // ---- sekme cubugu ----
        Rectangle {
            Layout.fillWidth: true
            height: Theme.touchMin + 16
            color: Theme.bg2
            RowLayout {
                anchors.left: parent.left
                anchors.leftMargin: 16
                anchors.verticalCenter: parent.verticalCenter
                spacing: Theme.gapMin
                Repeater {
                    model: ["STL Kontrol", "Tester"]
                    delegate: Button {
                        text: modelData
                        checkable: true
                        checked: tabStack.currentIndex === index
                        onClicked: tabStack.currentIndex = index
                        implicitHeight: Theme.touchMin
                        background: Rectangle {
                            radius: Theme.radiusSm
                            color: parent.checked ? Theme.accent : "transparent"
                            border.color: parent.checked ? Theme.accent : Theme.border
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
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: 0
            MainPage   { logModel: logModel }
            TesterPage { logModel: logModel }
        }
    }

    // ---- hareket sirasinda ilerleme katmani ----
    BusyOverlay { }

    // ---- bilgi/hata penceresi (islem sonucu) ----
    AppDialog { id: infoDialog }

    // ---- acilis uyarisi: referans alindi mi? ----
    AppDialog {
        id: startupDialog
        onAccepted: app.startHomeAll()
    }

    // Islem bitince sonucu goster (basarili / ariza / zaman asimi)
    Connections {
        target: app
        function onOperationFinished(ok, title, message) {
            infoDialog.show(title, message, "Tamam", "",
                            ok ? Theme.green : Theme.red)
        }
    }

    // Sistem baslatildiginda (baglanti + modul bulununca) referans uyarisi.
    // Her baglantida bir kez sorulur.
    property bool homeAsked: false
    Connections {
        target: app
        function onConnectedChanged() { if (!app.connected) win.homeAsked = false }
        function onActiveSlavesChanged() { win.maybeAskHome() }
        function onHomedChanged() { win.maybeAskHome() }
    }
    function maybeAskHome() {
        if (homeAsked || !app.connected || app.busy) return
        if (app.activeSlaves.length === 0 || app.allHomed) return
        homeAsked = true
        startupDialog.show("Referans alinmadi",
            "Sistem baslatildi ve eksenler henuz referans almadi.\n" +
            "Guvenli calisma icin once tum modullerde Home yapilmalidir.\n\n" +
            "Simdi referans alinsin mi? (yaklasik 150 sn)",
            "Home Yap", "Simdi Degil", Theme.orange)
    }
}
