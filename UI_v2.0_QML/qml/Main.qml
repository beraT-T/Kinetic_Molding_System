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

        // kalici durum/alarm seridi - her sekmede gorunur (ISA-101)
        StatusStrip { Layout.fillWidth: true }

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
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: 0
            MainPage   { logModel: logModel }
            TesterPage { logModel: logModel }
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
        height: 230
    }

    // ---- bilgi/hata penceresi (islem sonucu) ----
    AppDialog { id: infoDialog }

    // Islem bitince sonucu goster (basarili / ariza / zaman asimi)
    // NOT: referans uyarisi baglantida DEGIL, BASLA'ya basinca cikar (MainPage).
    Connections {
        target: app
        function onOperationFinished(ok, title, message) {
            infoDialog.show(title, message, "Tamam", "",
                            ok ? Theme.actionStart : Theme.alarm)
        }
    }
}
