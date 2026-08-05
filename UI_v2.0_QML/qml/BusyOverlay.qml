import QtQuick
import QtQuick.Layouts
import "Theme.js" as Theme

// Hareket sururken tum ekrani kaplayan ilerleme katmani.
// Kazara dokunmayi engeller (motorlar hareket halinde) ve fazi gosterir.
// NOT: protokol v5.1'de STOP/ABORT komutu YOK -> iptal butonu konulamaz;
// acil durum fiziksel guc kesme ile yapilir.
Item {
    id: root
    anchors.fill: parent
    visible: app.busy
    z: 1500

    property string phase: ""
    property int done: 0
    property int total: 0

    Connections {
        target: app
        function onOpProgress(label, done, total) {
            root.phase = label
            root.done = done
            root.total = total
        }
    }

    Rectangle {
        anchors.fill: parent
        color: "#d9000000"                 // #AARRGGBB
        MouseArea { anchors.fill: parent }  // dokunuslari yut
    }

    ColumnLayout {
        anchors.centerIn: parent
        width: Math.min(parent.width * 0.5, 620)
        spacing: 20

        Text {
            Layout.fillWidth: true
            text: root.phase.length ? root.phase : "Islem suruyor"
            color: Theme.text
            font.bold: true
            font.pixelSize: Theme.fsTitle
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
        }

        Text {
            Layout.fillWidth: true
            text: root.total > 0 ? ("Tamamlanan modul: " + root.done + " / " + root.total) : ""
            color: Theme.accent2
            font.pixelSize: Theme.fsButtonLg
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
        }

        // ilerleme cubugu
        Rectangle {
            Layout.fillWidth: true
            height: 14
            radius: 7
            color: Theme.panelHi
            Rectangle {
                height: parent.height
                radius: parent.radius
                color: Theme.green
                width: root.total > 0 ? parent.width * (root.done / root.total) : 0
                Behavior on width { NumberAnimation { duration: 220 } }
            }
        }

        Text {
            Layout.fillWidth: true
            text: "Hareket yavastir (tam strok ~150 sn). Lutfen bekleyin.\nDurdurmak icin cihazin gucunu kesin."
            color: Theme.textDim
            font.pixelSize: Theme.fsSmall
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
            lineHeight: 1.3
        }
    }
}
