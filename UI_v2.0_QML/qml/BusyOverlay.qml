import QtQuick
import QtQuick.Layouts
import "Theme.js" as Theme

// Hareket sirasindaki ilerleme paneli.
// TASARIM KARARI (ISA-101 / NN-g): hareket sururken proses durumu GORUNUR
// kalmalidir. Bu yuzden ekrani kaplayan opak modal KULLANILMAZ; panel altta
// serit olarak durur, isi haritasi ve durum seridi canli izlenebilir.
// Kazara dokunma, aksiyon butonlarinin devre disi birakilmasiyla onlenir.
Item {
    id: root
    visible: app.busy
    z: 1500

    property string phase: ""
    property int done: 0
    property int total: 0
    property int elapsed: 0
    property int remaining: -1

    Connections {
        target: app
        function onOpProgress(label, d, t, el, rem) {
            root.phase = label; root.done = d; root.total = t
            root.elapsed = el; root.remaining = rem
        }
    }

    Rectangle {
        anchors.fill: parent
        color: Theme.panel
        border.color: Theme.warn
        border.width: 2
        radius: Theme.radius

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 14
            spacing: 10

            // faz + sayilar
            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gapMin
                Text {
                    text: root.phase.length ? root.phase : "Islem suruyor"
                    color: Theme.text; font.bold: true
                    font.pixelSize: Theme.fsTitle
                }
                Item { Layout.fillWidth: true }
                Text {
                    text: root.done + " / " + root.total + " motor"
                    color: Theme.text; font.bold: true
                    font.pixelSize: Theme.fsValue
                    font.family: Theme.fontMono
                }
            }

            // ilerleme cubugu (notr - yesil "normal" anlamina saklanir)
            Rectangle {
                Layout.fillWidth: true
                height: 18
                radius: Theme.radiusSm
                color: Theme.panelHi
                border.color: Theme.border
                Rectangle {
                    height: parent.height
                    radius: parent.radius
                    color: Theme.text
                    width: root.total > 0 ? parent.width * (root.done / root.total) : 0
                    Behavior on width { NumberAnimation { duration: 300 } }
                }
            }

            // sureler (NN/g: 1 dk ustu islemde gecen + tahmini kalan)
            RowLayout {
                Layout.fillWidth: true
                Text {
                    text: "Gecen " + Theme.clock(root.elapsed)
                    color: Theme.textDim; font.pixelSize: Theme.fsBody
                    font.family: Theme.fontMono
                }
                Item { Layout.fillWidth: true }
                Text {
                    text: root.remaining >= 0 ? ("Tahmini kalan " + Theme.clock(root.remaining))
                                              : "Tahmini kalan --:--"
                    color: Theme.textDim; font.pixelSize: Theme.fsBody
                    font.family: Theme.fontMono
                }
            }

            // DURDURMA TALIMATI - makinenin tek durdurma yolu, en okunur metin olmali
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: stopRow.implicitHeight + 16
                radius: Theme.radiusSm
                color: Theme.alarm
                RowLayout {
                    id: stopRow
                    anchors.centerIn: parent
                    spacing: Theme.gapMin
                    Text { text: "⛔"; font.pixelSize: Theme.fsButtonLg }
                    Text {
                        text: "DURDURMAK ICIN CIHAZIN GUCUNU KESIN"
                        color: "#ffffff"; font.bold: true
                        font.pixelSize: Theme.fsButtonLg
                    }
                }
            }
        }
    }
}
