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
    // Herhangi bir hareket komutu yolda/kabul edilmisken veya hareket suruyor/
    // durgunlugu dogrulanmamisken gorunur (Bakim hareketleri dahil).
    visible: app.busy || app.motionActive
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
        border.color: Theme.border
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
                    text: app.busy ? (root.phase.length ? root.phase : "İşlem sürüyor")
                                   : "Hareket izleniyor (işlem sonuçlandı)"
                    color: Theme.text; font.bold: true
                    font.pixelSize: Theme.fsTitle
                }
                Item { Layout.fillWidth: true }
                Text {
                    visible: app.busy
                    text: root.done + " / " + root.total + " motor"
                    color: Theme.text; font.bold: true
                    font.pixelSize: Theme.fsValue
                    font.family: Theme.fontMono
                }
            }

            // islem bittiyse (ariza/zaman asimi) ilerleme yerine gozetim sebebi
            Text {
                visible: !app.busy
                Layout.fillWidth: true
                text: app.motionText.length ? app.motionText
                                            : "Hareket sürüyor veya durgunluk doğrulanmadı"
                color: Theme.textDim; font.pixelSize: Theme.fsBody
                elide: Text.ElideRight
            }

            // ilerleme cubugu (notr - yesil "normal" anlamina saklanir)
            Rectangle {
                visible: app.busy
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
                visible: app.busy
                Layout.fillWidth: true
                Text {
                    text: "Geçen " + Theme.clock(root.elapsed)
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

            // DURDURMA SINIRI (v2.2): protokolde host STOP/ABORT yok, fiziksel E-stop
            // dogrulanmadi -> yalniz DOGRU olan soylenir; talimat/anahtar adi UYDURULMAZ.
            // Normal hareket = notr zemin (alarm kirmizisi yalniz gercek ariza icin).
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: stopRow.implicitHeight + 16
                radius: Theme.radiusSm
                color: Theme.panelHi
                border.color: Theme.warn
                border.width: 2
                RowLayout {
                    id: stopRow
                    anchors.centerIn: parent
                    spacing: Theme.gapMin
                    Text { text: "⚠"; color: Theme.warn; font.pixelSize: Theme.fsButtonLg }
                    Text {
                        text: "Bu arayüzden hareket durdurulamıyor."
                        color: Theme.text; font.bold: true
                        font.pixelSize: Theme.fsButtonLg
                    }
                }
            }
        }
    }
}
