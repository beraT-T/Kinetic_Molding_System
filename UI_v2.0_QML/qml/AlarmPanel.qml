import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// Alarm / uyari listesi (modul/motor bazli, kalici).
// Yasam dongusu (backend): AKTIF (gorulmedi) -> "Uyarıyı gördüm" -> AKTIF (goruldu)
//   -> taze gecerli STAT normale donusu dogrularsa -> GIDERILDI -> (goruldu ise) listeden kalkar.
// Iletisim belirsizliginde alarm cozuldu SAYILMAZ.
Item {
    id: root
    anchors.fill: parent
    visible: false
    z: 1900

    function open() { root.visible = true }

    // NOT: QML 8-hane hex = #AARRGGBB (alpha once)
    Rectangle {
        anchors.fill: parent
        color: "#cc000000"
        MouseArea { anchors.fill: parent }   // arkadaki dokunuslari yut
    }

    Rectangle {
        anchors.centerIn: parent
        width: Math.min(parent.width * 0.8, 1200)
        height: Math.min(parent.height * 0.85, 820)
        radius: Theme.radius
        color: Theme.panel
        border.color: Theme.border
        border.width: 2

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 24
            spacing: 14

            Text {
                text: "Alarmlar ve uyarılar"
                color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle
            }
            Text {
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                color: Theme.textDim; font.pixelSize: Theme.fsBody
                text: "“Uyarıyı gördüm” kaydı silmez. Kayıt, modülden taze ve geçerli bir " +
                      "durum cevabı normale dönüşü gösterince “giderildi” olur; görüldükten " +
                      "sonra listeden kalkar. Haberleşme belirsizken kayıt giderilmiş sayılmaz."
            }

            ListView {
                id: list
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                spacing: 8
                model: app.alarms
                ScrollBar.vertical: ScrollBar { width: 16 }
                delegate: Rectangle {
                    required property var modelData
                    width: list.width - 20
                    height: Math.max(Theme.touchDense, row.implicitHeight + 16)
                    radius: Theme.radiusSm
                    color: Theme.panelHi
                    border.color: modelData.active
                                  ? (modelData.level >= 2 ? Theme.alarm : Theme.warn)
                                  : Theme.border
                    border.width: modelData.active ? 3 : 1
                    RowLayout {
                        id: row
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: Theme.gapMin
                        Rectangle {
                            Layout.preferredWidth: 120
                            Layout.preferredHeight: 40
                            radius: Theme.radiusSm
                            color: !modelData.active ? Theme.panel
                                 : (modelData.level >= 2 ? Theme.alarm : Theme.warn)
                            border.color: Theme.border
                            border.width: modelData.active ? 0 : 1
                            Text {
                                anchors.centerIn: parent
                                text: modelData.level >= 2 ? "ARIZA" : "UYARI"
                                color: modelData.active ? "#101214" : Theme.textDim
                                font.bold: true; font.pixelSize: Theme.fsSmall
                            }
                        }
                        Text {
                            Layout.fillWidth: true
                            text: modelData.text
                            wrapMode: Text.WordWrap
                            color: Theme.text; font.pixelSize: Theme.fsBody
                        }
                        Column {
                            Layout.preferredWidth: 260
                            Text {
                                text: "Başlangıç " + modelData.since
                                color: Theme.textDim; font.pixelSize: Theme.fsSmall
                                font.family: Theme.fontMono
                            }
                            Text {
                                text: modelData.active
                                      ? (modelData.acked ? "AKTİF · görüldü" : "AKTİF · görülmedi")
                                      : ("Giderildi " + modelData.resolvedAt +
                                         (modelData.acked ? "" : " · görülmedi"))
                                color: modelData.active ? Theme.text : Theme.textDim
                                font.bold: modelData.active
                                font.pixelSize: Theme.fsSmall
                            }
                        }
                    }
                }
            }
            Text {
                visible: app.alarms.length === 0
                text: "Kayıt yok."
                color: Theme.textDim; font.pixelSize: Theme.fsBody
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gapMin
                Item { Layout.fillWidth: true }
                TouchButton {
                    text: "Uyarıyı gördüm"
                    color: Theme.actionNeutral
                    enabled: app.unackedAlarms > 0
                    disabledReason: "görülmemiş kayıt yok"
                    onClicked: app.acknowledgeAlarms()
                }
                TouchButton {
                    text: "Kapat"
                    color: Theme.actionPrimary
                    onClicked: root.visible = false
                }
            }
        }
    }
}
