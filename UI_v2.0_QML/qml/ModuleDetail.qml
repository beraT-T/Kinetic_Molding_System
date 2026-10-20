import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import "Theme.js" as Theme

// Haritadan acilan MODUL AYRINTISI (v2.2, dokunmatik): harita hucreleri 80 px'in altinda
// oldugundan tek motor buradan, buyuk hucrelerle okunur. "Hedef" = yuklu MODEL hedefi;
// "Ölçülen" = modulun son durum sorgusu (yasi ile). Ikisi ayri gosterilir.
// Bu pencere hareket BASLATMAZ; yalniz "Hedef modül" secimini degistirebilir.
Item {
    id: root
    anchors.fill: parent
    visible: false
    z: 1900
    property int sid: 1
    readonly property var targets: { app.gridData; return app.moduleTargets(root.sid) }
    readonly property var info: { app.moduleRevision; return app.moduleInfo(root.sid) }

    function open(s) { root.sid = s; root.visible = true }

    Rectangle {
        anchors.fill: parent
        color: "#cc000000"
        MouseArea { anchors.fill: parent; onClicked: root.visible = false }
    }

    Rectangle {
        anchors.centerIn: parent
        width: 760
        height: col.implicitHeight + 48
        radius: Theme.radius
        color: Theme.panel
        border.color: Theme.border
        border.width: 2
        MouseArea { anchors.fill: parent }      // panel ici dokunus kapatmasin

        ColumnLayout {
            id: col
            anchors.centerIn: parent
            width: parent.width - 48
            spacing: 12

            Text {
                text: "Modül " + root.sid + " — model hedefleri ve son ölçüm"
                color: Theme.text; font.bold: true; font.pixelSize: Theme.fsTitle
            }
            Text {
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                color: root.info.current ? Theme.textDim : Theme.warn
                font.pixelSize: Theme.fsSmall
                text: (root.info.known
                       ? ((root.info.current ? "Son ölçüm " : "Son bilinen durum ") +
                          (root.info.ageS < 0 ? "yok" : root.info.ageS + " sn önce") +
                          (!root.info.current ? " — güncel değil" : ""))
                       : "Bu modülden durum alınmadı (ölçüm bilinmiyor).") +
                      "   ·   Hedefler yüklü modelden; ölçülen konum değildir."
            }

            GridLayout {
                Layout.fillWidth: true
                columns: 3
                rowSpacing: 10; columnSpacing: 10
                Repeater {
                    model: 9
                    delegate: Rectangle {
                        required property int index
                        readonly property var mi: root.info.motors[index]
                        Layout.fillWidth: true
                        Layout.preferredHeight: 104
                        radius: Theme.radiusSm
                        color: Theme.panelHi
                        border.color: mi.fault ? Theme.alarm : Theme.border
                        border.width: mi.fault ? 3 : 1
                        Column {
                            anchors.centerIn: parent
                            spacing: 2
                            Text {
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: "Motor " + (index + 1)
                                color: Theme.textDim; font.pixelSize: Theme.fsSmall
                            }
                            Text {
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: "Hedef " + (root.targets.length ? root.targets[index] + " mm" : "—")
                                color: Theme.text; font.bold: true
                                font.pixelSize: Theme.fsBody; font.family: Theme.fontMono
                            }
                            Text {
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: (root.info.current ? "Ölçülen " : "Son bilinen ") +
                                      (mi.known ? mi.mm + " mm" : "—") +
                                      (mi.known ? " · " + Theme.stateLabel(mi.letter) : "")
                                color: mi.fault ? Theme.alarm : Theme.textDim
                                font.pixelSize: Theme.fsSmall; font.family: Theme.fontMono
                            }
                        }
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: Theme.gapMin
                Item { Layout.fillWidth: true }
                TouchButton {
                    objectName: "detailMakeTarget"
                    text: "Hedef modül yap"
                    color: Theme.actionNeutral
                    enabled: !app.busy && app.currentSlaveId !== root.sid
                    disabledReason: app.busy ? "işlem sürüyor" : "zaten seçili"
                    onClicked: { app.currentSlaveId = root.sid; root.visible = false }
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
