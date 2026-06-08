import QtQuick
import "Theme.js" as Theme

Item {
    id: root
    property int gridN: 12

    Canvas {
        id: canvas
        anchors.fill: parent
        onPaint: {
            var ctx = getContext('2d')
            ctx.reset()
            var n = root.gridN
            var cw = width / n
            var ch = height / n
            var g = app.gridData

            for (var r = 0; r < n; r++) {
                for (var c = 0; c < n; c++) {
                    var idx = r * n + c
                    if (g.length > idx) {
                        ctx.fillStyle = Theme.heat(g[idx], 600)
                    } else {
                        ctx.fillStyle = Theme.panelHi
                    }
                    ctx.fillRect(c * cw, r * ch, cw - 1, ch - 1)
                }
            }
            // ince grid
            ctx.strokeStyle = "rgba(255,255,255,0.10)"
            ctx.lineWidth = 1
            for (var i = 0; i <= n; i++) {
                ctx.beginPath(); ctx.moveTo(i * cw, 0); ctx.lineTo(i * cw, height); ctx.stroke()
                ctx.beginPath(); ctx.moveTo(0, i * ch); ctx.lineTo(width, i * ch); ctx.stroke()
            }
            // slave sinirlari (3x3 modul)
            ctx.strokeStyle = "rgba(255,255,255,0.45)"
            ctx.lineWidth = 2
            for (var k = 0; k <= 4; k++) {
                ctx.beginPath(); ctx.moveTo(k * 3 * cw, 0); ctx.lineTo(k * 3 * cw, height); ctx.stroke()
                ctx.beginPath(); ctx.moveTo(0, k * 3 * ch); ctx.lineTo(width, k * 3 * ch); ctx.stroke()
            }
        }
    }

    Connections {
        target: app
        function onGridChanged() { canvas.requestPaint() }
    }
    onWidthChanged: canvas.requestPaint()
    onHeightChanged: canvas.requestPaint()

    // hover bilgisi
    MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        onPositionChanged: {
            var n = root.gridN
            var c = Math.floor(mouseX / (width / n))
            var r = Math.floor(mouseY / (height / n))
            if (c < 0 || c >= n || r < 0 || r >= n) { tip.visible = false; return }
            var idx = r * n + c
            var g = app.gridData
            var slaveRow = Math.floor(r / 3), slaveCol = Math.floor(c / 3)
            var sid = slaveRow * 4 + slaveCol + 1
            tip.text = "Satir " + r + ", Sutun " + c + "  ·  Slave " + sid +
                       "\n" + (g.length > idx ? g[idx] + " mm" : "—")
            tip.x = Math.min(mouseX + 12, width - tip.width - 4)
            tip.y = Math.min(mouseY + 12, height - tip.height - 4)
            tip.visible = g.length > idx
        }
        onExited: tip.visible = false
    }

    Rectangle {
        id: tip
        property alias text: tipText.text
        visible: false
        width: tipText.width + 16
        height: tipText.height + 12
        color: "#000000cc"
        border.color: Theme.accent
        radius: 6
        Text {
            id: tipText
            anchors.centerIn: parent
            color: "white"; font.pixelSize: 12; font.family: "monospace"
        }
    }
}
