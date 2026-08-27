import QtQuick
import "Theme.js" as Theme

Item {
    id: root
    property int gridN: 12
    property int selR: -1        // secili hucre (dokunulan) - kalici
    property int selC: -1

    readonly property real cw: width / gridN
    readonly property real ch: height / gridN

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
            // modul (3x3) sinirlari
            ctx.strokeStyle = "rgba(255,255,255,0.45)"
            ctx.lineWidth = 2
            for (var k = 0; k <= 4; k++) {
                ctx.beginPath(); ctx.moveTo(k * 3 * cw, 0); ctx.lineTo(k * 3 * cw, height); ctx.stroke()
                ctx.beginPath(); ctx.moveTo(0, k * 3 * ch); ctx.lineTo(width, k * 3 * ch); ctx.stroke()
            }

            // modul numarasi: her 3x3 blogun ORTA hucresine (hizli teshis icin)
            // orta hucre = blok icindeki (1,1) -> merkez (mc*3+1.5)*cw
            ctx.textAlign = "center"
            ctx.textBaseline = "middle"
            ctx.font = "bold " + Math.max(11, Math.round(cw * 0.40)) + "px sans-serif"
            ctx.lineWidth = 3
            ctx.strokeStyle = "rgba(0,0,0,0.75)"       // kontrast icin koyu kontur
            ctx.fillStyle = "rgba(255,255,255,0.95)"
            for (var mr = 0; mr < 4; mr++) {
                for (var mc = 0; mc < 4; mc++) {
                    var sid = mr * 4 + mc + 1
                    var tx = (mc * 3 + 1.5) * cw
                    var ty = (mr * 3 + 1.5) * ch
                    ctx.strokeText(sid, tx, ty)
                    ctx.fillText(sid, tx, ty)
                }
            }
        }
    }

    Connections {
        target: app
        function onGridChanged() { canvas.requestPaint() }
    }
    onWidthChanged: canvas.requestPaint()
    onHeightChanged: canvas.requestPaint()

    // ---- secili hucre isareti (overlay - canvas repaint yok) ----
    Rectangle {
        visible: root.selR >= 0 && root.selC >= 0
        x: root.selC * root.cw
        y: root.selR * root.ch
        width: root.cw
        height: root.ch
        color: "transparent"
        border.color: "white"
        border.width: 3
        radius: 2
    }

    // ---- dokunma: hucre sec (hover yok, parmakla tap) ----
    TapHandler {
        onTapped: function(ep) {
            var n = root.gridN
            var c = Math.floor(ep.position.x / (width / n))
            var r = Math.floor(ep.position.y / (height / n))
            if (c < 0 || c >= n || r < 0 || r >= n) return
            if (app.gridData.length <= r * n + c) return
            root.selR = r
            root.selC = c
        }
    }

    // ---- bilgi etiketi: dokunulan hucrenin USTUNDE (parmak alttadir) ----
    Rectangle {
        id: tip
        visible: root.selR >= 0 && root.selC >= 0 && app.gridData.length > 0
        color: "#e6000000"   // QML 8-hane hex = #AARRGGBB; %90 siyah
        border.color: Theme.selection
        border.width: 2
        radius: Theme.radiusSm
        width: tipText.implicitWidth + 20
        height: tipText.implicitHeight + 14

        // yatayda dokunulan sutunla ortali, ekran icine clamp
        x: {
            if (root.selC < 0) return 0
            var cx = root.selC * root.cw + root.cw / 2 - width / 2
            return Math.max(2, Math.min(cx, root.width - width - 2))
        }
        // hucrenin ustunde; en ust satirda alta cevir (flip)
        y: {
            if (root.selR < 0) return 0
            var above = root.selR * root.ch - height - 8
            if (above >= 2) return above
            return root.selR * root.ch + root.ch + 8   // flip: hucrenin altina
        }

        Text {
            id: tipText
            anchors.centerIn: parent
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: Theme.fsTip
            font.family: "monospace"
            color: "white"
            text: {
                if (root.selR < 0 || root.selC < 0) return ""
                var idx = root.selR * root.gridN + root.selC
                var slaveRow = Math.floor(root.selR / 3), slaveCol = Math.floor(root.selC / 3)
                var sid = slaveRow * 4 + slaveCol + 1
                var mm = app.gridData.length > idx ? app.gridData[idx] + " mm" : "—"
                return "Satir " + root.selR + " · Sutun " + root.selC + "  ·  Modul " + sid + "\n" + mm
            }
        }
    }
}
