import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import "Theme.js" as Theme

// NOT: Ham STL mesh gorunumu (RuntimeLoader/AssetUtils) ERTELENDI - sonraki tur.
// Bu bilesen simdilik yalniz interpolasyon sonucu kalip yuzeyini (144 pin) cizer.
// controller.stlFileUrl backend'de hazir; STL gorunumu tekrar eklenecegi zaman kullanilacak.
Item {
    id: root
    property real spacing: 30
    property real footprint: 22
    property real maxHeight: 150

    View3D {
        id: view
        anchors.fill: parent

        environment: SceneEnvironment {
            clearColor: Theme.bg2
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.Medium
        }

        Node { id: originNode }

        PerspectiveCamera {
            id: cam
            position: Qt.vector3d(0, 280, 360)
            eulerRotation.x: -32
        }

        OrbitCameraController { origin: originNode; camera: cam }

        DirectionalLight { eulerRotation: Qt.vector3d(-45, -30, 0); brightness: 1.0 }
        DirectionalLight { eulerRotation: Qt.vector3d(45, 140, 0); brightness: 0.4 }

        // 144 pin (motor) - yukseklik = mm degeri
        Repeater3D {
            model: app.gridData
            delegate: Model {
                source: "#Cube"
                property int gi: index
                property real val: modelData
                property real h: (val / 600.0) * root.maxHeight + 2
                property int rr: Math.floor(gi / 12)
                property int cc: gi % 12
                position: Qt.vector3d((cc - 5.5) * root.spacing, h / 2, (rr - 5.5) * root.spacing)
                scale: Qt.vector3d(root.footprint / 100, h / 100, root.footprint / 100)
                materials: [
                    PrincipledMaterial { baseColor: Theme.heat(val, 600); roughness: 0.5; metalness: 0.0 }
                ]
            }
        }
    }

    Text {
        anchors.centerIn: parent
        visible: app.gridData.length === 0
        text: "STL yukleyip 'Hesapla' deyince\nkalip yuzeyi burada gorunur"
        color: Theme.textDim; font.pixelSize: Theme.fsBody
        horizontalAlignment: Text.AlignHCenter
    }

    Text {
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottomMargin: 6
        visible: app.gridData.length > 0
        text: "Surukle: dondur · Iki parmak: yakinlas"
        color: Theme.textDim; font.pixelSize: Theme.fsSmall
    }
}
