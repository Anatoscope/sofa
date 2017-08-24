import QtQuick 2.0
import QtQuick.Controls 1.3
import QtQuick.Layouts 1.0
import QtQuick.Dialogs 1.1
import Qt.labs.settings 1.0
import SofaBasics 1.0
import SofaManipulators 1.0

SofaSceneInterface {
    id: root

    toolpanel: ColumnLayout {
         enabled: sofaScene.ready

        Component.onCompleted: {
            SofaApplication.focusedSofaViewer.backgroundColor="#FFFFFFFF"
        }
        
        GroupBox {

            title: "Transformation"
            visible: true
            enabled: fileDialog.fileUrl !== ""

            implicitWidth: parent.width

            GridLayout {
                Layout.fillWidth: true
                columns: 3

                Label {
                    Layout.preferredWidth: implicitWidth
                    text: ""
                }
                RowLayout {
                    Layout.columnSpan: 2
                    Layout.fillWidth: true

                    Label {
                        Layout.fillWidth: true
                        Layout.preferredWidth: 70
                        horizontalAlignment: Text.AlignHCenter
                        text: "X    "
                        color: enabled ? "red" : "darkgrey"
                        font.bold: true
                    }
                    Label {
                        Layout.fillWidth: true
                        Layout.preferredWidth: 70
                        horizontalAlignment: Text.AlignHCenter
                        text: "Y    "
                        color: enabled ? "green" : "darkgrey"
                        font.bold: true
                    }
                    Label {
                        Layout.fillWidth: true
                        Layout.preferredWidth: 70
                        horizontalAlignment: Text.AlignHCenter
                        text: "Z    "
                        color: enabled ? "blue" : "darkgrey"
                        font.bold: true
                    }
                }

                
                // translation
                Label {
                    Layout.preferredWidth: implicitWidth
                    text: "Translation"
                }
                Vector3DSpinBox {
                    id: translation
                    Layout.columnSpan: 2
                    Layout.fillWidth: true
                    decimals: 4
                    stepSize:0.01

                    function update() {
                        sofaScene.sofaPythonInteractor.call("/moveController", "setTranslation", [vx, vy, vz] );
                    }

                    Component.onCompleted: {
                        onVxChanged.connect(update); onVyChanged.connect(update); onVzChanged.connect(update);
                    }
                }

                // rotation
                Label {
                    Layout.preferredWidth: implicitWidth
                    text: "Rotation"
                }
                Vector3DSpinBox {
                    id: rotation
                    Layout.columnSpan: 2
                    Layout.fillWidth: true
                    decimals: 0
                    stepSize:1

                    function update() {
                        sofaScene.sofaPythonInteractor.call("/moveController", "setRotation", [vx, vy, vz] );
                    }

                    Component.onCompleted: {
                        onVxChanged.connect(update); onVyChanged.connect(update); onVzChanged.connect(update);
                    }
                }

                // scale
                Label {
                    Layout.preferredWidth: implicitWidth
                    text: "Scale"
                }
                Vector3DSpinBox {
                    id: scale
                    Layout.columnSpan: 2
                    Layout.fillWidth: true
                    decimals: 3
                    stepSize:0.01
                    function update() {
                        sofaScene.sofaPythonInteractor.call("/moveController", "setScale", [vx, vy, vz] );
                    }

                    Component.onCompleted: {
                        onVxChanged.connect(update); onVyChanged.connect(update); onVzChanged.connect(update);
                    }
                }
            }
        }
    }
}
