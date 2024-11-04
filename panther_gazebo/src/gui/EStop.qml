// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import "qrc:/qml"

Rectangle {
  id: widgetRectangle
  color: "white"
  anchors.fill: parent
  focus: true
  Layout.minimumWidth: 275
  Layout.minimumHeight: 225

  RowLayout {
    id: namespaceLayout
    anchors.top: widgetRectangle.top
    anchors.topMargin: 10
    anchors.left: widgetRectangle.left
    anchors.leftMargin: 10
    spacing: 10

    Label {
      id: namespaceLabel
      text: "Namespace:"
      Layout.fillWidth: false
      Layout.margins: 10
      Layout.alignment: Qt.AlignLeft
    }

    TextField {
      id: namespaceField
      width: 175
      Layout.fillWidth: true
      text: EStop.ns
      placeholderText: qsTr("Robot namespace")
      Layout.alignment: Qt.AlignLeft
      onEditingFinished: {
        EStop.SetNamespace(text)
      }
    }
  }

  ToolButton {
    id: eStopButton
    anchors.top: namespaceLayout.bottom
    anchors.topMargin: 10
    anchors.horizontalCenter: widgetRectangle.horizontalCenter
    checkable: true
    checked: false
    width: 100
    height: 100
    contentItem: Rectangle {
      width: parent.width
      height: parent.height
      radius: 50
      color: eStopButton.checked ? "#d70000" : "#66b849"
      border.width: 2
      border.color: "#909090"

      Text {
        anchors.centerIn: parent
        text: eStopButton.checked ? "STOP" : "GO"
        font.bold: true
        color: "white"
        font.pixelSize: 24
      }
    }
    onPressed: {
      EStop.ButtonPressed(eStopButton.checked);
    }
  }
}
