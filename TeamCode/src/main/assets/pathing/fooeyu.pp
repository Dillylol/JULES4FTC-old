{
  "startPoint": {
    "x": 33,
    "y": 138,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-b5ldzl3omka",
      "name": "1. Start -> Score Pre-load",
      "endPoint": {
        "x": 42,
        "y": 102,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 45
      },
      "controlPoints": [
        {
          "x": 48,
          "y": 105
        }
      ],
      "color": "#6CB966",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8ati4-s3o6ay",
      "name": "2. Score -> Pickup 1 ",
      "endPoint": {
        "x": 39,
        "y": 93,
        "heading": "linear",
        "reverse": false,
        "startDeg": 45,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 48,
          "y": 96
        }
      ],
      "color": "#BC657D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8dhdy-mcnxsg",
      "name": "3. Pickup 1 -> Score",
      "endPoint": {
        "x": 15,
        "y": 93,
        "heading": "tangential",
        "reverse": false,
        "startDeg": 180,
        "endDeg": -135
      },
      "controlPoints": [],
      "color": "#675557",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8ebma-ttkvv6",
      "name": "4. Score -> Pickup 2 ",
      "endPoint": {
        "x": 42,
        "y": 102,
        "heading": "linear",
        "reverse": true,
        "startDeg": 180,
        "endDeg": 45
      },
      "controlPoints": [
        {
          "x": 57,
          "y": 93
        }
      ],
      "color": "#7A8DDD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml2krckb-q684s4",
      "endPoint": {
        "x": 39,
        "y": 60,
        "heading": "linear",
        "reverse": false,
        "startDeg": 45,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#CD9CCB",
      "name": "Path 8",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8g427-vmhfb5",
      "name": "5. Pickup 2 -> Score",
      "endPoint": {
        "x": 15,
        "y": 60,
        "heading": "tangential",
        "reverse": false,
        "startDeg": 180,
        "endDeg": -135,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#98DC7D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8gogm-cmjan5",
      "name": "6. Score -> Pickup 3 ",
      "endPoint": {
        "x": 42,
        "y": 102,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 45
      },
      "controlPoints": [
        {
          "x": 42,
          "y": 72
        }
      ],
      "color": "#98D69A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkw8lqj5-10p18g",
      "name": "park",
      "endPoint": {
        "x": 42,
        "y": 48,
        "heading": "linear",
        "reverse": false,
        "startDeg": 45,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#D768A8",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 7,
          "y": 119
        },
        {
          "x": 24,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-b5ldzl3omka"
    },
    {
      "kind": "wait",
      "id": "mkw8rg5a-j51nap",
      "name": "Wait",
      "durationMs": 4000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkw8ati4-s3o6ay"
    },
    {
      "kind": "path",
      "lineId": "mkw8dhdy-mcnxsg"
    },
    {
      "kind": "path",
      "lineId": "mkw8ebma-ttkvv6"
    },
    {
      "kind": "wait",
      "id": "mkw8ry34-dnss6s",
      "name": "Wait",
      "durationMs": 4000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "ml2krckb-q684s4"
    },
    {
      "kind": "path",
      "lineId": "mkw8g427-vmhfb5"
    },
    {
      "kind": "path",
      "lineId": "mkw8gogm-cmjan5"
    },
    {
      "kind": "wait",
      "id": "mkw8sbp6-9fde8f",
      "name": "Wait",
      "durationMs": 4000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkw8lqj5-10p18g"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-01-31T19:08:30.079Z"
}