{
  "startPoint": {
    "x": 90,
    "y": 9,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-1b9d1bgmwl1",
      "name": "go to preload zone, shoot",
      "endPoint": {
        "x": 72,
        "y": 72,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 105
      },
      "controlPoints": [],
      "color": "#CC6889",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlrq8umc-4p8err",
      "name": "preload to first catch",
      "endPoint": {
        "x": 129,
        "y": 36,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 99,
          "y": 36
        }
      ],
      "color": "#AAB697",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlrq9taj-qfof62",
      "name": "go to shoot #2",
      "endPoint": {
        "x": 72,
        "y": 72,
        "heading": "linear",
        "reverse": true,
        "startDeg": 0,
        "endDeg": 105
      },
      "controlPoints": [],
      "color": "#D8696A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlrqha2v-31l4ol",
      "name": "Path 7",
      "endPoint": {
        "x": 81,
        "y": 36,
        "heading": "linear",
        "reverse": false,
        "startDeg": 105,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#88C6D5",
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
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
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
      "lineId": "line-1b9d1bgmwl1"
    },
    {
      "kind": "path",
      "lineId": "mlrq8umc-4p8err"
    },
    {
      "kind": "path",
      "lineId": "mlrq9taj-qfof62"
    },
    {
      "kind": "path",
      "lineId": "mlrqha2v-31l4ol"
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
  "timestamp": "2026-02-19T08:30:58.629Z"
}