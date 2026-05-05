{
  "startPoint": {
    "x": 33.5,
    "y": 135.5,
    "heading": "linear",
    "startDeg": 180,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "path-startpreloadshoot",
      "name": "Start to Preload Shoot",
      "endPoint": {
        "x": 51,
        "y": 93,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": -170,
        "reverse": false
      },
      "controlPoints": [],
      "color": "#979858",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 4000,
      "waitBeforeName": "",
      "waitAfterName": "Shooting Preload"
    },
    {
      "id": "path-preloadshootrow1approach",
      "name": "Preload Shoot to Row 1 Approach",
      "endPoint": {
        "x": 42,
        "y": 84,
        "heading": "linear",
        "startDeg": -170,
        "endDeg": 180,
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 45,
          "y": 84.01164153949128
        }
      ],
      "color": "#A8A888",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "path-row1approw1capture",
      "name": "Row 1 Approach to Row 1 Capture",
      "endPoint": {
        "x": 18,
        "y": 84,
        "heading": "tangential",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#9769C6",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "Intake On",
      "waitAfterName": ""
    },
    {
      "id": "path-row1captureshootrow1",
      "name": "Row 1 Capture to Shoot Row 1",
      "endPoint": {
        "x": 51,
        "y": 93,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": -170,
        "reverse": false
      },
      "controlPoints": [],
      "color": "#8CD777",
      "waitBeforeMs": 0,
      "waitAfterMs": 4000,
      "waitBeforeName": "",
      "waitAfterName": "Shooting Row 1"
    },
    {
      "id": "path-shootrow1row2approach",
      "name": "Shoot Row 1 to Row 2 Approach",
      "endPoint": {
        "x": 45,
        "y": 60,
        "heading": "linear",
        "startDeg": -170,
        "endDeg": 180,
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 48.472446854235386,
          "y": 63.43435958007255
        }
      ],
      "color": "#857775",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "path-row2approw2capture",
      "name": "Row 2 Approach to Row 2 Capture",
      "endPoint": {
        "x": 18,
        "y": 60,
        "heading": "tangential",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#9AC8D8",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "Intake On",
      "waitAfterName": ""
    },
    {
      "id": "path-row2capturerow2shoot",
      "name": "Row 2 Capture to Row 2 Shoot",
      "endPoint": {
        "x": 51,
        "y": 93,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": -170,
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 30,
          "y": 63
        }
      ],
      "color": "#69CC7D",
      "waitBeforeMs": 0,
      "waitAfterMs": 4000,
      "waitBeforeName": "",
      "waitAfterName": "Shooting Row 2"
    },
    {
      "id": "path-row2shootrow3approach",
      "name": "Row 2 Shoot to Row 3 Approach",
      "endPoint": {
        "x": 48,
        "y": 36,
        "heading": "linear",
        "startDeg": -170,
        "endDeg": 180,
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 48,
          "y": 51
        }
      ],
      "color": "#C575AA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "path-row3approw3capture",
      "name": "Row 3 Approach to Row 3 Capture",
      "endPoint": {
        "x": 18,
        "y": 36,
        "heading": "tangential",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#7ABB9A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "Intake On",
      "waitAfterName": ""
    },
    {
      "id": "path-row3capturepark",
      "name": "Row 3 Capture to Park",
      "endPoint": {
        "x": 27,
        "y": 51,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180,
        "reverse": false
      },
      "controlPoints": [],
      "color": "#7ABA8A",
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
      "lineId": "path-startpreloadshoot"
    },
    {
      "kind": "path",
      "lineId": "path-preloadshootrow1approach"
    },
    {
      "kind": "path",
      "lineId": "path-row1approw1capture"
    },
    {
      "kind": "path",
      "lineId": "path-row1captureshootrow1"
    },
    {
      "kind": "path",
      "lineId": "path-shootrow1row2approach"
    },
    {
      "kind": "path",
      "lineId": "path-row2approw2capture"
    },
    {
      "kind": "path",
      "lineId": "path-row2capturerow2shoot"
    },
    {
      "kind": "path",
      "lineId": "path-row2shootrow3approach"
    },
    {
      "kind": "path",
      "lineId": "path-row3approw3capture"
    },
    {
      "kind": "path",
      "lineId": "path-row3capturepark"
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
  "timestamp": "2026-02-21T01:31:55.342Z"
}