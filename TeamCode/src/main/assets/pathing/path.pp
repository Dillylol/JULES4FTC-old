{
  "startPoint": {
    "x": 33.86770428015564,
    "y": 135.47081712062257,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-oavum7hje7i",
      "name": "Start --> Score (90 deg)",
      "endPoint": {
        "x": 39,
        "y": 105,
        "heading": "constant",
        "startDeg": 90,
        "endDeg": 90,
        "degrees": 90
      },
      "controlPoints": [],
      "color": "#799BAA",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7hchfz-thgu2f",
      "name": "Score --> set 1 approach",
      "endPoint": {
        "x": 45,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "degrees": 45,
        "startDeg": 90,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#CD96DA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7hfma5-1e3cca",
      "name": "set 1 approach --> set 1 capture",
      "endPoint": {
        "x": 15,
        "y": 84,
        "heading": "constant",
        "reverse": false,
        "startDeg": 45,
        "endDeg": 180,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#BB5D78",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7hig2p-ld2u2v",
      "name": "set 1 capture --> score 2 (180 deg)",
      "endPoint": {
        "x": 39,
        "y": 105,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#8859C6",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7ivh0e-ipj7qg",
      "name": "Score 2 --> set 2 approach",
      "endPoint": {
        "x": 42,
        "y": 60,
        "heading": "linear",
        "reverse": true,
        "degrees": 180,
        "startDeg": 90,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 45,
          "y": 75
        }
      ],
      "color": "#C6DC6B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7mhsnc-yavkw8",
      "endPoint": {
        "x": 15,
        "y": 60,
        "heading": "constant",
        "reverse": false,
        "startDeg": 90,
        "endDeg": 180,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#AC6B66",
      "name": "set 2 approach --> Set 2 capture",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7ixq5g-5v61r8",
      "name": "set 2 capture --> score 3 (90)",
      "endPoint": {
        "x": 39,
        "y": 105,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 90
      },
      "controlPoints": [
        {
          "x": 15,
          "y": 75
        }
      ],
      "color": "#ff0000",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7kgl6r-q06xlg",
      "name": "Score 3 --> Set 3 approach",
      "endPoint": {
        "x": 42,
        "y": 36,
        "heading": "linear",
        "reverse": false,
        "startDeg": 90,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 54,
          "y": 36
        }
      ],
      "color": "#5865AD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7kuy9y-8b3074",
      "name": "set  3 approach --> set 3 capture",
      "endPoint": {
        "x": 15,
        "y": 36,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#655DC8",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7kvvlx-ewe6zy",
      "name": "set 3 capture --> park",
      "endPoint": {
        "x": 66,
        "y": 36,
        "heading": "tangential",
        "reverse": true,
        "startDeg": 180,
        "endDeg": null
      },
      "controlPoints": [],
      "color": "#5B957B",
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
      "lineId": "line-oavum7hje7i"
    },
    {
      "kind": "wait",
      "id": "ml7hp1qm-fyxzfc",
      "name": "Wait",
      "durationMs": 1000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "ml7hchfz-thgu2f"
    },
    {
      "kind": "path",
      "lineId": "ml7hfma5-1e3cca"
    },
    {
      "kind": "path",
      "lineId": "ml7hig2p-ld2u2v"
    },
    {
      "kind": "wait",
      "id": "ml7igeih-11zjeh",
      "name": "Wait",
      "durationMs": 1000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "ml7ivh0e-ipj7qg"
    },
    {
      "kind": "path",
      "lineId": "ml7mhsnc-yavkw8"
    },
    {
      "kind": "path",
      "lineId": "ml7ixq5g-5v61r8"
    },
    {
      "kind": "wait",
      "id": "ml7mpmqu-647kb0",
      "name": "Wait",
      "durationMs": 1000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "ml7kgl6r-q06xlg"
    },
    {
      "kind": "path",
      "lineId": "ml7kuy9y-8b3074"
    },
    {
      "kind": "path",
      "lineId": "ml7kvvlx-ewe6zy"
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
  "timestamp": "2026-02-04T06:41:22.076Z"
}