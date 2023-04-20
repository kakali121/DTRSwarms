# Pink Ballon Detection

### `pbOMV.py`

#### Functions
1. Detects color blob within the threshold
2. Tracks the biggest blob detected
3. Sends detection result `cx, cy, w, h` via UART

#### Camera Setup
- pixformat=sensor.RGB565
- framesize=sensor.HQVGA
- windowsize=None
- gain=18
- autoexposure=False
- exposure=5000
- autowhitebal=False
- contrast=0
- saturation=0

#### Ballon Threshold
(30, 75, 40, 70, -40, 20)


### `pbPC.py`
PC repeatedly registers pink_ballon callback functions on OpenMV

### `rpc.py`
rpc library dependency - must be in the same directory with `pbPC.py`