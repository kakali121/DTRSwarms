# Pink Ballon Detection

### `pbOMV.py`
1. Detects color blob within the threshold
2. Tracks the biggest blob detected
3. Sends detection result `cx, cy, w, h` via UART

### `pbPC.py`
PC repeatedly registers pink_ballon callback functions on OpenMV

### `rpc.py`
rpc library dependency