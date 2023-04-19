# Collision Detection - Light Following

### `lfOMV.py`
1. Computes pixels information on the left and right corners
3. Sends detection result `r-l, intensity` via UART

### `lfPC.py`
PC repeatedly registers light_following callback functions on OpenMV

### `rpc.py`
rpc library dependency