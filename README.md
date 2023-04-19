# DTRSwarms
Perception repository for visual servoing for DTR competition.

## BallonDetection
Repository for various colored ballon detection (OpenMV), sending results using UART & RPC (OpenMV & PC).

### pink_ballon
- `pbOMV.py`: file on OpenMV that sends detection result via UART 
- `pbPC.py`: file on PC that repeatedly register pink_ballon callback functions on OpenMV
- `rpc.py`: rpc library dependency


## GoalDetection
Repository for various colored goal detection (OpenMV), sending results using UART & RPC (OpenMV & PC).

## LightFollowing
Repository for collision detection - using lights as landmarks (OpenMV), sending results using RPC (OpenMV & PC).

- `lfOMV.py`: file on OpenMV that sends detection result `r-l, intensity` via rpc
- `lfPC.py`: file on PC that repeatedly registers light_following callback functions on OpenMV
- `rpc.py`: rpc library dependency

## Integration
Repository for state machined with integrated functionality.

### CommuinicateGoalBlob.py
April. 18 - First version
