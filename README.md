# DTRSwarms
Repository for work done for DTR competition

## BallonDetection
Repository for various colored ballon detection (OpenMV), sending results using UART & RPC (OpenMV & PC).

- pink_ballon
    - `pbOMV.py`: file on OpenMV that sends detection result via rpc
    - `pbPC.py`: file on PC that register call_back functions on OpenMV
    - `rpc.py`: rpc library dependency

## GoalDetection
Repository for various colored goal detection (OpenMV), sending results using UART & RPC (OpenMV & PC).

## LightFollowing
Repository for collision detection - using lights as landmarks (OpenMV), sending results using RPC (OpenMV & PC).

## Integration
Repository for state machined with integrated functionality.

### CommuinicateGoalBlob.py
April. 18 - First version
