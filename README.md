# DTRSwarms
Perception repository for various visual servoing methodologies for the autonomous blimps serving for DTR competition.

## BallonDetection
Repository for various colored ballon detection, sending results using UART.

### pink_ballon
- `pbOMV.py`: file on OpenMV that sends detection result via UART 
- `pbPC.py`: file on PC that repeatedly register pink_ballon callback functions on OpenMV
- `rpc.py`: rpc library dependency

## GoalDetection
Repository for various colored goal detection, sending results using UART & RPC.

## LightFollowing
Repository for collision detection - using lights as landmarks, sending results using RPC.

- `lfOMV.py`: file on OpenMV that sends detection result `r-l, intensity` via rpc
- `lfPC.py`: file on PC that repeatedly registers light_following callback functions on OpenMV
- `rpc.py`: rpc library dependency

## Integration
Repository for state machined with integrated functionality.

### CommuinicateGoalBlob.py
April. 18 - First version
