# Branch `port/jazzy`

### Fixed Simulation Freezing
* `sdls_servo.py` executed IK with `MultiThreadedExecutor()` which worked on humble, but due to Python GIR changes in jazzy threads conflicted for CPU, and caused `/jointstates` to update below 50Hz instead of 100Hz. Switching to `SingleThreadedProcessor()` fixed the issue