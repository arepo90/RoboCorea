# Branch `port/jazzy`

## Applied Patches

### Fixed Simulation Freezing
* `sdls_servo.py` executed IK with `MultiThreadedExecutor()` which worked on humble, but due to Python GIR changes in jazzy threads conflicted for CPU, and caused `/jointstates` to update below 50Hz instead of 100Hz. Switching to `SingleThreadedProcessor()` fixed the issue

### Self colission Fails to start up
* `self_colission.py` uses libraries `fcl` and `trimesh` for collision. port/jazzy updates deprecated library `fcl` to `hppfcl` and pip requires virtual [`venv`] environment to install `trimesh`. *PD: porting trimesh to newer version is WIP since trimesh is not included as a standard dependency for rosdep and little documentation was found

### Enables `laptop.sh`, `jetson.sh` and `run_robot_manager.sh` to open in non-humble environment
* Replaces hard coded directory with new `${VERSION}` variable insertion into the source file directory in .sh files 

## Other Known Issues
* `gazebo_plugins` is not recognized as a valid dependency name by rosdep, but simulation still launches without it