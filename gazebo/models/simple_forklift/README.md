# Simple forklift model

Files to merge into `~/Projects/palletStackerWS`:

```text
code/pallet_stacker/settings.py
code/pallet_stacker/generateForkliftModel.py
code/pallet_stacker/forkliftTeleop.py
gazebo/models/simple_forklift/model.config
gazebo/models/simple_forklift/model.sdf
gazebo/bridge/forklift_bridge.yaml
gazebo/worlds/forklift_test.sdf
```

Regenerate the model after changing forklift values in `settings.py`:

```bash
cd ~/Projects/palletStackerWS
source .venv/bin/activate
PYTHONPATH=code python -m pallet_stacker.generateForkliftModel
```

Run the test world:

```bash
cd ~/Projects/palletStackerWS
export GZ_SIM_RESOURCE_PATH="$PWD/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r gazebo/worlds/forklift_test.sdf
```

Run the bridge in a second terminal:

```bash
cd ~/Projects/palletStackerWS
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="$PWD/gazebo/bridge/forklift_bridge.yaml"
```

Run teleoperation in a third terminal. Use the ROS system Python because a
normal isolated virtual environment usually cannot import apt-installed
`rclpy`:

```bash
cd ~/Projects/palletStackerWS
source /opt/ros/jazzy/setup.bash
PYTHONPATH=code /usr/bin/python3 -m pallet_stacker.forkliftTeleop
```

Controls: W/S forward/reverse, A/D turn, up/down arrows move both forks,
Space stops the chassis, and Q exits.
