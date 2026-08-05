# ForkliftTeleop Gazebo GUI plugin

This plugin provides forklift keyboard control directly inside the Gazebo GUI.
It publishes Gazebo Transport messages to the forklift model, so the generated
world needs neither ROS 2, `ros_gz_bridge`, nor a separate teleoperation process.

`python code/main.py` performs the required preparation automatically:

1. CMake builds `build/libForkliftTeleop.so` inside this project.
2. The current library is installed or updated at
   `~/.gz/gui/plugins/libForkliftTeleop.so`.
3. The generated world writes `<plugin filename="ForkliftTeleop">` in its GUI
   configuration.

Gazebo GUI searches `~/.gz/gui/plugins` by default. The stable plugin filename is
also required because Gazebo derives the embedded QML resource path from it:
`:/ForkliftTeleop/ForkliftTeleop.qml`.

Normal workflow:

```bash
python code/main.py
gz sim gazebo/worlds/pallet_stacker_world.sdf
```

No extra terminal or environment variable is required during simulation.

Controls:

- `W` / `S`: forward and reverse
- `A` / `D`: turn left and right
- `Up` / `Down`: raise and lower the persistent fork position target
- `Space`: stop chassis motion immediately

The plugin requests `pause: false` from the world-control service when automatic
start is enabled in `settings.py`. If the service is not ready immediately, the
request is retried during Gazebo startup.

One-time Ubuntu 24.04 / Gazebo Harmonic build dependencies:

```bash
sudo apt install build-essential cmake libgz-gui8-dev \
  libgz-transport13-dev libgz-msgs10-dev qtbase5-dev \
  qtdeclarative5-dev qtquickcontrols2-5-dev
```
