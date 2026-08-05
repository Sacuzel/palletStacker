# Integrated Gazebo simulation update

This patch changes the Gazebo stage to the intended two-command workflow:

```bash
python code/main.py
gz sim gazebo/worlds/pallet_stacker_world.sdf
```

`main.py` generates the world and all supporting assets but does not launch
Gazebo. The generated world includes the pallets, loaded boxes, forklift,
rendering configuration, physics configuration, and a GUI declaration for the
integrated forklift controller.

## One-time build dependencies

The first `main.py` run compiles a small Gazebo GUI plugin. Install the required
development packages once on Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic:

```bash
sudo apt install build-essential cmake libgz-gui8-dev \
  libgz-transport13-dev libgz-msgs10-dev qtbase5-dev \
  qtdeclarative5-dev qtquickcontrols2-5-dev
```

## What main.py creates or updates

- `gazebo/worlds/pallet_stacker_world.sdf`
- `gazebo/worlds/pallet_stacker_world_manifest.json`
- `gazebo/models/simple_forklift/model.sdf`
- `gazebo/models/simple_forklift/model.config`
- project-local plugin build output under
  `gazebo/plugins/forklift_teleop/build/`
- user-local runtime plugin
  `~/.gz/gui/plugins/libForkliftTeleop.so`

The last location is Gazebo GUI's normal per-user plugin directory. No sudo is
used to copy the plugin there. `main.py` prints the exact path so the generated
runtime asset is not hidden.

The world uses:

```xml
<plugin filename="ForkliftTeleop" name="Forklift controls">
```

The base name is intentional: Gazebo uses it both to find
`libForkliftTeleop.so` and to find the embedded
`:/ForkliftTeleop/ForkliftTeleop.qml` resource.

## Integrated controls

- `W` / `S`: forward and reverse
- `A` / `D`: turn left and right
- `Up` / `Down`: change the fork position target
- `Space`: stop chassis motion

Fork height is position-controlled. Releasing an arrow key leaves the final
position target active, so the forks hold that height.

The GUI plugin also retries a `pause: false` request during startup. Therefore
plain `gz sim <world>` starts the simulation without a separate `-r` flag,
bridge process, ROS node, or teleoperation terminal.

## Applying the patch

Copy the files from this bundle over the matching paths in
`palletStackerWS`. Do not copy any `build/` or `__pycache__/` directory from an
older version. The first generation run will create a clean plugin build.

## Git hygiene

Keep generated plugin binaries and caches out of the repository:

```gitignore
gazebo/plugins/forklift_teleop/build/
__pycache__/
*.pyc
```

The runtime copy under `~/.gz/gui/plugins/` is outside the repository already.
