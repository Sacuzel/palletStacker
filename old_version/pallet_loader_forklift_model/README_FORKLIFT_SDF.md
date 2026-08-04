# Simplified controllable forklift for Gazebo Harmonic

This addition places a three-link forklift in every generated Gazebo world:

1. `base_link`: one rectangular electric-forklift body.
2. `left_fork_link`: one rectangular fork tine.
3. `right_fork_link`: one rectangular fork tine.

There are no wheels, mast meshes, tapered tips, or other physical components.
The body is moved in its local forward direction by Gazebo's `VelocityControl`
system. Each fork is attached directly to the body by a vertical prismatic joint.

## Reference geometry

The defaults in `config.py` are based on representative Toyota 3-wheel electric
warehouse-truck dimensions:

- body length to fork face: 1.83 m
- body width: 1.05 m
- body height: 2.05 m
- each fork: 1.067 x 0.102 x 0.040 m
- fork centre spacing: 0.550 m

The model mass and contact values are simulation parameters, not a claim that
all warehouse forklifts have the same mass or friction.

## Controls

The controls are press-and-hold:

| Input | Action |
|---|---|
| Up arrow | Move forward |
| Down arrow | Move backward |
| Left arrow | Turn counterclockwise |
| Right arrow | Turn clockwise |
| Shift + Up arrow | Raise both forks |
| Shift + Down arrow | Lower both forks |
| Space | Emergency stop for body and forks |

Maximum linear speed is 2.0 m/s. The GUI controller reads Gazebo world
statistics and ramps the command using simulation time at no more than 0.5 m/s2.
It does not accumulate acceleration while the world is paused. Angular speed and
angular acceleration have separate tunable limits in `config.py`.

## One-time GUI plugin build

Gazebo's stock `KeyPublisher` publishes only a key code and does not expose the
modifier state or key-release event needed for Shift+Arrow and press-and-hold
control. Therefore, the project includes a small Gazebo GUI plugin.

On Ubuntu Noble with Gazebo Harmonic installed, build and install it once:

```bash
./plugins/forklift_teleop/build_and_install.sh
```

The script installs:

```text
~/.gz/gui/plugins/libForkliftTeleop.so
```

Gazebo GUI searches that directory automatically. The plugin requires the
Harmonic development packages (`gz-gui8`, `gz-transport13`, and `gz-msgs10`)
and Qt 5 development components. A typical package installation is:

```bash
sudo apt update
sudo apt install cmake g++ qtbase5-dev qtdeclarative5-dev \
  qtquickcontrols2-5-dev libgz-gui8-dev libgz-transport13-dev libgz-msgs10-dev
```

If CMake reports a missing `CPPZMQ::CPPZMQ` target on Ubuntu Noble, install:

```bash
sudo apt install cppzmq-dev
```

## Run

Generate the packed world normally:

```bash
python main.py
```

Then open it:

```bash
gz sim gazebo_runs/latest/pallet_towers.sdf
```

Click Play, click once in the 3D viewport, and use the controls above.

A generated empty-pallet test world is included at:

```bash
gz sim examples/forklift_pallet_world.sdf
```

## Geometry and safety constraints

- The two fork joints receive identical velocity commands.
- Joint zero puts the fork bottom 30 mm above the floor.
- Fork thickness is 40 mm, so the initial upper face is at 70 mm.
- Maximum joint travel is calculated as:

  `body height - initial fork bottom - fork thickness`

  With the defaults, this is `2.05 - 0.03 - 0.04 = 1.98 m`.
- At maximum travel, the fork upper face is exactly level with the body upper
  face. The SDF joint limit prevents it from moving higher.
- Spawn distance is measured from the fork tips to the nearest open face of the
  first pallet. The default is exactly 2.0 m.
- The forklift faces the negative-Y side of pallet 1, which is one of the open
  fork-channel sides of the simplified pallet.

## Configuration ownership

All tunable dimensions, masses, speeds, acceleration limits, topics, key codes,
spawn pose values, friction values, and joint limits originate from `config.py`.
`gazebo_exporter.py` reads those values and inlines the configured model into the
world. The packing algorithm, stability logic, and Plotly visualization remain
unchanged.

## Validation performed in the delivery environment

The following checks were completed:

- all Python modules compile;
- standalone model and generated world are well-formed XML;
- generated forklift contains exactly three links and two prismatic joints;
- fork upper limits resolve to 1.98 m with the default geometry;
- the generated GUI contains the custom teleoperation plugin;
- the unpaused / paused option is propagated to the GUI world-control plugin;
- the fork-tip-to-pallet-face spawn distance resolves to 2.0 m.

Gazebo and its development headers were not installed in the delivery container,
so the C++ GUI plugin could not be compiled or run there. The included source
follows the Gazebo GUI 8 plugin pattern and must be built on the target Ubuntu /
Gazebo Harmonic machine using the script above.
