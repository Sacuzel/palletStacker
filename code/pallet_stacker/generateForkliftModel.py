"""Generate the simplified three-link forklift Gazebo model.

The generated SDF contains exactly three links:

* ``body``: one rigid link rendered and collided as front and rear boxes;
* ``left_fork``;
* ``right_fork``.

All user-adjustable model values are read from :mod:`pallet_stacker.settings`.
Run from the project root with::

    PYTHONPATH=code python -m pallet_stacker.generateForkliftModel
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from textwrap import dedent
from xml.etree import ElementTree

from . import settings


@dataclass(frozen=True, slots=True)
class Inertia:
    ixx: float
    iyy: float
    izz: float


def _box_inertia(mass: float, length: float, width: float, height: float) -> Inertia:
    """Return principal inertia of an axis-aligned solid cuboid."""

    return Inertia(
        ixx=mass * (width**2 + height**2) / 12.0,
        iyy=mass * (length**2 + height**2) / 12.0,
        izz=mass * (length**2 + width**2) / 12.0,
    )


def _body_mass_properties() -> tuple[float, float, Inertia]:
    """Return body mass, rear-shifted COM x, and composite inertia."""

    fork_mass_total = 2.0 * settings.FORKLIFT_FORK_MASS_KG
    body_mass = settings.FORKLIFT_TOTAL_MASS_KG - fork_mass_total
    if body_mass <= 0.0:
        raise ValueError("Fork masses must be lower than total forklift mass.")

    front_fraction = settings.FORKLIFT_BODY_FRONT_LENGTH_FRACTION
    if not 0.0 < front_fraction < 1.0:
        raise ValueError("FORKLIFT_BODY_FRONT_LENGTH_FRACTION must be in (0, 1).")

    front_length = settings.FORKLIFT_BODY_LENGTH_M * front_fraction
    rear_length = settings.FORKLIFT_BODY_LENGTH_M - front_length
    ratio = settings.FORKLIFT_REAR_TO_FRONT_BODY_MASS_RATIO
    if ratio <= 0.0:
        raise ValueError("Rear-to-front body mass ratio must be positive.")

    front_mass = body_mass / (1.0 + ratio)
    rear_mass = body_mass - front_mass

    # Model origin is at the centre of the body's front face on the floor.
    front_x = -front_length / 2.0
    rear_x = -front_length - rear_length / 2.0
    com_x = (front_mass * front_x + rear_mass * rear_x) / body_mass

    front_i = _box_inertia(
        front_mass,
        front_length,
        settings.FORKLIFT_BODY_WIDTH_M,
        settings.FORKLIFT_BODY_HEIGHT_M,
    )
    rear_i = _box_inertia(
        rear_mass,
        rear_length,
        settings.FORKLIFT_BODY_WIDTH_M,
        settings.FORKLIFT_BODY_HEIGHT_M,
    )

    # Parallel-axis correction. Both sub-box centres differ only in X.
    front_dx = front_x - com_x
    rear_dx = rear_x - com_x
    inertia = Inertia(
        ixx=front_i.ixx + rear_i.ixx,
        iyy=(
            front_i.iyy
            + front_mass * front_dx**2
            + rear_i.iyy
            + rear_mass * rear_dx**2
        ),
        izz=(
            front_i.izz
            + front_mass * front_dx**2
            + rear_i.izz
            + rear_mass * rear_dx**2
        ),
    )
    return body_mass, com_x, inertia


def _validate_settings() -> None:
    positive_values = {
        "FORKLIFT_BODY_LENGTH_M": settings.FORKLIFT_BODY_LENGTH_M,
        "FORKLIFT_BODY_WIDTH_M": settings.FORKLIFT_BODY_WIDTH_M,
        "FORKLIFT_BODY_HEIGHT_M": settings.FORKLIFT_BODY_HEIGHT_M,
        "FORKLIFT_FORK_LENGTH_M": settings.FORKLIFT_FORK_LENGTH_M,
        "FORKLIFT_FORK_WIDTH_M": settings.FORKLIFT_FORK_WIDTH_M,
        "FORKLIFT_FORK_THICKNESS_M": settings.FORKLIFT_FORK_THICKNESS_M,
        "FORKLIFT_FORK_CENTRE_SPACING_M": settings.FORKLIFT_FORK_CENTRE_SPACING_M,
        "FORKLIFT_MAX_FORK_VELOCITY_MPS": settings.FORKLIFT_MAX_FORK_VELOCITY_MPS,
    }
    for name, value in positive_values.items():
        if value <= 0.0:
            raise ValueError(f"{name} must be greater than zero.")

    if settings.FORKLIFT_FORK_MAX_POSITION_M <= settings.FORKLIFT_FORK_MIN_POSITION_M:
        raise ValueError("Fork maximum position must exceed minimum position.")
    if not (
        settings.FORKLIFT_FORK_MIN_POSITION_M
        <= settings.FORKLIFT_FORK_INITIAL_POSITION_M
        <= settings.FORKLIFT_FORK_MAX_POSITION_M
    ):
        raise ValueError("Initial fork position is outside its joint limits.")


def _fmt(value: float) -> str:
    return f"{value:.12g}"


def _surface_xml(friction: float, indent: str = "        ") -> str:
    return dedent(
        f"""
        <surface>
          <friction>
            <ode>
              <mu>{_fmt(friction)}</mu>
              <mu2>{_fmt(friction)}</mu2>
            </ode>
            <bullet>
              <friction>{_fmt(friction)}</friction>
              <friction2>{_fmt(friction)}</friction2>
              <rolling_friction>{_fmt(friction)}</rolling_friction>
            </bullet>
          </friction>
          <bounce>
            <restitution_coefficient>0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <ode>
              <kp>{_fmt(settings.FORKLIFT_CONTACT_STIFFNESS)}</kp>
              <kd>{_fmt(settings.FORKLIFT_CONTACT_DAMPING)}</kd>
              <max_vel>1</max_vel>
              <min_depth>0.0005</min_depth>
            </ode>
          </contact>
        </surface>
        """
    ).strip().replace("\n", "\n" + indent)


def build_model_sdf() -> str:
    """Build and return the complete model SDF text."""

    _validate_settings()
    body_mass, body_com_x, body_inertia = _body_mass_properties()

    body_length = settings.FORKLIFT_BODY_LENGTH_M
    front_length = body_length * settings.FORKLIFT_BODY_FRONT_LENGTH_FRACTION
    rear_length = body_length - front_length
    front_x = -front_length / 2.0
    rear_x = -front_length - rear_length / 2.0
    body_z = settings.FORKLIFT_BODY_HEIGHT_M / 2.0

    fork_x = settings.FORKLIFT_FORK_LENGTH_M / 2.0
    half_spacing = settings.FORKLIFT_FORK_CENTRE_SPACING_M / 2.0
    fork_inertia = _box_inertia(
        settings.FORKLIFT_FORK_MASS_KG,
        settings.FORKLIFT_FORK_LENGTH_M,
        settings.FORKLIFT_FORK_WIDTH_M,
        settings.FORKLIFT_FORK_THICKNESS_M,
    )

    body_surface = _surface_xml(settings.FORKLIFT_BODY_FLOOR_FRICTION)
    fork_surface = _surface_xml(settings.FORKLIFT_FORK_CONTACT_FRICTION)

    return dedent(
        f'''\
        <?xml version="1.0"?>
        <sdf version="1.10">
          <!--
            Simplified electric indoor counterbalance forklift.

            Coordinate convention:
              X = forward, toward the forks
              Y = left
              Z = upward

            Exactly three dynamic links are used: body, left_fork, right_fork.
            The body link contains two rigid box members. Its rear member carries
            twice the mass of its front member through the composite inertial model.
          -->
          <model name="{settings.FORKLIFT_MODEL_NAME}" canonical_link="body">
            <static>false</static>
            <self_collide>false</self_collide>
            <allow_auto_disable>false</allow_auto_disable>

            <frame name="fork_tip_low" attached_to="body">
              <pose relative_to="body">{_fmt(settings.FORKLIFT_FORK_LENGTH_M)} 0 {_fmt(settings.FORKLIFT_FORK_LOW_POSITION_Z_M)} 0 0 0</pose>
            </frame>

            <link name="body">
              <gravity>true</gravity>
              <kinematic>false</kinematic>
              <inertial>
                <pose>{_fmt(body_com_x)} 0 {_fmt(body_z)} 0 0 0</pose>
                <mass>{_fmt(body_mass)}</mass>
                <inertia>
                  <ixx>{_fmt(body_inertia.ixx)}</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>{_fmt(body_inertia.iyy)}</iyy>
                  <iyz>0</iyz>
                  <izz>{_fmt(body_inertia.izz)}</izz>
                </inertia>
              </inertial>

              <collision name="front_body_collision">
                <pose>{_fmt(front_x)} 0 {_fmt(body_z)} 0 0 0</pose>
                <geometry><box><size>{_fmt(front_length)} {_fmt(settings.FORKLIFT_BODY_WIDTH_M)} {_fmt(settings.FORKLIFT_BODY_HEIGHT_M)}</size></box></geometry>
                {body_surface}
              </collision>
              <visual name="front_body_visual">
                <pose>{_fmt(front_x)} 0 {_fmt(body_z)} 0 0 0</pose>
                <geometry><box><size>{_fmt(front_length)} {_fmt(settings.FORKLIFT_BODY_WIDTH_M)} {_fmt(settings.FORKLIFT_BODY_HEIGHT_M)}</size></box></geometry>
                <material>
                  <ambient>0.95 0.45 0.02 1</ambient>
                  <diffuse>1.00 0.52 0.04 1</diffuse>
                  <specular>0.15 0.15 0.15 1</specular>
                </material>
              </visual>

              <collision name="rear_body_collision">
                <pose>{_fmt(rear_x)} 0 {_fmt(body_z)} 0 0 0</pose>
                <geometry><box><size>{_fmt(rear_length)} {_fmt(settings.FORKLIFT_BODY_WIDTH_M)} {_fmt(settings.FORKLIFT_BODY_HEIGHT_M)}</size></box></geometry>
                {body_surface}
              </collision>
              <visual name="rear_body_visual">
                <pose>{_fmt(rear_x)} 0 {_fmt(body_z)} 0 0 0</pose>
                <geometry><box><size>{_fmt(rear_length)} {_fmt(settings.FORKLIFT_BODY_WIDTH_M)} {_fmt(settings.FORKLIFT_BODY_HEIGHT_M)}</size></box></geometry>
                <material>
                  <ambient>0.65 0.25 0.01 1</ambient>
                  <diffuse>0.78 0.31 0.02 1</diffuse>
                  <specular>0.12 0.12 0.12 1</specular>
                </material>
              </visual>
            </link>

            <link name="left_fork">
              <pose>{_fmt(fork_x)} {_fmt(half_spacing)} {_fmt(settings.FORKLIFT_FORK_LOW_POSITION_Z_M)} 0 0 0</pose>
              <gravity>true</gravity>
              <inertial>
                <mass>{_fmt(settings.FORKLIFT_FORK_MASS_KG)}</mass>
                <inertia>
                  <ixx>{_fmt(fork_inertia.ixx)}</ixx><ixy>0</ixy><ixz>0</ixz>
                  <iyy>{_fmt(fork_inertia.iyy)}</iyy><iyz>0</iyz><izz>{_fmt(fork_inertia.izz)}</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry><box><size>{_fmt(settings.FORKLIFT_FORK_LENGTH_M)} {_fmt(settings.FORKLIFT_FORK_WIDTH_M)} {_fmt(settings.FORKLIFT_FORK_THICKNESS_M)}</size></box></geometry>
                {fork_surface}
              </collision>
              <visual name="visual">
                <geometry><box><size>{_fmt(settings.FORKLIFT_FORK_LENGTH_M)} {_fmt(settings.FORKLIFT_FORK_WIDTH_M)} {_fmt(settings.FORKLIFT_FORK_THICKNESS_M)}</size></box></geometry>
                <material><ambient>0.22 0.24 0.27 1</ambient><diffuse>0.32 0.35 0.40 1</diffuse><specular>0.55 0.55 0.55 1</specular></material>
              </visual>
            </link>

            <link name="right_fork">
              <pose>{_fmt(fork_x)} -{_fmt(half_spacing)} {_fmt(settings.FORKLIFT_FORK_LOW_POSITION_Z_M)} 0 0 0</pose>
              <gravity>true</gravity>
              <inertial>
                <mass>{_fmt(settings.FORKLIFT_FORK_MASS_KG)}</mass>
                <inertia>
                  <ixx>{_fmt(fork_inertia.ixx)}</ixx><ixy>0</ixy><ixz>0</ixz>
                  <iyy>{_fmt(fork_inertia.iyy)}</iyy><iyz>0</iyz><izz>{_fmt(fork_inertia.izz)}</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry><box><size>{_fmt(settings.FORKLIFT_FORK_LENGTH_M)} {_fmt(settings.FORKLIFT_FORK_WIDTH_M)} {_fmt(settings.FORKLIFT_FORK_THICKNESS_M)}</size></box></geometry>
                {fork_surface}
              </collision>
              <visual name="visual">
                <geometry><box><size>{_fmt(settings.FORKLIFT_FORK_LENGTH_M)} {_fmt(settings.FORKLIFT_FORK_WIDTH_M)} {_fmt(settings.FORKLIFT_FORK_THICKNESS_M)}</size></box></geometry>
                <material><ambient>0.22 0.24 0.27 1</ambient><diffuse>0.32 0.35 0.40 1</diffuse><specular>0.55 0.55 0.55 1</specular></material>
              </visual>
            </link>

            <joint name="left_fork_lift_joint" type="prismatic">
              <parent>body</parent><child>left_fork</child>
              <axis>
                <xyz>0 0 1</xyz>
                <limit>
                  <lower>{_fmt(settings.FORKLIFT_FORK_MIN_POSITION_M)}</lower>
                  <upper>{_fmt(settings.FORKLIFT_FORK_MAX_POSITION_M)}</upper>
                  <effort>{_fmt(settings.FORKLIFT_FORK_JOINT_MAX_EFFORT_N)}</effort>
                </limit>
                <dynamics><damping>{_fmt(settings.FORKLIFT_FORK_JOINT_DAMPING)}</damping><friction>{_fmt(settings.FORKLIFT_FORK_JOINT_FRICTION)}</friction></dynamics>
              </axis>
            </joint>
            <joint name="right_fork_lift_joint" type="prismatic">
              <parent>body</parent><child>right_fork</child>
              <axis>
                <xyz>0 0 1</xyz>
                <limit>
                  <lower>{_fmt(settings.FORKLIFT_FORK_MIN_POSITION_M)}</lower>
                  <upper>{_fmt(settings.FORKLIFT_FORK_MAX_POSITION_M)}</upper>
                  <effort>{_fmt(settings.FORKLIFT_FORK_JOINT_MAX_EFFORT_N)}</effort>
                </limit>
                <dynamics><damping>{_fmt(settings.FORKLIFT_FORK_JOINT_DAMPING)}</damping><friction>{_fmt(settings.FORKLIFT_FORK_JOINT_FRICTION)}</friction></dynamics>
              </axis>
            </joint>

            <!-- Direct model velocity command. Teleop applies acceleration ramps. -->
            <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
              <topic>{settings.FORKLIFT_CMD_VEL_TOPIC}</topic>
            </plugin>

            <!--
              One position target drives both fork joints. ABS mode limits motion
              with cmd_max but retains the final target after input is released.
            -->
            <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
              <joint_name>left_fork_lift_joint</joint_name>
              <joint_name>right_fork_lift_joint</joint_name>
              <topic>{settings.FORKLIFT_FORK_POSITION_TOPIC}</topic>
              <initial_position>{_fmt(settings.FORKLIFT_FORK_INITIAL_POSITION_M)}</initial_position>
              <use_velocity_commands>true</use_velocity_commands>
              <cmd_max>{_fmt(settings.FORKLIFT_MAX_FORK_VELOCITY_MPS)}</cmd_max>
              <cmd_min>-{_fmt(settings.FORKLIFT_MAX_FORK_VELOCITY_MPS)}</cmd_min>
            </plugin>
          </model>
        </sdf>
        '''
    )


def build_model_config() -> str:
    return dedent(
        f'''\
        <?xml version="1.0"?>
        <model>
          <name>Simple electric warehouse forklift</name>
          <version>1.0.0</version>
          <sdf version="1.10">model.sdf</sdf>
          <author><name>Pallet Stacker project</name></author>
          <description>
            Three-link box-geometry electric counterbalance forklift for Gazebo Harmonic.
          </description>
        </model>
        '''
    )


def build_bridge_yaml() -> str:
    return dedent(
        f'''\
        - ros_topic_name: "{settings.FORKLIFT_CMD_VEL_TOPIC}"
          gz_topic_name: "{settings.FORKLIFT_CMD_VEL_TOPIC}"
          ros_type_name: "geometry_msgs/msg/Twist"
          gz_type_name: "gz.msgs.Twist"
          direction: ROS_TO_GZ
          publisher_queue: 10
          subscriber_queue: 10

        - ros_topic_name: "{settings.FORKLIFT_FORK_POSITION_TOPIC}"
          gz_topic_name: "{settings.FORKLIFT_FORK_POSITION_TOPIC}"
          ros_type_name: "std_msgs/msg/Float64"
          gz_type_name: "gz.msgs.Double"
          direction: ROS_TO_GZ
          publisher_queue: 10
          subscriber_queue: 10
        '''
    )


def build_test_world() -> str:
    """Build a minimal forklift-only world using shared Gazebo settings."""

    cast_shadows = "false" if settings.GAZEBO_DISABLE_SHADOWS else "true"
    ambient = " ".join(_fmt(value) for value in settings.GAZEBO_AMBIENT_LIGHT_RGBA)
    background = " ".join(_fmt(value) for value in settings.GAZEBO_BACKGROUND_RGBA)
    ground_color = " ".join(_fmt(value) for value in settings.GAZEBO_GROUND_RGBA)
    gravity = " ".join(_fmt(value) for value in settings.GAZEBO_GRAVITY_MPS2)
    ground_size = _fmt(settings.GAZEBO_GROUND_MIN_SIZE_M)

    return dedent(
        f'''\
        <?xml version="1.0"?>
        <sdf version="1.10">
          <!-- Minimal development harness, separate from gzSimulator.py output. -->
          <world name="forklift_test">
            <gravity>{gravity}</gravity>
            <physics name="default_physics" type="ignored">
              <max_step_size>{_fmt(settings.GAZEBO_PHYSICS_MAX_STEP_SIZE_S)}</max_step_size>
              <real_time_factor>{_fmt(settings.GAZEBO_REAL_TIME_FACTOR)}</real_time_factor>
            </physics>
            <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
            <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
            <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
            <scene>
              <ambient>{ambient}</ambient>
              <background>{background}</background>
              <shadows>{cast_shadows}</shadows>
            </scene>
            <light type="directional" name="sun">
              <pose>0 0 10 0 0 0</pose><cast_shadows>{cast_shadows}</cast_shadows>
              <direction>-0.5 0.2 -1</direction>
              <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
            </light>
            <model name="ground_plane">
              <static>true</static>
              <link name="link">
                <collision name="collision">
                  <geometry><plane><normal>0 0 1</normal><size>{ground_size} {ground_size}</size></plane></geometry>
                  <surface><friction><ode><mu>{_fmt(settings.GAZEBO_GROUND_FRICTION)}</mu><mu2>{_fmt(settings.GAZEBO_GROUND_FRICTION)}</mu2></ode></friction></surface>
                </collision>
                <visual name="visual">
                  <geometry><plane><normal>0 0 1</normal><size>{ground_size} {ground_size}</size></plane></geometry>
                  <material><ambient>{ground_color}</ambient><diffuse>{ground_color}</diffuse></material>
                </visual>
              </link>
            </model>
            <include>
              <uri>model://{settings.FORKLIFT_MODEL_NAME}</uri>
              <pose>0 0 {_fmt(settings.GAZEBO_FORKLIFT_SPAWN_CLEARANCE_M)} 0 0 0</pose>
            </include>
          </world>
        </sdf>
        '''
    )


def write_forklift_assets(
    model_directory: Path | None = None,
    bridge_file: Path | None = None,
    test_world_file: Path | None = None,
) -> tuple[Path, Path, Path, Path]:
    """Write the model, bridge configuration, and test world."""

    model_dir = (model_directory or settings.FORKLIFT_MODEL_DIRECTORY).resolve()
    bridge_path = (bridge_file or settings.FORKLIFT_BRIDGE_CONFIG_FILE).resolve()
    world_path = (test_world_file or settings.FORKLIFT_TEST_WORLD_FILE).resolve()
    model_dir.mkdir(parents=True, exist_ok=True)
    bridge_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.parent.mkdir(parents=True, exist_ok=True)

    sdf_path = model_dir / "model.sdf"
    config_path = model_dir / "model.config"
    sdf_path.write_text(build_model_sdf(), encoding="utf-8")
    config_path.write_text(build_model_config(), encoding="utf-8")
    bridge_path.write_text(build_bridge_yaml(), encoding="utf-8")
    world_path.write_text(build_test_world(), encoding="utf-8")

    # Parse generated XML immediately so malformed output fails at generation.
    ElementTree.parse(sdf_path)
    ElementTree.parse(config_path)
    ElementTree.parse(world_path)
    return sdf_path, config_path, bridge_path, world_path


def main() -> int:
    sdf_path, config_path, bridge_path, world_path = write_forklift_assets()
    print(f"Wrote {sdf_path}")
    print(f"Wrote {config_path}")
    print(f"Wrote {bridge_path}")
    print(f"Wrote {world_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
