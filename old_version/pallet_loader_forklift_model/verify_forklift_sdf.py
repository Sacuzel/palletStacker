"""Independent structural checks for the generated forklift SDF.

This checker does not launch Gazebo. It verifies the exported XML contract:
three physical links, two vertical fork joints, configured controller topics,
the hard fork-height limit, GUI teleoperation configuration, and the 2 m spawn
clearance measured from fork tips to the open face of pallet 1.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import config
from gazebo_exporter import create_world_sdf, forklift_spawn_pose
from pallet import Pallet


def _floats(text: str | None) -> list[float]:
    if text is None:
        raise AssertionError("Expected numeric XML text, got None")
    return [float(value) for value in text.split()]


def main() -> None:
    pallet = Pallet("P01")
    sdf, _manifest = create_world_sdf([pallet], start_paused=True)
    root = ET.fromstring(sdf)
    world = root.find("world")
    assert world is not None, "Generated SDF has no world"

    model = world.find(f"./model[@name='{config.FORKLIFT_MODEL_NAME}']")
    assert model is not None, "Forklift model is missing"

    links = model.findall("link")
    joints = model.findall("joint")
    assert [link.get("name") for link in links] == [
        "base_link",
        "left_fork_link",
        "right_fork_link",
    ], "Forklift must contain exactly the body and two forks"
    assert [joint.get("name") for joint in joints] == [
        "left_fork_joint",
        "right_fork_joint",
    ], "Forklift must contain exactly two fork joints"

    expected_travel = (
        config.FORKLIFT_BODY_HEIGHT_M
        - config.FORKLIFT_INITIAL_FORK_BOTTOM_M
        - config.FORKLIFT_FORK_THICKNESS_M
    )
    for joint in joints:
        assert joint.get("type") == "prismatic"
        assert _floats(joint.findtext("axis/xyz")) == [0.0, 0.0, 1.0]
        upper = float(joint.findtext("axis/limit/upper", "nan"))
        assert math.isclose(upper, expected_travel, abs_tol=1e-9)

    final_fork_top = (
        config.FORKLIFT_INITIAL_FORK_BOTTOM_M
        + config.FORKLIFT_FORK_THICKNESS_M
        + expected_travel
    )
    assert math.isclose(
        final_fork_top, config.FORKLIFT_BODY_HEIGHT_M, abs_tol=1e-9
    ), "Fork upper face exceeds the body upper face"

    controller_topics = {
        plugin.findtext("topic")
        for plugin in model.findall("plugin")
        if plugin.get("name", "").endswith("JointController")
    }
    assert controller_topics == {
        config.FORKLIFT_LEFT_FORK_TOPIC,
        config.FORKLIFT_RIGHT_FORK_TOPIC,
    }

    gui_plugin = world.find(
        f"./gui/plugin[@filename='{config.FORKLIFT_GUI_PLUGIN_FILENAME}']"
    )
    assert gui_plugin is not None, "Forklift GUI plugin is missing"
    assert gui_plugin.findtext("forward_key") == str(
        config.FORKLIFT_KEY_BINDINGS["forward"]
    )
    assert gui_plugin.findtext("lift_key") == str(
        config.FORKLIFT_KEY_BINDINGS["lift"]
    )
    assert gui_plugin.findtext("world_stats_topic") == config.FORKLIFT_WORLD_STATS_TOPIC

    x, y, _z, _r, _p, yaw = forklift_spawn_pose()
    forward = (math.cos(yaw), math.sin(yaw))
    tip_offset = (
        config.FORKLIFT_BODY_LENGTH_M / 2.0 + config.FORKLIFT_FORK_LENGTH_M
    )
    tip_x = x + tip_offset * forward[0]
    tip_y = y + tip_offset * forward[1]
    pallet_face_x = config.PALLET_LENGTH * 0.001 / 2.0
    pallet_face_y = 0.0
    distance = math.hypot(tip_x - pallet_face_x, tip_y - pallet_face_y)
    assert math.isclose(distance, config.FORKLIFT_SPAWN_DISTANCE_M, abs_tol=1e-9)

    print("Forklift SDF verification passed.")
    print(f"  links                 : {len(links)}")
    print(f"  prismatic joints      : {len(joints)}")
    print(f"  maximum fork travel   : {expected_travel:.3f} m")
    print(f"  maximum fork top      : {final_fork_top:.3f} m")
    print(f"  fork-tip spawn gap    : {distance:.3f} m")
    print(f"  maximum drive speed   : {config.FORKLIFT_MAX_SPEED_M_S:.3f} m/s")
    print(f"  maximum acceleration  : {config.FORKLIFT_MAX_ACCEL_M_S2:.3f} m/s^2")


if __name__ == "__main__":
    main()
