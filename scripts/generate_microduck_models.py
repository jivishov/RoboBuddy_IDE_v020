#!/usr/bin/env python3
"""Generate the pinned MicroDuck Phase 5C physical MuJoCo assets.

The articulated hierarchy - body names, transforms, joint axes, joint ranges,
inertials and named sites - is read verbatim from the Apache-2.0 source model this
repository already distributes:

  assets/microduck/kinematics/robot_walk.xml
  (pollen-robotics/microduck@590b986bd8c0d50ae02cb3ea2f59c463b6828168,
   kinematics/assets/alpha/robot_walk.xml)

That file carries no collision geometry and no actuators, so those two layers come
from the reconciled RL/training environment:

  pollen-robotics/microduck_rl@519142bd2f30ff8bbd7b6a6b03e33a4f31d9dc2f
  (robot_allcollisions.xml, joints_properties.xml, scene.xml, ball.xml)

whose robot_walk.xml is structurally identical to the deployed one - same 37 named
bodies/joints/sites with identical pos/quat/axis/range, and the same 14 inertials.
docs/physics/microduck-provenance.md records the reconciliation in full.

The upstream collision layer is a set of CC BY-SA-NC mesh colliders, which this
repository does not redistribute. Every collider below is a repository-authored
primitive fitted to the corresponding source mesh envelope, measured from the
compiled upstream model. The two soles - the only load-bearing walking contact -
are fitted to the actual sole contact face rather than to its bounding box, so the
support polygon is not silently enlarged.

Run:  python scripts/generate_microduck_models.py            # write
      python scripts/generate_microduck_models.py --check    # CI byte-check
"""
from __future__ import annotations

import argparse
import hashlib
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SOURCE_XML = REPO / "assets" / "microduck" / "kinematics" / "robot_walk.xml"
OUT = REPO / "models" / "microduck"

# --- reconciled simulation settings (microduck_rl@519142b scene.xml + infer_policy.py) ---
TIMESTEP = 0.005            # infer_policy.py: model.opt.timestep = 0.005
INTEGRATOR = "Euler"        # MuJoCo default, unchanged by the source scene
SOLVER_ITERATIONS = 100     # MuJoCo default, unchanged by the source scene
LS_ITERATIONS = 50          # MuJoCo default, unchanged by the source scene

# --- matched actuator contract (joints_properties.xml, class "chosen_actuator") ---
# An identified XL330 servo model at the firmware position gain the daemon ships
# (robotd Tuning::gain = 200), not a generic PD guess.
ACT_KP = 0.55               # N m / rad at firmware kp 200
ACT_KV = 0.0
ACT_FORCE_NM = 0.96         # identified torque envelope
ACT_CTRL_RAD = 10.0
JOINT_DAMPING = 0.053
JOINT_FRICTIONLOSS = 0.0048
JOINT_ARMATURE = 0.0018

# --- contact contract, as the upstream CPU reference runner compiles it ---
FLOOR_FRICTION = (1.0, 0.005, 0.0001)
LOW_TRACTION_FRICTION = (0.02, 0.005, 0.0001)
CONDIM = 3
SOLREF = (0.02, 1.0)

# --- source ball prop (microduck_rl@519142b ball.xml) ---
BALL_RADIUS = 0.035
BALL_MASS = 0.015
BALL_INERTIA = 1.225e-5
BALL_FRICTION = (0.5, 0.005, 0.0001)
# microduck_ball_kick_env_cfg.py BALL_OFFSET, in the robot yaw frame, right foot.
BALL_OFFSET_X = 0.09
BALL_OFFSET_Y = 0.042

FLOOR_HALF_EXTENT = 4.0

# --- collision primitives fitted to the compiled upstream mesh envelopes -------
# (body, geom name, half-extents, centre, quat, self-collision-only)
# The geom quat maps the box's own frame INTO the parent body frame, so it is the INVERSE of
# the rotation that takes body-frame points into the frame the fit was measured in. Getting
# that sign wrong stands the robot on a sole edge at twice the angle instead of flat on its
# foot, which looks like a gait-tuning problem rather than a frame one.
# Boxes are axis-aligned bounding fits of the source collision mesh unless a quat is
# given. The two soles carry a 4.75 deg roll about the ankle x axis, which is the
# plane the source sole face actually lies in, and their footprint is the measured
# contact face (45.6 x 34.0 mm), not the mesh bounding box (54.0 x 41.2 mm).
SOLE_HALF = (0.022784, 0.003000, 0.016975)
SOLE_ROLL_RAD = math.radians(4.75)
_SR = (math.cos(SOLE_ROLL_RAD / 2), math.sin(SOLE_ROLL_RAD / 2))

COLLIDERS = (
    # trunk
    ("trunk_base", "trunk_shell_right", (0.041215, 0.016750, 0.017350), (-0.006405, -0.014250, 0.024715), None, False),
    ("trunk_base", "trunk_shell_left", (0.041215, 0.015450, 0.017850), (-0.006405, 0.015550, 0.025215), None, False),
    ("trunk_base", "battery_pack", (0.010875, 0.019305, 0.035525), (-0.035955, -0.000005, -0.003855), None, False),
    ("trunk_base", "power_support", (0.008500, 0.027250, 0.041745), (-0.022500, 0.000000, -0.002325), None, True),
    # head (all three source head colliders hang off bottom_head_shell)
    ("bottom_head_shell", "head_shell_lower", (0.010070, 0.045880, 0.058375), (-0.007070, 0.000000, -0.019875), None, False),
    ("bottom_head_shell", "head_shell_upper", (0.023165, 0.045880, 0.061345), (0.018665, 0.000000, -0.022845), None, False),
    ("bottom_head_shell", "jaw", (0.014725, 0.045710, 0.034355), (-0.001765, 0.000000, -0.043355), None, False),
    # left leg
    ("hip_l", "left_hip", (0.016225, 0.009500, 0.017250), (0.006725, 0.000000, -0.010750), None, False),
    ("left_upper_leg", "left_thigh", (0.023830, 0.030500, 0.014000), (0.010360, 0.018300, 0.014000), None, False),
    ("leg", "left_shank", (0.010000, 0.029000, 0.003975), (0.000000, 0.019000, 0.002025), None, False),
    ("ankle_left", "left_foot_collision", SOLE_HALF, (-0.006993, -0.020873, -0.014521), (_SR[0], -_SR[1], 0.0, 0.0), False),
    # right leg
    ("hip_l_2", "right_hip", (0.016225, 0.009500, 0.017250), (0.006725, 0.000000, -0.010750), None, False),
    ("right_upper_leg", "right_thigh", (0.023830, 0.030500, 0.014000), (-0.010360, 0.018300, 0.014000), None, False),
    ("leg_2", "right_shank", (0.029000, 0.010000, 0.003975), (-0.019000, 0.000000, 0.002025), None, False),
    ("ankle_right", "right_foot_collision", SOLE_HALF, (-0.006993, 0.020873, -0.014515), (_SR[0], _SR[1], 0.0, 0.0), False),
)

POLICY_JOINTS = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)

# HOME pose. duck-control model.rs DEFAULT_POSITION with the mouth slot removed, which is
# byte-identical to microduck_rl HOME_FRAME and to the source scene.xml STAND keyframe.
HOME_RAD = (
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
)
TRUNK_SPAWN_Z = 0.12  # source robot_walk.xml trunk_base pos and scene.xml keyframes


def num(value: float) -> str:
    text = f"{value:.9g}"
    return "0" if text in ("-0", "0") else text


def triple(values) -> str:
    return " ".join(num(v) for v in values)


def source_body_tree() -> str:
    """The exact articulated hierarchy from the pinned Apache-2.0 source model."""
    text = SOURCE_XML.read_text()
    match = re.search(r"<worldbody>(.*)</worldbody>", text, re.S)
    if not match:
        raise SystemExit(f"{SOURCE_XML}: no <worldbody>")
    return match.group(1)


def with_colliders(tree: str) -> str:
    """Insert the fitted collision primitives into the source hierarchy."""
    by_body: dict[str, list[str]] = {}
    for body, name, half, pos, quat, self_only in COLLIDERS:
        cls = "self_contact" if self_only else "duck_contact"
        attrs = f'name="{name}" class="{cls}" type="box" size="{triple(half)}" pos="{triple(pos)}"'
        if quat is not None:
            attrs += f' quat="{triple(quat)}"'
        by_body.setdefault(body, []).append(f"<geom {attrs} mass=\"0\"/>")
    out = tree
    for body, geoms in by_body.items():
        pattern = re.compile(rf'(<body name="{re.escape(body)}"[^>]*>)')
        if not pattern.search(out):
            raise SystemExit(f"source hierarchy has no body {body}")
        indent = "\n        "
        out = pattern.sub(lambda m: m.group(1) + indent + indent.join(geoms), out, count=1)
    return out


def joint_defaults() -> str:
    return (
        f'    <default class="duck_servo">\n'
        f'      <joint damping="{num(JOINT_DAMPING)}" frictionloss="{num(JOINT_FRICTIONLOSS)}" armature="{num(JOINT_ARMATURE)}"/>\n'
        f'      <position kp="{num(ACT_KP)}" kv="{num(ACT_KV)}" forcerange="-{num(ACT_FORCE_NM)} {num(ACT_FORCE_NM)}"'
        f' ctrlrange="-{num(ACT_CTRL_RAD)} {num(ACT_CTRL_RAD)}"/>\n'
        f'    </default>\n'
    )


def model_xml(name: str, floor_friction, *, ball: bool) -> str:
    tree = with_colliders(source_body_tree())
    # The source hierarchy declares its hinges without a class; attach the matched
    # servo class to exactly the fourteen policy joints.
    for joint in POLICY_JOINTS:
        tree = tree.replace(f'name="{joint}" type="hinge"', f'name="{joint}" class="duck_servo" type="hinge"')
    tree = tree.replace('\n    ', '\n      ').replace('\n  ', '\n    ')

    actuators = "\n".join(
        f'    <position name="act_{j}" class="duck_servo" joint="{j}"/>' for j in POLICY_JOINTS
    )
    fric = triple(floor_friction)
    ball_body = ""
    ball_key = ""
    if ball:
        ball_body = (
            f'\n    <body name="microduck_ball" pos="{num(BALL_OFFSET_X)} -{num(BALL_OFFSET_Y)} {num(BALL_RADIUS)}">'
            f'\n      <freejoint name="microduck_ball_free"/>'
            f'\n      <inertial pos="0 0 0" mass="{num(BALL_MASS)}" diaginertia="{num(BALL_INERTIA)} {num(BALL_INERTIA)} {num(BALL_INERTIA)}"/>'
            f'\n      <geom name="microduck_ball_geom" type="sphere" size="{num(BALL_RADIUS)}"'
            f' condim="{CONDIM}" friction="{triple(BALL_FRICTION)}" solref="{triple(SOLREF)}" rgba="1 0.55 0 1"/>'
            f'\n    </body>'
        )
        ball_key = f" {num(BALL_OFFSET_X)} -{num(BALL_OFFSET_Y)} {num(BALL_RADIUS)} 1 0 0 0"

    home = " ".join(num(v) for v in HOME_RAD)
    keyframe = (
        f'  <keyframe>\n'
        f'    <key name="HOME" qpos="0 0 {num(TRUNK_SPAWN_Z)} 1 0 0 0 {home}{ball_key}" ctrl="{home}"/>\n'
        f'  </keyframe>\n'
    )

    return (
        f'<mujoco model="{name}">\n'
        f'  <compiler angle="radian" autolimits="true"/>\n'
        f'  <option integrator="{INTEGRATOR}" timestep="{num(TIMESTEP)}" iterations="{SOLVER_ITERATIONS}"'
        f' ls_iterations="{LS_ITERATIONS}" gravity="0 0 -9.81"/>\n'
        f'  <default>\n'
        f'{joint_defaults()}'
        f'    <default class="duck_contact">\n'
        f'      <geom group="3" contype="1" conaffinity="1" condim="{CONDIM}" friction="{fric}"'
        f' solref="{triple(SOLREF)}" rgba="0.85 0.72 0.25 1"/>\n'
        f'    </default>\n'
        f'    <default class="self_contact">\n'
        f'      <geom group="3" contype="2" conaffinity="2" condim="{CONDIM}" friction="{fric}"'
        f' solref="{triple(SOLREF)}" rgba="0.55 0.57 0.60 1"/>\n'
        f'    </default>\n'
        f'    <default class="duck_floor">\n'
        f'      <geom type="plane" contype="1" conaffinity="1" condim="{CONDIM}" friction="{fric}"'
        f' solref="{triple(SOLREF)}" rgba="0.38 0.42 0.46 1"/>\n'
        f'    </default>\n'
        f'  </default>\n'
        f'  <worldbody>\n'
        f'    <light pos="0 0 2" dir="0 0 -1"/>\n'
        f'    <geom name="microduck_floor" class="duck_floor" size="{num(FLOOR_HALF_EXTENT)} {num(FLOOR_HALF_EXTENT)} 0.05"/>'
        f'{tree}'
        f'{ball_body}\n'
        f'  </worldbody>\n'
        f'  <sensor>\n'
        f'    <gyro name="imu_ang_vel" site="imu"/>\n'
        f'    <accelerometer name="imu_accel" site="imu"/>\n'
        f'    <framequat name="imu_orientation" objtype="site" objname="imu"/>\n'
        f'  </sensor>\n'
        f'  <actuator>\n'
        f'{actuators}\n'
        f'  </actuator>\n'
        f'{keyframe}'
        f'</mujoco>\n'
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify the committed assets are byte-identical")
    args = parser.parse_args()

    assets = {
        "walk.xml": model_xml("robobuddy_microduck_walk", FLOOR_FRICTION, ball=False),
        "walk_lowtraction.xml": model_xml("robobuddy_microduck_walk_lowtraction", LOW_TRACTION_FRICTION, ball=False),
        "kick.xml": model_xml("robobuddy_microduck_kick", FLOOR_FRICTION, ball=True),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    failures = []
    for name, text in assets.items():
        path = OUT / name
        digest = hashlib.sha256(text.encode()).hexdigest()
        if args.check:
            current = path.read_text() if path.exists() else None
            ok = current == text
            if not ok:
                failures.append(name)
            print(f"{name}: sha256 {digest} {'OK' if ok else 'DIFFERS'}")
        else:
            path.write_text(text)
            print(f"{name}: sha256 {digest}")
    if failures:
        print(f"Regenerated MicroDuck assets differ from the committed files: {', '.join(failures)}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
