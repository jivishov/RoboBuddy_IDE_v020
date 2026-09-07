from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco

EXPECTED_MUJOCO_VERSION = "3.11.0"
EXPECTED_TIMESTEP_SECONDS = 0.002
MODEL_ID = "phase1-vertical-slice-v1"
MODEL_ASSET = "models/vertical-slice/model.xml"
ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "vertical-slice" / "model.xml"


def _name_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    index = int(mujoco.mj_name2id(model, obj_type, name))
    if index < 0:
        raise RuntimeError(f"Required MuJoCo object is missing: {obj_type.name}:{name}")
    return index


def _geom_name(model: mujoco.MjModel, geom_id: int) -> str | None:
    if geom_id < 0:
        return None
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def _contacts(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for contact in data.contact:
        if hasattr(contact, "geom"):
            geom1, geom2 = int(contact.geom[0]), int(contact.geom[1])
        else:
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
        result.append(
            {
                "geom1": geom1,
                "geom2": geom2,
                "geom1Name": _geom_name(model, geom1),
                "geom2Name": _geom_name(model, geom2),
                "distanceM": float(contact.dist),
            }
        )
    return result


def run(target_rad: float, steps: int, allow_version_mismatch: bool = False) -> dict[str, object]:
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(MODEL_PATH)
    if not allow_version_mismatch and mujoco.__version__ != EXPECTED_MUJOCO_VERSION:
        raise RuntimeError(
            f"Native reference requires MuJoCo {EXPECTED_MUJOCO_VERSION}, got {mujoco.__version__}. "
            "Use --allow-version-mismatch only for exploratory diagnostics, not conformance evidence."
        )
    if steps < 1 or steps > 100_000:
        raise ValueError("steps must be between 1 and 100000")

    model_sha256 = hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    hinge_id = _name_id(model, mujoco.mjtObj.mjOBJ_JOINT, "hinge")
    actuator_id = _name_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "hinge_position")
    free_box_body_id = _name_id(model, mujoco.mjtObj.mjOBJ_BODY, "free_box")

    qpos_addr = int(model.jnt_qposadr[hinge_id])
    dof_addr = int(model.jnt_dofadr[hinge_id])
    actuator_joint = int(model.actuator_trnid[actuator_id, 0])
    if actuator_joint != hinge_id:
        raise RuntimeError("hinge_position actuator is not mapped to hinge")

    control_min, control_max = map(float, model.actuator_ctrlrange[actuator_id])
    joint_min, joint_max = map(float, model.jnt_range[hinge_id])
    if not control_min <= target_rad <= control_max:
        raise ValueError(
            f"target_rad {target_rad} is outside actuator control range {control_min}..{control_max}"
        )
    if abs(float(model.opt.timestep) - EXPECTED_TIMESTEP_SECONDS) > 1e-12:
        raise RuntimeError(
            f"Unexpected timestep {model.opt.timestep}; expected {EXPECTED_TIMESTEP_SECONDS}"
        )

    data.ctrl[actuator_id] = target_rad
    for _ in range(steps):
        mujoco.mj_step(model, data)

    contacts = _contacts(model, data)
    return {
        "simulationTimeSeconds": float(data.time),
        "frames": {
            "world": {
                "id": "mujoco_world",
                "handedness": "right-handed",
                "upAxis": "+Z",
                "linearUnit": "m",
                "angularUnit": "rad",
            }
        },
        "model": {
            "id": MODEL_ID,
            "asset": MODEL_ASSET,
            "sha256": model_sha256,
        },
        "engine": {
            "name": "MuJoCo",
            "version": mujoco.__version__,
            "versionEvidence": "mujoco.__version__ native package introspection",
            "timestepSeconds": float(model.opt.timestep),
        },
        "joints": {
            "hinge": {
                "positionRad": float(data.qpos[qpos_addr]),
                "velocityRadS": float(data.qvel[dof_addr]),
                "targetRad": float(data.ctrl[actuator_id]),
                "controlRangeRad": [control_min, control_max],
                "jointRangeRad": [joint_min, joint_max],
            }
        },
        "bodies": {
            "free_box": {
                "frame": "mujoco_world",
                "positionM": [float(value) for value in data.xpos[free_box_body_id]],
            }
        },
        "contactCount": int(data.ncon),
        "contactsReadable": True,
        "contacts": contacts,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the RoboBuddy Phase 1 native MuJoCo reference scene.")
    parser.add_argument("--target", type=float, default=0.8, help="Hinge position target in radians.")
    parser.add_argument("--steps", type=int, default=500, help="Number of fixed MuJoCo steps.")
    parser.add_argument(
        "--allow-version-mismatch",
        action="store_true",
        help="Allow a native MuJoCo version other than 3.11.0 for exploratory diagnostics only.",
    )
    args = parser.parse_args()
    print(json.dumps(run(args.target, args.steps, args.allow_version_mismatch), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
