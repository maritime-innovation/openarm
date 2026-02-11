#!/usr/bin/env python3
"""
Minimal MuJoCo PD control for OpenArm (single or bimanual).
- Loads an MJCF (defaults to bimanual if available)
- Applies small sinusoidal PD position targets to each actuated joint
- Opens the MuJoCo viewer to visualize motion

Requirements: pip install mujoco glfw numpy
"""
import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import mujoco
from mujoco import viewer


def find_default_xml():
    # Prefer bimanual MJCF in the cloned repo under the workspace
    candidates = [
        Path(__file__).resolve().parent.parent / "openarm_mujoco" / "v1" / "openarm_bimanual.xml",
        Path(__file__).resolve().parent.parent / "openarm_mujoco" / "v1" / "openarm.xml",
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    return None


def build_model(xml_path: str):
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"MJCF not found: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    return model, data


def collect_joint_ids_for_actuators(model: mujoco.MjModel):
    # Map actuator index -> joint id (only for actuators targeting joints)
    joint_ids = []
    for i in range(model.nu):
        trn_type = model.actuator_trntype[i]
        if trn_type == mujoco.mjtTrn.mjTRN_JOINT:
            jid = model.actuator_trnid[i, 0]
        else:
            jid = -1
        joint_ids.append(jid)
    return np.array(joint_ids, dtype=int)


def main():
    parser = argparse.ArgumentParser(description="Minimal MuJoCo PD control for OpenArm")
    parser.add_argument("--xml", type=str, default=None, help="Path to OpenArm MJCF (XML)")
    parser.add_argument("--amp", type=float, default=0.1, help="target amplitude (radians)")
    parser.add_argument("--freq", type=float, default=0.2, help="target frequency (Hz)")
    parser.add_argument("--kp", type=float, default=20.0, help="PD proportional gain")
    parser.add_argument("--kd", type=float, default=1.0, help="PD derivative gain")
    parser.add_argument("--dt", type=float, default=1.0 / 240.0, help="simulation timestep for control loop")
    args = parser.parse_args()

    xml_path = args.xml or find_default_xml()
    if xml_path is None:
        print("Could not find default MJCF. Please supply --xml path to openarm_mujoco v1 XML.", file=sys.stderr)
        sys.exit(1)

    print(f"Loading MJCF: {xml_path}")
    model, data = build_model(xml_path)

    # Initial positions per joint (address by joint id via jnt_qposadr)
    qpos0 = data.qpos.copy()

    # Map actuators to joint IDs so we can do per-joint PD and write torque into ctrl
    act_joint_ids = collect_joint_ids_for_actuators(model)

    # Run with viewer (passive) so we step manually
    with viewer.launch_passive(model, data) as v:
        t0 = time.time()
        while v.is_running():
            t = time.time() - t0
            # Compute PD torque for each actuator targeting a single hinge joint
            for i in range(model.nu):
                jid = act_joint_ids[i]
                if jid < 0:
                    # Actuator not targeting a joint (e.g., tendon) — skip
                    data.ctrl[i] = 0.0
                    continue
                qadr = model.jnt_qposadr[jid]
                dadr = model.jnt_dofadr[jid]
                q = float(data.qpos[qadr])
                qd = float(data.qvel[dadr])
                target = float(qpos0[qadr] + args.amp * np.sin(2.0 * np.pi * args.freq * t))
                tau = args.kp * (target - q) - args.kd * qd
                data.ctrl[i] = tau

            # Step the simulation
            mujoco.mj_step(model, data)
            v.sync()
            # Sleep a bit to limit CPU usage and approximate realtime
            time.sleep(args.dt)


if __name__ == "__main__":
    main()
