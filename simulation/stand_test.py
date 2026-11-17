"""Run the five-second flat-ground standing experiment."""

from pathlib import Path

import mujoco
import numpy as np


def main():
    model_path = Path(__file__).with_name("biped.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.eq_active[model.equality("pelvis_support").id] = False

    # One gain per actuator, ordered by hip pitch, roll, yaw, knee,
    # ankle pitch, and ankle roll for the left leg, then the right.
    kp = 2.0 * np.tile([170.0, 160.0, 55.0, 90.0, 100.0, 100.0], 2)
    kd = np.sqrt(2.0) * np.tile([20.0, 18.0, 5.0, 7.0, 9.0, 9.0], 2)
    lower, upper = model.actuator_ctrlrange.T

    # The floating pelvis occupies 7 position and 6 velocity entries.
    for _ in range(round(5.0 / model.opt.timestep)):
        torque = -kp * data.qpos[7:] - kd * data.qvel[6:]
        data.ctrl[:] = np.clip(torque, lower, upper)
        mujoco.mj_step(model, data)

    floor_id = model.geom("floor").id
    pads = ("left_heel_tread", "left_forefoot_tread", "right_heel_tread", "right_forefoot_tread")
    loads = dict.fromkeys(pads, 0.0)
    wrench = np.zeros(6)
    for i, contact in enumerate(data.contact):
        first, second = contact.geom
        if floor_id in (first, second):
            pad = model.geom(second if first == floor_id else first).name
            if pad in loads:
                mujoco.mj_contactForce(model, data, i, wrench)
                loads[pad] += wrench[0]

    weight = mujoco.mj_getTotalmass(model) * np.linalg.norm(model.opt.gravity)
    print(f"Pelvis height after {data.time:.1f} s: {data.qpos[1]:.3f} m")
    for pad, load in loads.items():
        print(f"{pad}: {load:.1f} N")
    print(f"Total floor load: {sum(loads.values()):.1f} N (robot weight: {weight:.1f} N)")

    if data.qpos[1] < 0.69 or not all(load > 0.0 for load in loads.values()):
        raise SystemExit("Standing check failed.")


if __name__ == "__main__":
    main()
