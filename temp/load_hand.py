import time

import mujoco
from robot_descriptions.loaders.mujoco import load_robot_description

# Load the Psyonic Ability hand description
# hand_description = "ability_hand_description"
hand_description = "ability_hand_mj_description"
try:
    model = load_robot_description(hand_description)
except ImportError as e:
    print(f"Error loading robot description: {e}")
    exit()

# Create a data instance
data = mujoco.MjData(model)

# Create a viewer
viewer = mujoco.viewer.launch_passive(model, data)

# Simulation loop
try:
    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(model, data)
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
except KeyboardInterrupt:
    print("Simulation stopped.")

viewer.close()
