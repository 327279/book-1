from omni.isaac.core.utils.prims import randomize_light_properties
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

def randomize_scene_lights():
    """
    Randomizes lighting conditions in the current Isaac Sim stage.
    """
    randomize_light_properties(
        "/World/Light",
        color_mean=[0.8, 0.8, 0.8],
        color_sigma=[0.2, 0.2, 0.2],
        intensity_mean=1000,
        intensity_sigma=200
    )
    print("Lights randomized.")

if __name__ == "__main__":
    randomize_scene_lights()
