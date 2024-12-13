
"""This script demonstrates how to spawn a cart-pole and interact with it.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/tutorials/01_assets/run_articulation.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse
import math 
from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Spawning xgomini2")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext

##
# Pre-defined configs
##
from mini2 import MINI2_CFG # isort:skip

NUM_ROBOTS_X = 16
NUM_ROBOTS_Y = 16
SPACING      = 1

def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=1000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    #Create grid of robots
    origins = [[SPACING * x, SPACING *y, 0.3] for x in range(NUM_ROBOTS_X) for y in range(NUM_ROBOTS_Y)]

    for idx, origin in enumerate(origins):
        prim_utils.create_prim(f"/World/Origin{idx}", "Xform", translation=origin)


    # Articulation
    mini2_cfg = MINI2_CFG.copy()
    mini2_cfg.prim_path = "/World/Origin.*/Robot"
    mini2 = Articulation(cfg=mini2_cfg)

    # return the scene information
    scene_entities = {"mini2": mini2}
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    robot = entities["mini2"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 2500 == 0:
            # reset counter
            count = 0
            # reset the scene entities
            # root state
            # we offset the root state by the origin since the states are written in simulation world frame
            # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += origins
            robot.write_root_state_to_sim(root_state)
            # set joint positions with some noise
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            #joint_pos += torch.rand_like(joint_pos) * 5
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            robot.reset()
            print("[INFO]: Resetting robot state...")
            
        # Apply random action
        # -- generate random joint efforts
        # if count % 50 == 0:
        #     positions += torch.randn_like(robot.data.joint_pos) / 10
        #     # -- apply action to the robot
        #     #robot.set_joint_effort_target(efforts)
        #     robot.set_joint_position_target(positions)

        if count % 15 == 0:
            positions = torch.randn_like(robot.data.joint_pos) / 2
            robot.set_joint_position_target(positions)

        # -- write data to sim
        robot.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        # robot.update(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    #sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    sim.set_camera_view([13.55, 2.3, 0.38], [13, 3, 0.2])
    # Design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
