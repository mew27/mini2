"""Configuration for a xgo-mini2 quadruped robot."""


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

MINI2_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="mini2_description/usd/xgomini.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    # init_state=ArticulationCfg.InitialStateCfg(
    #     pos=(0.0, 0.0, 2.0), joint_pos={"slider_to_cart": 0.0, "cart_to_pole": 0.0}
    # ),
    actuators={
        "cart_actuator": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit=0.441,
            velocity_limit=10.0,
            stiffness=80.0,
            damping=20.0,
        ),
    },
)
"""Configuration for a xgo-mini2 quadruped robot."""
