from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
import numpy as np


class FlamingoRoughCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        # TODO: Change the num obs
        # num_observations = 163
        num_observations = 42
        num_actions = 10

    class terrain(LeggedRobotCfg.terrain):
        # measured_points_x = [
        #     -0.5,
        #     -0.4,
        #     -0.3,
        #     -0.2,
        #     -0.1,
        #     0.0,
        #     0.1,
        #     0.2,
        #     0.3,
        #     0.4,
        #     0.5,
        # ]  # 1mx1m rectangle (without center line)
        # measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        mesh_type = "plane"
        measure_heights = False

    class init_state(LeggedRobotCfg.init_state):
        # pos = [0.0, 0.0, 1.0]  # x,y,z [m]
        pos = [0.0, 0.0, 0.5]  # x,y,z [m]
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            "hip_roll_l": 0,
            "hip_pitch_l": 0,
            "knee_l": 0,
            "ankle_l": 0,
            "feet_l": 0,
            "hip_roll_r": 0,
            "hip_pitch_r": 0,
            "knee_r": 0,
            "ankle_r": 0,
            "feet_r": 0,
        }

    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        names = [
            "hip_roll",
            "hip_pitch",
            "knee",
            "ankle",
            "feet",
        ]
        stiffness_b = np.array([100.0, 100.0, 200.0, 200.0, 40.0])  # [N*m/rad]
        damping_b = np.array([3.0, 3.0, 6.0, 6.0, 1.0])  # [N*m*s/rad]     # [N*m*s/rad]

        # scale the PD values
        stiffness_scale = 1e-2
        damping_scale = 1e-2

        stiffness = dict(zip(names, stiffness_b * stiffness_scale))
        damping = dict(zip(names, damping_b * damping_scale))

        # action scale: target angle = actionScale * action + defaultAngle
        # action_scale = 0.5
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/flamingo/urdf/flamingo.urdf"
        name = "flamingo"
        foot_name = "feet"
        terminate_after_contacts_on = [
            "base_link",
            "hip-roll-l_1",
            "hip-link-l_1",
            "hip-roll-r_1",
            "hip-link-r_1",
            # "thigh-l_1",
            # "thigh-r_1",
            # "ankle-l_1",
            # "ankle-r_1",
        ]
        flip_visual_attachments = False
        self_collisions = 1  # 1 to disable, 0 to enable...bitwise filter
        max_angular_velocity = 50.0
        max_linear_velocity = 50.0

    class rewards(LeggedRobotCfg.rewards):
        # soft_dof_pos_limit = 0.95
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.9
        max_contact_force = 300.0
        only_positive_rewards = False
        # from a1 config
        # base_height_target = 0.25

        class scales(LeggedRobotCfg.rewards.scales):
            termination = -200.0
            tracking_ang_vel = 1.0
            torques = -5.0e-6
            dof_acc = -2.0e-7
            lin_vel_z = -0.5
            feet_air_time = 5.0
            dof_pos_limits = -1.0
            no_fly = 0.25
            dof_vel = -0.0
            ang_vel_xy = -0.0
            feet_contact_forces = -0.0


class FlamingoRoughCfgPPO(LeggedRobotCfgPPO):
    class runner(LeggedRobotCfgPPO.runner):
        run_name = ""
        experiment_name = "rough_flamingo"

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
