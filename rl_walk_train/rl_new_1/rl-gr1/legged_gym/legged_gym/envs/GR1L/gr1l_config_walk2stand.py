# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from pickle import FALSE
from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
from legged_gym import LEGGED_GYM_ROOT_DIR
import os
import glob

frames_path = os.path.join(LEGGED_GYM_ROOT_DIR, 'datasets', 'motion_train', "*")
MOTION_FILES = (glob.glob(frames_path))


class GR1LCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_observations = 82  # 175
        num_actions = 20
        episode_length_s = 20  # episode length in seconds
        obs_profile = 'GR1L'  # full-osu, full, min-osu, min

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'trimesh'  # "heightfield" # none, plane, heightfield or trimesh
        measure_heights = False

        # 1mx1m rectangle (without center line)
        measured_points_x = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]

        # measured_points_x = [-0.1, -0.1, -0.05, -0.05, -0.0, 0., 0.0, 0.05, 0.05, 0.1, 0.1] # 1mx1m rectangle (without center line)
        # measured_points_y = [-0.1, -0.1, -0.05, -0.05, -0.0, 0., 0.0, 0.05, 0.05, 0.1, 0.1]

        max_init_terrain_level = 3  # starting curriculum state
        num_rows = 6  # number of terrain rows (levels)

    class commands(LeggedRobotCfg.commands):
        curriculum = False

        num_commands = 3  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        heading_command = False  # if true: compute ang vel command from heading error

        resampling_time = 10.  # time before command are changed[s]
        resample_command_profile = ["GR1L-stand"]
        resample_command_profile_randomize = False
        resample_command_log = False

        gait_cycle = 1.0

        # stand
        left_phase_ratio = 0.0
        right_phase_ratio = 0.0
        theta_left = 0.0
        theta_right = 0.0

        class ranges(LeggedRobotCfg.commands.ranges):
            lin_vel_x = [-0.0, 0.0]  # min max [m/s]
            lin_vel_y = [-0.0, 0.0]  # min max [m/s]
            ang_vel_yaw = [-0.0, 0.0]  # min max [rad/s]

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.95]  # x,y,z [m]
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            'hipRoll_Left': 0.0,
            'hipYaw_Left': 0.,
            'hipPitch_Left': -0.5236,
            'kneePitch_Left': 1.0472,
            'anklePitch_Left': -0.5236,
            'ankleRoll_Left': -0.0,

            'hipRoll_Right': -0.,
            'hipYaw_Right': 0.,
            'hipPitch_Right': -0.5236,
            'kneePitch_Right': 1.0472,
            'anklePitch_Right': -0.5236,
            'ankleRoll_Right': 0.0,

            'shoulderPitch_Left': 0.0,
            'shoulderRoll_Left': 0.,
            'shoulderYaw_Left': 0.0,  # 1.0,
            'elbow_Left': -0.3,

            'shoulderPitch_Right': 0.0,
            'shoulderRoll_Right': -0.,
            'shoulderYaw_Right': 0.0,  # 0.0,
            'elbow_Right': 0.3
        }

    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        stiffness = {
            'hipRoll': 328.5, 'hipYaw': 200.0, 'hipPitch': 300.,
            'kneePitch': 300.,
            'anklePitch': 10.0, 'ankleRoll': 0.,
            'shoulderPitch': 20.0, 'shoulderRoll': 10.0, 'shoulderYaw': 10.0,
            'elbow': 10.0
        }  # [N*m/rad]
        damping = {
            'hipRoll': 13.14, 'hipYaw': 4.0, 'hipPitch': 6.,
            'kneePitch': 6.,
            'anklePitch': 1.25, 'ankleRoll': 0.175,
            'shoulderPitch': 1.0, 'shoulderRoll': 1.0, 'shoulderYaw': 1.0,
            'elbow': 1.0
        }  # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 10

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/GR1L/urdf/GR1L.urdf'
        name = "GR1L"
        foot_name = 'toe'
        thigh_name = 'thigh'
        shin_name = 'shin'
        torso_name = 'pelvis'
        upper_arm_name = 'shoulderYaw'
        lower_arm_name = 'elbow'

        terminate_after_contacts_on = ['pelvis', 'thigh', 'shoulder', 'elbow']
        # terminate_after_contacts_on = []
        flip_visual_attachments = False
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True  # Jason: for learn to stand need to set to True
        friction_range = [0.3, 1.0]
        restitution_range = [0.0, 1.0]
        randomize_base_mass = True
        randomize_thigh_mass = True
        randomize_shin_mass = True
        randomize_torso_mass = True
        randomize_upper_arm_mass = True
        randomize_lower_arm_mass = True
        added_mass_range = [-0.05, 0.05]
        randomize_base_com = True
        added_com_range_x = [-0.05, 0.05]
        added_com_range_y = [-0.02, 0.02]
        added_com_range_z = [-0.05, 0.05]
        randomize_motor_strength = True
        motor_strength = [0.7, 1.4]
        push_robots = True
        push_interval_s = 3.5
        max_push_vel_xy = 0.5
        apply_forces = True
        continue_time_s = 0.5
        max_ex_forces = [-200.0, 200.0]
        max_ex_torques = [-0.0, 0.0]
        randomize_obs_linvel = True
        obs_linvel = [0.8, 1.2]

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.8
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.9
        max_contact_force = 500.
        only_positive_rewards = False
        base_height_target = 0.8
        feet_height_target = 0.2
        tracking_sigma = 0.25  # tracking reward = exp(-error^2/sigma)

        class scales(LeggedRobotCfg.rewards.scales):
            termination = 0.0
            tracking_lin_vel = 0.0
            tracking_ang_vel = 0.0
            lin_vel_z = 0.0
            ang_vel_xy = 0.0
            orientation = 0.0
            base_height = 0.0
            stand_still = -0.0
            feet_contact_forces = 0.0
            feet_air_time = 0.0
            dof_vel = -0.001  # -0.01  # -0.001
            dof_pos_limits = -0.0
            no_fly = 0.0

            collision = 0.0
            stumble = 0.0
            action_rate = 0.0
            feet_orien = 0.0
            feet_clearence = 0.0
            hip_yaw = 0.0
            symmetric_contact = 0.0
            # last_feet_airtime = 5.0
            torques = -25.e-6  # -30.e-6
            ankle_torques = -5.0e-6  # -0.005  # -0.001# -0.005
            knee_torques = -25.e-6  # -0.0001
            dof_acc = -1.e-7  # -1.e-6  # -1.e-7

            feet_frc_osu = 0.0
            feet_spd_osu = 0.0

            x_vel_diff_osu = 1.0
            y_vel_diff_osu = 0.5
            y_vel_lip_osu = 0.0
            z_vel_diff_osu = 0.2
            ang_vel_xy_osu = 0.1
            ang_vel_diff_osu = 0.4

            orientation_diff_osu = 0.2
            orientation_pitch_osu = -0.0
            base_height_diff_osu = 0.0
            feet_orien_diff_osu = 0.2

            action_diff_osu = 0.15
            ankle_action_diff_osu = 0.6
            upper_body_action_diff_osu = 1.0
            hip_action_diff_osu = 0.0
            knee_action_diff_osu = 0.0
            torque_diff_osu = 0.0
            hip_yaw_osu = 0.3
            feet_clearence_osu = 0.0
            dof_acc_diff_osu = 0.15
            dof_vel_diff_osu = 0.0
            dof_pos_limits_osu = 0.8

            feet_orien_osu = 0.0
            orientation_yaw_diff_osu = 0.0
            feet_contactforces_osu = 0.0

            swing_tracking = -0.0  # -0.1
            swing_height = 0.0  # 0.3
            swing_arm = 0.3
            swing_symmetric = 0.0
            feet_distance = 0.0
            feet_speed_diff_osu = 0.0

            arm_pose = -0.3
            torso_orientation_diff_osu = -1.0
            torso_ang_vel_xy_osu = -0.1

    class noise:
        add_noise = True
        noise_level = 0.1  # scales other values

        class noise_scales:
            dof_pos = 0.01
            dof_vel = 1.5
            lin_vel = 0.1
            ang_vel = 0.2
            gravity = 0.05
            height_measurements = 0.1

    class normalization(LeggedRobotCfg.normalization):
        class obs_scales(LeggedRobotCfg.normalization.obs_scales):
            lin_vel = 2.0

        clip_actions = 100.0

    class sim(LeggedRobotCfg.sim):
        dt = 0.001


class GR1LCfgPPO(LeggedRobotCfgPPO, GR1LCfg):
    runner_class_name = 'OnPolicyRunnerWithMirror'

    class runner(LeggedRobotCfgPPO.runner):
        algorithm_class_name = 'PPOWithMirror'
        policy_class_name = 'ActorCritic'

        experiment_name = 'GR1L'
        run_name = 'stand'
        num_steps_per_env = 64  # 100 # per iteration
        num_steps_prepare = 10  # must less than num_steps_per_env
        max_iterations = 5000  # number of policy updates
        save_interval = 200  # check for potential saves every this many iterations

        # load and resume
        resume = False
        load_run = -1
        checkpoint = -1  # 1000 # -1 = last saved model
        resume_path = None  # updated from load_run and chkpt

    class algorithm(LeggedRobotCfgPPO.algorithm):
        # training params
        num_learning_epochs = 8
        num_mini_batches = 40  # mini batch size = num_envs*nsteps / nminibatches
        learning_rate = 5.e-4  # 5.e-4

        mirror_profile = 'GR1L'
        mirror_coef = 1.0

    class policy(LeggedRobotCfgPPO.policy):
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        activation = 'elu'  # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
        actor_output_activation = None  # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
        critic_output_activation = None  # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
        init_noise_std = 1.0
