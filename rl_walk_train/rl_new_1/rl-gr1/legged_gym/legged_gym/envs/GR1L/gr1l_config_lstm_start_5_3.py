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
        obs_profile = 'GR1L_new'  # full-osu, full, min-osu, min

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'trimesh'  # "heightfield" # none, plane, heightfield or trimesh
        measure_heights = False
        measured_points_x = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]  # 1mx1m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        # measured_points_x = [-0.1, -0.1, -0.05, -0.05, -0.0, 0., 0.0, 0.05, 0.05, 0.1, 0.1] # 1mx1m rectangle (without center line)
        # measured_points_y = [-0.1, -0.1, -0.05, -0.05, -0.0, 0., 0.0, 0.05, 0.05, 0.1, 0.1]
        max_init_terrain_level = 1  # starting curriculum state
        terrain_length = 12.
        terrain_width = 12.
        num_rows = 10  # number of terrain rows (levels)
        num_cols = 5  # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [0.4, 0.6, 0.0, 0.0, 0.0]
        # terrain_proportions = [0.2, 0.0, 0.4, 0.4, 0.0]
        # trimesh only:
        slope_treshold = 0.75  # slopes above this threshold will be corrected to vertical surfaces

    class commands(LeggedRobotCfg.commands):
        curriculum = False
        max_curriculum = 1.
        num_commands = 3  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10.  # time before command are changed[s]
        resample_command_profile = ["GR1L-walk"]
        resample_command_profile_randomize = False
        resample_command_log = True
        heading_command = False  # if true: compute ang vel command from heading error
        # left_phase_ratio = 0.45
        # right_phase_ratio = 0.45
        # theta_left = 0.45
        # theta_right = 0.95
        left_phase_ratio = 0.35
        right_phase_ratio = 0.35
        theta_left = 0.35
        theta_right = 0.85
        gait_cycle = 1.0
        gait_cycle_percent = 0.8
        clock_transition_time = 0.02

        class ranges(LeggedRobotCfg.commands.ranges):
            lin_vel_x = [-0.5, 1.5]  # min max [m/s]
            lin_vel_y = [-0.4, 0.4]  # min max [m/s]
            ang_vel_yaw = [-0.25, 0.25]  # min max [rad/s]
            # lin_vel_x = [-0.0, 0.0] # min max [m/s]
            # lin_vel_y = [-0.0, 0.0]   # min max [m/s]
            # ang_vel_yaw = [-0.0, 0.0]    # min max [rad/s]
            heading = [-3.14, 3.14]
            # reaching_x = [-0.5,0.5]
            # reaching_y = [0.2,0.5]
            # reaching_z = [0.2,0.6]
            reaching_x = [-0.0, 0.0]
            reaching_y = [0.0, 0.0]
            reaching_z = [0.0, 0.0]

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.9]  # x,y,z [m]
        # rot = [0, 0, 0.8509035, 0.525322]  # x,y,z,w [quat]
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            # left leg
            'l_hip_roll': 0.0,
            'l_hip_yaw': 0.0,
            'l_hip_pitch': -0.5236,
            'l_knee_pitch': 1.0472,
            'l_ankle_pitch': -0.5236,
            'l_ankle_roll': -0.0,

            # right leg
            'r_hip_roll': 0.0,
            'r_hip_yaw': 0.0,
            'r_hip_pitch': -0.5236,
            'r_knee_pitch': 1.0472,
            'r_ankle_pitch': -0.5236,
            'r_ankle_roll': 0.0,

            # left arm (Jason : make sure not easily collide with the leg or waist)
            'wl_shoulder_pitch': 0.0,
            'wl_shoulder_roll': 0.3,
            'wl_shoulder_yaw': 0.0,
            'wl_elbow_pitch': -0.3,

            # right arm (Jason : make sure not easily collide with the leg or waist)
            'wr_shoulder_pitch': 0.0,
            'wr_shoulder_roll': -0.3,
            'wr_shoulder_yaw': 0.0,
            'wr_elbow_pitch': -0.3,
        }

    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        # stiffness = {   'hipRoll': 200.0, 'hipYaw': 200.0,
        #                 'hipPitch': 300., 'kneePitch': 300., 'anklePitch': 2.0,
        #                 'ankleRoll': 0.}  # [N*m/rad]
        # damping = { 'hipRoll': 12.0, 'hipYaw': 12.0,
        #             'hipPitch': 6., 'kneePitch': 6., 'anklePitch': 2.,
        #             'ankleRoll': 0.28125}  # [N*m*s/rad]     # [N*m*s/rad]
        stiffness = {
            'hip_roll': 328.5, 'hip_yaw': 200.0, 'hip_pitch': 300.,
            'knee_pitch': 300.,
            'ankle_pitch': 10.0, 'ankle_roll': 1.0,
            'shoulder_pitch': 20.0, 'shoulder_roll': 10.0, 'shoulder_yaw': 10.0,
            'elbow_pitch': 10.0
        }  # [N*m/rad]
        damping = {
            'hip_roll': 13.14, 'hip_yaw': 4.0, 'hip_pitch': 6.,
            'knee_pitch': 6.,
            'ankle_pitch': 1.25, 'ankle_roll': 0.175,
            'shoulder_pitch': 1.0, 'shoulder_roll': 1.0, 'shoulder_yaw': 1.0,
            'elbow_pitch': 1.0
        }  # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 10

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/GR1L_new/urdf/GR1T1lite.urdf'
        name = "GR1T1lite"
        foot_name = 'foot_roll'
        thigh_name = 'thigh'
        shin_name = 'shank'
        torso_name = 'base'
        upper_arm_name = 'upper_arm'
        lower_arm_name = 'lower_arm'

        terminate_after_contacts_on = ['base', 'thigh', 'upper_arm', 'lower_arm']
        flip_visual_attachments = False
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.3, 1.0]
        restitution_range = [0.0, 1.0]
        randomize_base_mass = True
        randomize_thigh_mass = True
        randomize_shin_mass = True
        randomize_torso_mass = True
        randomize_upper_arm_mass = True
        randomize_lower_arm_mass = True
        added_mass_range = [-0.15, 0.15]
        randomize_base_com = True
        added_com_range_x = [-0.05, 0.05]
        added_com_range_y = [-0.03, 0.03]
        # added_com_range_z = [-0.05, 0.05]
        added_com_range_z = [-0.1, 0.2]
        randomize_motor_strength = True
        motor_strength = [0.7, 1.4]
        push_robots = True
        push_interval_s = 3.5
        max_push_vel_xy = 0.8
        apply_forces = True
        continue_time_s = 1.0
        max_ex_forces = [-300.0, 300.0]
        # max_ex_forces_foot = [-100.0, 100.0]
        # max_ex_forces_thigh = [-50, 50]
        # max_ex_torques = [-40.0, 40.0]
        max_ex_forces_foot = [-0.0, 0.0]
        max_ex_forces_thigh = [-0, 0]
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
        feetheight_target = 0.1
        tracking_sigma = 0.25  # tracking reward = exp(-error^2/sigma)

        class scales(LeggedRobotCfg.rewards.scales):
            # standard osu
            feet_frc_osu = 0.5#0.6#-0.4
            feet_spd_osu = 0.5#0.4#0.4#-0.4
            # probably donot need
            feet_frc_osu_positive = 0.0

            x_vel_diff_osu = -0.1
            x_vel_diff_osu_positive = 1.5
            y_vel_diff_osu = 0.0
            y_vel_diff_osu_positive = 0.5
            z_vel_diff_osu = 0.0
            z_vel_diff_osu_positive = 0.2
            ang_vel_diff_osu = 0.0
            ang_vel_diff_osu_positive = 0.4
            ang_vel_xy_osu_positive = 0.3#0.15#0.1

            orientation_diff_osu = 0.0
            orientation_diff_osu_positive = 0.4

            action_diff_osu = 0.0
            action_diff_osu_positive = 0.3#0.15
            torques_diff_osu = 0.0
            torques_diff_osu_strong = 0.0#-10.0e-5
            torque_limits_osu = 0.0
            torque_limits_osu_positive = 0.6##0.3
            ankle_torques_osu = 0.0#-0.01
            dof_acc_diff_osu = 0.0
            dof_acc_diff_osu_positive = 0.4
            dof_vel_diff_osu = 0.0 
            dof_vel_diff_osu_positive = 0.3
            # addition reward
            dof_pos_limits_osu = -0.8
            dof_pos_limits_osu_positive = 0.3
            swing_landing_vel = 0.0#-0.3
            swing_landing_vel_positive = 0.3
            swing_height_positive = 0.3
            swing_height_penalty = 0.0
             
            feet_distance_new = 0.0
            feet_distance_new_positive = 0.4##0.2
            

            feet_orien_diff_osu = 0.0
            feet_orien_diff_osu_positive = 0.2
            hipyaw_osu = 0.0
            hipyaw_osu_positive = 0.3
            bias = 0

            swing_arm = 0.3
            swing_symmetric = 0.2

            torques_osu = 0.0
            torques = -30.0e-6#-10.0e-6
            knee_torques = -10.e-5  # -0.0001
            knee_torques_shape = 0.0#-5.e-5#-1.e-5#-5.e-5
            dof_vel = 0.0#-0.001
            dof_acc = 0.0#-1.0e-7#
            dof_acc_hippitch = 0.0#-10.0e-6#-5.e-7
            # arm reward
            arm_pose = 0.0#-0.3
            upperbody_action_diff_osu = 0.0
            upperbody_action_diff_osu_positive = 1.0
            arm_torques_osu = 0.0
            arm_keepposture_penalty = 0.0
            arm_keepposture_positive = 0.1
            arm_torques = 0.0#-0.001
            arm_torque_limits_penalty = 0.0
            arm_torque_limits_positive = 0.4
            dof_vel_upperbody = 0.0
            dof_acc_upperbody = 0.0#-1.0e-6 #-1.e-6
            dof_vel_upperbody_osu = 0.0#-0.3
            dof_vel_upperbody_osu_positive = 0.3
            dof_acc_upperbody_osu = 0.0#-0.3
            dof_acc_upperbody_osu_positive = 0.3

            ankle_torque_limits_penalty = 0.0
            ankle_torque_limits_positive = 0.2
            ankle_action_diff_osu_penalty = 0.0#
            ankle_action_diff_osu_positive = 0.6#

            ankle_torques = 0.0#-5.e-6#-0.005  # -0.001# -0.005
            ankle_torques_osu_positive = 0.2
            ankle_roll_torque_limits_penalty = 0.0#-0.2
            ankle_roll_torque_limits_positive = 0.0#0.2
            ankle_roll_actions_penalty = 0.0#-0.00002
            ankle_roll_actions_positive = 0.2#0.2
            arm_pose_shoulder_pitch = 0.0#-0.1
            ankle_roll_torques = 0.0#-25.e-5
            torso_orientation_diff_osu = 0.0#-0.1

            contact_force_diff = 0.0
            feet_speed_close_to_ground_positive = 0.4
            feet_double_fly_penalty = 0.0#-0.2
    class noise:
        add_noise = True
        noise_level = 0.1  # scales other values

        class noise_scales:
            dof_pos = 0.01
            dof_vel = 1.5
            lin_vel = 0.1#0.1
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
    runner_class_name = 'OnPolicyRunner_lstm'
    class policy(LeggedRobotCfgPPO.policy):
        fixed_std = False
        init_noise_std = 1.0
        actor_hidden_dims = [128,64]
        critic_hidden_dims = [128,64]
        actor_hidden_dims_pre = []
        critic_hidden_dims_pre = []
        activation = 'elu'  # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
        # only for 'ActorCriticRecurrent':
        rnn_type = 'lstm'
        rnn_hidden_size = 256
        rnn_num_layers = 1

    class runner(LeggedRobotCfgPPO.runner):
        policy_class_name = 'ActorCriticRecurrent'
        algorithm_class_name = 'PPO_lstm_T1'
        num_steps_per_env = 200  # 100 # per iteration
        # policy_class_name = 'ActorCritic'
        run_name = ''
        experiment_name = 'GR1L_lstm_start5_3'
        max_iterations = 50000  # number of policy updates
        save_interval = 100  # check for potential saves every this many iterations
        # load and resume
        resume = False
        # "_Dec05_15-30-19": 
        # 上身左右摇晃幅度太大，
        # ang_vel_xy_osu_positive alpha from -1.0 to -5.0; 
        # orientation_diff_osu_positive alpha from -10.0 to -20.0;
        # 滑步，
        # add feet_speed_close_to_ground_positive = 0.4
        # z向速度波动大
        # z_vel_diff_osu_positive alpha from -2.0 to -20.0
        # 脚踝抖动较大，
        # ankle_action_diff_osu_positive alpha from -0.05 to -0.5
        # x 速度跟随较差
        # x_vel_diff_osu_positive alpha from -10.0 to -20.0

        # "_Dec06_14-57-46": 需要惩罚双脚离地
        # "_Dec06_21-47-16": 需要惩罚双脚离地, hipyaw固定，增加dofacc dof vel
        load_run = -1#"_Dec06_10-36-19"
        checkpoint = -1# 1000 # -1 = last saved model
        resume_path = None  # updated from load_run and chkpt

    class algorithm(LeggedRobotCfgPPO.algorithm):
        # training params
        num_learning_epochs = 8
        num_mini_batches = 40  # mini batch size = num_envs*nsteps / nminibatches
        learning_rate = 5.e-4  # 5.e-4
        # mirror_profile = 'GR1L'
        # mirror_coef = [1.0]