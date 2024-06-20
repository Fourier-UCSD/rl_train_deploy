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
        measured_points_x = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]  # 1mx1m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        # measured_points_x = [-0.1, -0.1, -0.05, -0.05, -0.0, 0., 0.0, 0.05, 0.05, 0.1, 0.1] # 1mx1m rectangle (without center line)
        # measured_points_y = [-0.1, -0.1, -0.05, -0.05, -0.0, 0., 0.0, 0.05, 0.05, 0.1, 0.1]
        max_init_terrain_level = 1  # starting curriculum state
        terrain_length = 12.
        terrain_width = 12.
        num_rows = 3  # number of terrain rows (levels)
        num_cols = 2  # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [0.6, 0.4, 0.0, 0.0, 0.0]
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
        left_phase_ratio = 0.35#0.45
        right_phase_ratio = 0.35#0.45
        theta_left = 0.35#0.45
        theta_right = 0.85#0.95
        gait_cycle = 1.0
        gait_cycle_percent = 0.8
        clock_transition_time = 0.02

        class ranges(LeggedRobotCfg.commands.ranges):
            lin_vel_x = [-0.5, 1.0]  # min max [m/s]
            lin_vel_y = [-0.25, 0.25]  # min max [m/s]
            ang_vel_yaw = [-0.2, 0.2]  # min max [rad/s]
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
            'shoulderRoll_Left': 0.2,
            'shoulderYaw_Left': 0.0,  # 1.0,
            'elbow_Left': -0.3,

            'shoulderPitch_Right': 0.0,
            'shoulderRoll_Right': -0.2,
            'shoulderYaw_Right': 0.0,  # 0.0,
            'elbow_Right': 0.3
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
            'hipRoll': 328.5, 'hipYaw': 200.0, 'hipPitch': 300.,
            'kneePitch': 300.,
            'anklePitch': 10.0, 'ankleRoll': 1.0,
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
        flip_visual_attachments = False
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.3, 1.5]
        restitution_range = [0.0, 1.0]
        randomize_base_mass = True
        randomize_thigh_mass = True
        randomize_shin_mass = True
        randomize_torso_mass = True
        randomize_upper_arm_mass = True
        randomize_lower_arm_mass = True
        added_mass_range = [-0.3, 0.3]
        randomize_base_com = True
        added_com_range_x = [-0.1, 0.1]
        # added_com_range_y = [-0.05, 0.05]
        added_com_range_y = [-0.1, 0.1]
        # added_com_range_z = [-0.05, 0.05]
        added_com_range_z = [-0.2, 0.2]
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
        feetheight_target = 0.2
        tracking_sigma = 0.25  # tracking reward = exp(-error^2/sigma)

        class scales(LeggedRobotCfg.rewards.scales):
            # standard osu
            feet_frc_osu = 0.4
            
            feet_spd_osu = -0.4
            feet_frc_osu_positive = 0.8

            x_vel_diff_osu = -0.8#0.3
            x_vel_diff_osu_positive = 0.8
            y_vel_diff_osu = -0.3
            z_vel_diff_osu = -0.3
            orientation_diff_osu = -0.3
            ang_vel_diff_osu = -0.3
            ang_vel_diff_osu_positive = 0.3
            action_diff_osu = -0.15
            torques_osu = 0.0#-0.1
            # addition reward
            dof_pos_limits_osu = -0.8

            swing_landing_vel = 0.0#-0.3
            swing_landing_vel_positive = 0.1

            upperbody_action_diff_osu = -1.0
            feet_distance = -0.2
            arm_pose = 0.0#-0.3
            arm_pose_shoulder_pitch = 0.0#-0.1
            dof_acc_diff_osu = -0.15
            dof_vel_diff_osu = -0.3 
            feet_orien_diff_osu = -0.2
            ankleaction_diff_osu = -0.6
            hipyaw_osu = -0.3
            torso_orientation_diff_osu = 0.0#-0.1
            bias = 4.2

            swing_arm = 0.3
            swing_symmetric = 0.2
            # torso_yaw = -0.3
            # torso_ang_vel_xy_osu = -0.1

            double_air = 0.0#-0.5
            torques = -25.0e-6
            knee_torques = 0.0#-25.e-5  # -0.0001
            knee_torques_shape = -20.e-5
            
            # arm reward
            arm_roll_keepposture = -0.0
            arm_pitch_keepposture = -0.0#-0.1
            lowarm_keepposture = -0.0#-0.1
            arm_keepposture_penalty = -0.8#-0.4
            arm_keepposture_positive = 0.8#0.4
            arm_torques = -0.001
            arm_torque_limits_penalty = -0.2
            arm_torque_limits_positive = 0.2

            arm_right_elbow_positive = 0.3
            arm_right_elbow_penalty = -0.3

            dof_vel = -0.002
            dof_acc = -1.e-7#-1.e-6# apply phase changed method
            dof_vel_upperbody = -0.0
            dof_acc_upperbody =0.0 #-1.e-6
            

            ankle_torques = 0.0#-25.e-5#-5.e-6#-0.005  # -0.001# -0.005
            ankle_torque_limits_penalty = -0.2
            ankle_torque_limits_positive = 0.2
            ankle_roll_torque_limits_penalty = -0.2
            ankle_roll_torque_limits_positive = 0.2
            ankle_action_diff_osu_penalty = -0.2#
            ankle_action_diff_osu_positive = 0.2#
            ankle_roll_actions_penalty = 0.0#-0.00002
            ankle_roll_actions_positive = 0.0#0.2

            ankle_roll_torques = 0.0#-25.e-5

            swing_height_positive = 0.3
            swing_height_penalty = -0.3
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
    runner_class_name = 'OnPolicyRunner_lstm'
    class policy(LeggedRobotCfgPPO.policy):
        init_noise_std = 1.0
        actor_hidden_dims = []
        critic_hidden_dims = []
        activation = 'elu'  # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
        # only for 'ActorCriticRecurrent':
        rnn_type = 'lstm'
        rnn_hidden_size = 128
        rnn_num_layers = 2

    class runner(LeggedRobotCfgPPO.runner):
        policy_class_name = 'ActorCriticRecurrent'
        algorithm_class_name = 'PPO_lstm'
        num_steps_per_env = 200  # 100 # per iteration
        # policy_class_name = 'ActorCritic'
        run_name = ''
        experiment_name = 'GR1L_lstm_start2'
        max_iterations = 50000  # number of policy updates
        save_interval = 100  # check for potential saves every this many iterations
        # load and resume
        resume = True 
        # "_Oct09_18-21-32" : mirro leg actions.
                # res : x vel 不跟隨， left arm 奇怪, 落地較重， 
            # "_Oct10_11-22-40"： add arm keep posture from -0.2 to -0.4, alpha from -50 to -100, add x vel from -0.5 to -0.8,  adjust landing_vel, add trimesh, leg 截止頻率爲50Hz
                # "_Oct10_14-01-37": add dof_vel, add dof_acc
                    # "_Oct10_21-17-51" : delete torso_orientation reward, reduce orientation reward from -10 to -2, x vel alpha from 50 to 100
                            #res: action_diff 不夠， action 抖動嚴重
                        # “_Oct11_10-48-29”： action_diff alpha from -5.0 to - 50.0, upperbody_action_diff_osu alpha from -0.05 to -20.0
                                # res : 難以產生效果，
                            # "_Oct11_13-18-47" : delete arm_pose
                                    # res : action_diff 無效果
                                # "" :action filter add to torque compute, but keep in the reward compute, change action diff alpha to small
        # "_Oct11_17-10-04": start
            # "_Oct12_19-45-52" : add dof_vel dof_acc
                # "_Oct13_19-05-14" : upperbody_action_diff from alpha 1.0 to 50.0, arm_keep_posture from -0.4 to -0.8
                        # res : 跆腳高度低， 右臂最後一個關節位置不對 傾向於80 電機運動
                    # "_Oct14_14-38-33" : delete knee_torque penalty, add foot height positive, add right arm elbow penalty
                            # res : 右臂最後一個關節位置不對
                        # "_Oct16_20-13-07 " : 使用last_actions
                            # "_Oct18_10-24-12" : obs 使用actions_filter,
                                    # res : webots, 不抬腿，脚踝roll 方向容易侧翻
                                # “_Oct24_17-59-05” ： add penalty foot height, 
                                    # : add random, add reduce landing vel penalty
                                        # : add knee penalty
                                            # : dof_acc and dof_vel add more, add more mirror loss
        load_run = -1#"_Oct26_13-31-41"#-1
        checkpoint = -1#1600  # 1000 # -1 = last saved model
        resume_path = None  # updated from load_run and chkpt

    class algorithm(LeggedRobotCfgPPO.algorithm):
        # training params
        num_learning_epochs = 8
        num_mini_batches = 40  # mini batch size = num_envs*nsteps / nminibatches
        learning_rate = 5.e-4  # 5.e-4
        # mirror_profile = 'GR1L'
        # mirror_coef = [1.0]