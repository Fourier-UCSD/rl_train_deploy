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
# import cv2
from legged_gym import LEGGED_GYM_ROOT_DIR
import os

import isaacgym
from isaacgym import gymapi
from legged_gym.envs import *
from legged_gym.utils import get_args, export_policy_as_jit, task_registry, Logger

import numpy as np
import torch


def play(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)

    # override some parameters for testing
    env_cfg.env.episode_length_s = 60
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 1)
    env_cfg.terrain.mesh_type = 'plane'  #
    # env_cfg.terrain.num_rows = 10
    # env_cfg.terrain.num_cols = 20
    env_cfg.terrain.terrain_proportions = [0.2, 0.2, 0.0, 0.0, 1.0]
    env_cfg.terrain.slope_treshold = 0.1

    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = True
    env_cfg.noise.noise_level = 0.1

    env_cfg.domain_rand.randomize_friction = True
    env_cfg.domain_rand.friction_range = [0.3, 1.0]
    env_cfg.domain_rand.randomize_base_mass = True
    env_cfg.domain_rand.randomize_thigh_mass = False
    env_cfg.domain_rand.randomize_shin_mass = False
    env_cfg.domain_rand.randomize_torso_mass = False
    env_cfg.domain_rand.randomize_upper_arm_mass = False
    env_cfg.domain_rand.randomize_lower_arm_mass = False
    env_cfg.domain_rand.added_mass_range = [0.0, 0.0]
    env_cfg.domain_rand.randomize_base_com = True
    env_cfg.domain_rand.added_com_range_x = [-0.1, 0.1]
    env_cfg.domain_rand.added_com_range_y = [-0.05, 0.05]
    env_cfg.domain_rand.added_com_range_z = [0.2, 0.2]

    env_cfg.domain_rand.push_robots = False
    env_cfg.domain_rand.push_interval_s = 2.0
    env_cfg.domain_rand.max_push_vel_xy = 0.8
    env_cfg.domain_rand.apply_forces = False
    env_cfg.domain_rand.continue_time_s = 1.0
    # env_cfg.domain_rand.max_ex_forces = [-0.0, 0.0]
    # env_cfg.domain_rand.max_ex_forces_foot = [-0.0, 0.0]
    # env_cfg.domain_rand.max_ex_forces_thigh = [-0, 0]
    # env_cfg.domain_rand.max_ex_torques = [-0.0, 0.0]
    env_cfg.domain_rand.randomize_motor_strength = True
    env_cfg.domain_rand.motor_strength = [1.0, 1.0]
    env_cfg.domain_rand.randomize_obs_linvel = True
    env_cfg.domain_rand.obs_linvel = [0.8, 1.0]

    env_cfg.commands.curriculum = False
    # env_cfg.commands.gait_cycle =1.0
    env_cfg.commands.heading_command = False
    env_cfg.commands.ranges.lin_vel_x = [0.3, 0.3]  # min max [m/s]
    env_cfg.commands.ranges.lin_vel_y = [-0.0, -0.0]  # min max [m/s]
    env_cfg.commands.ranges.ang_vel_yaw = [0.0, 0.0]  # min max [rad/s]
    env_cfg.commands.ranges.heading = [0.0, 0.0]

    # env_cfg.commands.ranges_walk.lin_vel_x = [0.5, 0.5]  # min max [m/s]

    env_cfg.commands.resample_command_log = True

    env_cfg.commands.resample_command_log = True  # open debug log

    print("-------------------")
    print("lin_vel_x = ", env_cfg.commands.ranges.lin_vel_x)
    print("ang_vel_yaw = ", env_cfg.commands.ranges.ang_vel_yaw)
    print("heading = ", env_cfg.commands.ranges.heading)

    env_cfg.asset.self_collisions = 0

    # prepare environment
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()
    data = np.zeros(env.num_obs)
    # load policy
    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)

    # export policy as a jit module (used to run it from C++)
    if EXPORT_POLICY:
        path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'policies')
        export_policy_as_jit(ppo_runner.alg.actor_critic, path)
        print('Exported policy as jit script to: ', path)

    logger = Logger(env.dt)
    robot_index = 0  # 7 which robot is used for logging
    joint_index = 0 # which joint is used for logging
    stop_state_log = 1200  # number of steps before plotting states
    stop_rew_log = env.max_episode_length + 1  # number of steps before print average episode rewards

    camera_rot = 0.8 * np.pi / 4
    camera_rot_per_sec = -np.pi / 24
    img_idx = 0

    video = None
    for i in range(1 * int(env.max_episode_length)):
        # obs[:,:]=0.0
        # for i in range(5):
        actions = policy(obs.detach())
        # print("actions:")
        # print(actions[:,12:])
        # exit(1)
        # actions[:,16] = actions[:,16].clip(max=0.2,min=-0.2)
        # action_temp = actions.clone()
        # actions[:,19] = 0
        # actions[:,14] = 0
        obs, _, rews, dones, infos, _, _ = env.step(actions.detach())
        actions = env.get_actions()
        # amp_obs = env.get_amp_observations()[0].cpu().numpy()

        data = np.vstack((data, obs.cpu().numpy()))
        if RECORD_FRAMES:
            frames_path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported',
                                       'frames')
            if not os.path.isdir(frames_path):
                os.mkdir(frames_path)
            filename = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'frames',
                                    f"{img_idx}.png")
            env.gym.write_viewer_image_to_file(env.viewer, filename)
            img = cv2.imread(filename)
            if video is None:
                video = cv2.VideoWriter('./record_whole_walkfast_0427_9800.mp4', cv2.VideoWriter_fourcc(*'mp4v'),
                                        int(1 / env.dt), (img.shape[1], img.shape[0]))
            video.write(img)
            img_idx += 1
        if MOVE_CAMERA:
            # Reset camera position.
            look_at = np.array(env.root_states[robot_index, :3].cpu(), dtype=np.float64)
            camera_rot = (camera_rot + camera_rot_per_sec * env.dt) % (2 * np.pi)
            camera_relative_position = 3.0 * np.array([np.cos(camera_rot), np.sin(camera_rot), 0.45])  # 5.2*
            env.set_camera(look_at + camera_relative_position, look_at)

        if i < stop_state_log:
            logger.log_states(
                {
                    'dof_pos_target': actions[robot_index, joint_index].item() * env.cfg.control.action_scale,
                    # 'dof_pos': env.dof_pos[robot_index, joint_index].item(),
                    'dof_pos': env.obs_buf[robot_index, 12 + joint_index].item(),
                    # 'dof_vel': env.dof_vel[robot_index, joint_index].item(),
                    'dof_vel': env.obs_buf[robot_index, 35 + joint_index].item(),
                    'dof_torque': env.torques[robot_index, joint_index].item(),
                    'dof_torque_filter': env.computed_torques_filter[robot_index, joint_index].item(),
                    'dof_vel_hiproll': env.obs_buf[robot_index, 35].item(),
                    'dof_torque_hiproll': env.torques[robot_index, 0].item(),
                    'dof_vel_hipyaw': env.obs_buf[robot_index, 36].item(),
                    'dof_torque_hipyaw': env.torques[robot_index, 1].item(),
                    'dof_vel_hippitch': env.obs_buf[robot_index, 37].item(),
                    'dof_torque_hippitch': env.torques[robot_index, 2].item(),
                    'dof_vel_knee': env.obs_buf[robot_index, 38].item(),
                    'dof_torque_knee': env.torques[robot_index, 3].item(),
                    'dof_vel_ankle': env.obs_buf[robot_index, 39].item(),
                    'dof_torque_ankle': env.torques[robot_index, 4].item(),
                    'dof_vel_yaw': env.obs_buf[robot_index, 36].item(),
                    'dof_torque_yaw': env.torques[robot_index, 1].item(),
                    'command_x': env.commands[robot_index, 0].item(),
                    'command_y': env.commands[robot_index, 1].item(),
                    'command_yaw': env.commands[robot_index, 2].item(),
                    'base_vel_x': env.base_lin_vel[robot_index, 0].item(),
                    'base_vel_y': env.base_lin_vel[robot_index, 1].item(),
                    'base_vel_z': env.base_lin_vel[robot_index, 2].item(),
                    'base_vel_yaw': env.base_ang_vel[robot_index, 2].item(),
                    'contact_forces_z': env.contact_forces[robot_index, env.feet_indices, 2].cpu().numpy(),
                    # 'contact_forces_z': env.force_sensor_readings[robot_index, env.feet_indices, 2].cpu().numpy(),
                    'left_foot_vel': env.rigid_body_states[:, env.feet_indices][robot_index, 0, 9].item(),
                    'right_foot_vel': env.rigid_body_states[:, env.feet_indices][robot_index, 1, 9].item(),
                    'base_avg_vel_x': env.avg_x_vel[robot_index].item(),
                    'base_avg_vel_y': env.avg_y_vel[robot_index].item(),
                    'base_avg_vel_yaw': env.avg_yaw_vel[robot_index].item()
                }
            )
        elif i == stop_state_log:
            logger.plot_states()
        if 0 < i < stop_rew_log:
            if infos["episode"]:
                num_episodes = torch.sum(env.reset_buf).item()
                if num_episodes > 0:
                    logger.log_rewards(infos["episode"], num_episodes)
        elif i == stop_rew_log:
            logger.print_rewards()
    if SAVE_DATA:
        np.savetxt('datacollection_walk0411.txt', data, fmt='%f', delimiter='\t')
    if RECORD_FRAMES:
        video.release()


if __name__ == '__main__':
    EXPORT_POLICY = True
    RECORD_FRAMES = False
    MOVE_CAMERA = False
    SAVE_DATA = False
    args = get_args()
    play(args)
