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

from turtle import right
from legged_gym import LEGGED_GYM_ROOT_DIR, envs
from time import time
from warnings import WarningMessage
import numpy as np
import os

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
from torch import Tensor
from typing import Tuple, Dict

from legged_gym import LEGGED_GYM_ROOT_DIR
from legged_gym.envs.base.base_task import BaseTask
from legged_gym.utils.terrain import Terrain
from legged_gym.utils.math import quat_apply_yaw, wrap_to_pi, torch_rand_sqrt_float
from legged_gym.utils.helpers import class_to_dict
from rsl_rl.datasets.motion_loader import AMPLoader
from scipy.spatial.transform import Rotation as R

from .legged_robot_config import LeggedRobotCfg


class LeggedRobot(BaseTask):
    def __init__(self, cfg, sim_params, physics_engine, sim_device, headless):
        """ Parses the provided config file,
            calls create_sim() (which creates, simulation, terrain and environments),
            initilizes pytorch buffers used during training
        Args:
            cfg (Dict): Environment config file
            sim_params (gymapi.SimParams): simulation parameters
            physics_engine (gymapi.SimType): gymapi.SIM_PHYSX (must be PhysX)
            device_type (string): 'cuda' or 'cpu'
            device_id (int): 0, 1, ...
            headless (bool): Run without rendering if True
        """

        # 标志位置 False
        self.init_done = False

        self.cfg = None
        self.init_cfg(cfg)

        self.height_samples = None
        self.debug_viz = False

        self.obs_profile = None
        self.init_obs(cfg)

        super().__init__(cfg, sim_params, physics_engine, sim_device, headless)
        # self.sim_params = sim_params

        self._parse_cfg()

        # 其他内容初始化
        if not self.headless:
            self.set_camera(self.cfg.viewer.pos, self.cfg.viewer.lookat)

        self._init_buffers()
        self._prepare_reward_function()

        if self.cfg.env.reference_state_initialization:
            self.amp_loader = AMPLoader(motion_files=self.cfg.env.amp_motion_files, device=self.device,
                                        time_between_frames=self.dt)
        # self.amp_loader = AMPLoader(motion_files=self.cfg.env.amp_motion_files, device=self.device, time_between_frames=self.dt)

        # 标志位置 True
        self.init_done = True

    def init_cfg(self, cfg: LeggedRobotCfg):
        self.cfg: LeggedRobotCfg = cfg

    def init_obs(self, cfg: LeggedRobotCfg):
        self.obs_profile = cfg.env.obs_profile

    def reset(self):
        """ Reset all robots"""
        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        obs, privileged_obs, _, _, _, _, _ = self.step(
            torch.zeros(self.num_envs, self.num_actions, device=self.device, requires_grad=False))
        return obs, privileged_obs

    def get_observations(self):
        return self.obs_buf

    def get_privileged_observations(self):
        return self.privileged_obs_buf

    def show(self, time):
        # self.render()
        amp_obs = self.amp_loader.get_full_frame_at_time(0, time)
        # print(amp_obs)
        self.dof_pos[:, 12:23] = torch.zeros(11, device=self.device, requires_grad=False)
        self.dof_pos[:, :6] = amp_obs[6:12]
        self.dof_pos[:, 6:12] = amp_obs[:6]
        self.dof_vel[:, :6] = amp_obs[18:24]
        self.dof_vel[:, 6:12] = amp_obs[12:18]
        self.dof_vel[:, 12:23] = torch.zeros(11, device=self.device, requires_grad=False)
        env_ids_int32 = torch.arange(self.num_envs, device=self.device).to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(self.sim,
                                              gymtorch.unwrap_tensor(self.dof_state),
                                              gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))

        self._reset_root_states_show(torch.arange(self.num_envs, device=self.device))
        self.render()
        self.gym.simulate(self.sim)
        if self.device == 'cpu':
            self.gym.fetch_results(self.sim, True)
        if self.viewer and self.enable_viewer_sync and self.debug_viz:
            self._draw_debug_vis()
        # obs, privileged_obs, _, _, _, _, _ = self.step(torch.zeros(self.num_envs, self.num_actions, device=self.device, requires_grad=False))

    def _reset_root_states_show(self, env_ids):
        """ Resets ROOT states position and velocities of selected environmments
            Sets base position based on the curriculum
            Selects randomized base velocities within -0.5:0.5 [m/s, rad/s]
        Args:
            env_ids (List[int]): Environemnt ids
        """
        # base position
        self.root_states[env_ids] = self.base_init_state
        self.root_states[env_ids, 2] += 0.15
        self.root_states[env_ids, :3] += self.env_origins[env_ids]
        # self.root_states[env_ids, :2] += torch_rand_float(-1., 1., (len(env_ids), 2), device=self.device) # xy position within 1m of the center

        # base velocities
        self.root_states[env_ids, 7:13] = torch_rand_float(-0.0, 0.0, (len(env_ids), 6),
                                                           device=self.device)  # [7:10]: lin vel, [10:13]: ang vel
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(self.sim,
                                                     gymtorch.unwrap_tensor(self.root_states),
                                                     gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))

    def step(self, actions):
        """ Apply actions, simulate, call self.post_physics_step()
        Args:
            actions (torch.Tensor): Tensor of shape (num_envs, num_actions_per_env)
        """
        clip_actions = self.cfg.normalization.clip_actions
        self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device)
        # self.actions[:,[5,11]] = 0.0
        alpha = 1.0#0.39347
        alpha2 = 0.05
        self.actions_filter[:,0:12] = (1.0 - alpha)*self.actions_filter[:,0:12] + alpha*self.last_actions[:,0:12]
        self.actions_filter[:,12:] = (1.0 - alpha2)*self.actions_filter[:,12:] + alpha2*self.last_actions[:,12:]
        # self.actions[:] = self.actions_filter[:]
        # step physics and render each frame
        self.render()
        self.avg_feet_spd = 0.0
        self.avg_feet_frc = 0.0

        for i in range(self.cfg.control.decimation):

            # self.torques = self._compute_torques(self.actions).view(self.torques.shape)
            self.torques = self._compute_torques(self.actions_filter).view(self.torques.shape)
            # self.torques = self._compute_torques(self.delayed_actions).view(self.torques.shape)
            # # play for check model
            # pos = torch.zeros(self.num_envs, self.num_actions, dtype=torch.float, device=self.device, requires_grad=False)
            # self.gym.set_env_rigid_body_states(self.sim, gymtorch.unwrap_tensor(pos))

            self.gym.set_dof_actuation_force_tensor(self.sim, gymtorch.unwrap_tensor(self.torques))
            self.gym.simulate(self.sim)

            if self.device == 'cpu':
                self.gym.fetch_results(self.sim, True)

            self.gym.refresh_dof_state_tensor(self.sim)
            self.gym.refresh_actor_root_state_tensor(self.sim)
            self.gym.refresh_net_contact_force_tensor(self.sim)
            self.gym.refresh_rigid_body_state_tensor(self.sim)

            if i > 1:
                self.delayed_actions = self.actions.clone()

            if i < self.cfg.control.decimation - 2:
                self.delayed_dofpos = self.dof_pos.clone()
                self.delayed_dofvel = self.dof_vel.clone()

            # print("self.dof_pos")
            # print(self.dof_pos)
            # print("self.delayed_dofpos")
            # print(self.delayed_dofpos)
            # print("actions")
            # print(self.actions)
            # print(self.delayed_actions)

            self.avg_feet_frc += torch.norm(self.contact_forces[:, self.feet_indices, :3], dim=-1)
            self.avg_feet_spd += torch.norm(self.rigid_body_states[:, self.feet_indices][:, 0:2, 7:10], dim=-1)

        self.avg_feet_frc /= self.cfg.control.decimation
        self.avg_feet_spd /= self.cfg.control.decimation

        reset_env_ids, terminal_amp_states = self.post_physics_step()

        # print(self.avg_feet_frc)
        # print(self.avg_feet_spd)
        # return clipped obs, clipped states (None), rewards, dones and infos
        clip_obs = self.cfg.normalization.clip_observations
        self.obs_buf = torch.clip(self.obs_buf, -clip_obs, clip_obs)

        if self.privileged_obs_buf is not None:
            self.privileged_obs_buf = torch.clip(self.privileged_obs_buf, -clip_obs, clip_obs)

        return (self.obs_buf,
                self.privileged_obs_buf,
                self.rew_buf,
                self.reset_buf,
                self.extras,
                reset_env_ids,
                terminal_amp_states)

    def post_physics_step(self):
        """ check terminations, compute observations and rewards
            calls self._post_physics_step_callback() for common computations 
            calls self._draw_debug_vis() if needed
        """
        # self.gym.refresh_actor_root_state_tensor(self.sim)
        # self.gym.refresh_net_contact_force_tensor(self.sim)
        # self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.gym.refresh_force_sensor_tensor(self.sim)
        # print(self.contact_forces[:, self.feet_indices[1], :])
        self.episode_length_buf += 1
        self.common_step_counter += 1

        # prepare quantities
        self.base_quat[:] = self.root_states[:, 3:7]
        self.base_lin_vel[:] = quat_rotate_inverse(self.base_quat, self.root_states[:, 7:10])
        self.base_ang_vel[:] = quat_rotate_inverse(self.base_quat, self.root_states[:, 10:13])
        self.projected_gravity[:] = quat_rotate_inverse(self.base_quat, self.gravity_vec)
        # self.footstates = self.rigid_body_states[:,self.feet_indices][:, :, 0:13]
        # self.foot_pos = self.rigid_body_states[:,self.feet_indices][:, :, 0:3]
        # self.foot_orien = self.rigid_body_states[:,self.feet_indices][:, :, 3:7]
        # self.left_foot_orien[:] = quat_mul(self.base_quat,self.foot_orien[:,0,:])
        self.left_foot_orien_projected[:] = quat_rotate_inverse(self.rigid_body_states[:, self.feet_indices][:, 0, 3:7],
                                                                self.gravity_vec)
        self.right_foot_orien_projected[:] = quat_rotate_inverse(
            self.rigid_body_states[:, self.feet_indices][:, 1, 3:7], self.gravity_vec)
        # print("contact_forces")
        # print(self.contact_forces[:, self.feet_indices, 2])
        # print("force_sensor_readings")
        # print(self.force_sensor_readings[:,:,2])
        self._post_physics_step_callback()

        # compute observations, rewards, resets, ...
        self.check_termination()
        self.compute_reward()
        env_ids = self.reset_buf.nonzero(as_tuple=False).flatten()
        terminal_amp_states = self.get_amp_observations()[env_ids]
        self.reset_idx(env_ids)
        self.compute_observations()  # in some cases a simulation step might be required to refresh some obs (for example body positions)

        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = self.dof_vel[:]
        self.last_root_vel[:] = self.root_states[:, 7:13]

        if self.viewer and self.enable_viewer_sync and self.debug_viz:
            self._draw_debug_vis()

        return env_ids, terminal_amp_states

    def check_termination(self):
        """ Check if environments need to be reset
        """
        self.reset_buf = torch.any(torch.norm(self.contact_forces[:, self.termination_contact_indices, :], dim=-1) > 1.,
                                   dim=1)

        self.time_out_buf = self.episode_length_buf > self.max_episode_length  # no terminal reward for time-outs
        self.reset_buf |= self.time_out_buf

    def reset_idx(self, env_ids):
        """ Reset some environments.
            Calls self._reset_dofs(env_ids), self._reset_root_states(env_ids), and self._resample_commands(env_ids)
            [Optional] calls self._update_terrain_curriculum(env_ids), self.update_command_curriculum(env_ids) and
            Logs episode info
            Resets some buffers
        Args:
            env_ids (list[int]): List of environment ids which must be reset
        """
        if len(env_ids) == 0:
            return

        # update curriculum
        if self.cfg.terrain.curriculum:
            self._update_terrain_curriculum(env_ids)

        # avoid updating command curriculum at each step since the maximum command is common to all envs
        if self.cfg.commands.curriculum and (self.common_step_counter % self.max_episode_length == 0):
            self.update_command_curriculum(env_ids)

        # reset robot states
        if self.cfg.env.reference_state_initialization:
            frames = self.amp_loader.get_full_frame_batch(len(env_ids))
            self._reset_dofs_amp(env_ids, frames)
            self._reset_root_states_amp(env_ids, frames)
        else:
            self._reset_dofs(env_ids)
            self._reset_root_states(env_ids)

        self._resample_commands(env_ids,
                                self.cfg.commands.resample_command_profile,
                                self.cfg.commands.resample_command_profile_randomize,
                                self.cfg.commands.resample_command_log)
        # self._resample_reaching_target(env_ids,env_ids)

        self._reset_fric_para(env_ids)

        # reset buffers
        self.actions[env_ids] = 0.0
        self.last_actions[env_ids] = 0.
        self.last_dof_vel[env_ids] = 0.
        self.last_root_vel[env_ids] = 0.
        self.avg_feet_frc[env_ids] = 0.
        self.avg_feet_spd[env_ids] = 0.
        self.avg_x_vel[env_ids] = 0.0
        self.avg_y_vel[env_ids] = 0.0
        self.avg_yaw_vel[env_ids] = 0.0
        self.feet_air_time[env_ids] = 0.
        self.last_feet_air_time[env_ids] = 0.
        self.feet_distance[env_ids] = 0.
        self.last_feet_distance[env_ids] = 0.
        self.episode_length_buf[env_ids] = 0

        self.reset_buf[env_ids] = 1
        self.actions_filter[env_ids] = 0
        self.rand_init_phases[env_ids] = torch.randint(0,2,(len(env_ids),),dtype=torch.float,device=self.device,requires_grad=False)*2 - 1

        # fill extras
        self.extras["episode"] = {}
        for key in self.episode_sums.keys():
            self.extras["episode"]['rew_' + key] = torch.mean(
                self.episode_sums[key][env_ids]) / self.max_episode_length_s
            self.episode_sums[key][env_ids] = 0.

        # log additional curriculum info
        if self.cfg.terrain.curriculum:
            self.extras["episode"]["terrain_level"] = torch.mean(self.terrain_levels.float())

        if self.cfg.commands.curriculum:
            self.extras["episode"]["min_command_x"] = self.command_ranges["lin_vel_x"][0]
            self.extras["episode"]["max_command_x"] = self.command_ranges["lin_vel_x"][1]
            self.extras["episode"]["min_command_y"] = self.command_ranges["lin_vel_y"][0]
            self.extras["episode"]["max_command_y"] = self.command_ranges["lin_vel_y"][1]
            self.extras["episode"]["min_command_yaw"] = self.command_ranges["ang_vel_yaw"][0]
            self.extras["episode"]["max_command_yaw"] = self.command_ranges["ang_vel_yaw"][1]

        # send timeout info to the algorithm
        if self.cfg.env.send_timeouts:
            self.extras["time_outs"] = self.time_out_buf

    def compute_reward(self):
        """ Compute rewards
            Calls each reward function which had a non-zero scale (processed in self._prepare_reward_function())
            adds each terms to the episode sums and to the total reward
        """
        self.rew_buf[:] = 0.0

        for i in range(len(self.reward_functions)):
            name = self.reward_names[i]
            rew = self.reward_functions[i]() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        if self.cfg.rewards.only_positive_rewards:
            self.rew_buf[:] = torch.clip(self.rew_buf[:], min=0.0)

        # add termination reward after clipping
        if "termination" in self.reward_scales:
            rew = self._reward_termination() * self.reward_scales["termination"]
            self.rew_buf += rew
            self.episode_sums["termination"] += rew

    def compute_observations(self):
        """ Computes observations
        """
        # compute observations profile
        self.computer_observation_profile()

        # calculate observation noise
        self.computer_observation_noise()

    def computer_observation_profile(self):

        if self.obs_profile == "min":
            self.obs_buf = torch.cat((
                self.base_ang_vel * self.obs_scales.ang_vel,
                self.projected_gravity,
                self.commands[:, :3] * self.commands_scale,
                (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                self.dof_vel * self.obs_scales.dof_vel,
                self.actions
            ), dim=-1)

        elif self.obs_profile == "full":
            # print("self.base_lin_vel * self.obs_scales.lin_vel")
            # print(self.base_lin_vel * self.obs_scales.lin_vel)
            # print("self.base_ang_vel  * self.obs_scales.ang_vel")
            # print(self.base_ang_vel  * self.obs_scales.ang_vel)
            # print("self.projected_gravity")
            # print(self.projected_gravity)
            # print("(self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos")
            # print((self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos)
            # print("self.dof_vel * self.obs_scales.dof_vel")
            # print(self.dof_vel * self.obs_scales.dof_vel)
            # print("self.actions")
            # print(self.actions)
            # print("torch.sin(2*torch.pi*self.gait_phase)")
            # print(torch.sin(2*torch.pi*self.gait_phase))
            # print("self.phase_ratio")
            # print(self.phase_ratio)
            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      self.actions
                                      ), dim=-1)
            # add perceptive inputs if not blind
            if self.cfg.terrain.measure_heights:
                heights = torch.clip(
                    self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights,
                    -1, 1.) * self.obs_scales.height_measurements
                self.obs_buf = torch.cat((self.obs_buf, heights), dim=-1)

        elif self.obs_profile == "full-osu":

            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      self.actions,
                                      torch.sin(2 * torch.pi * self.gait_phase),
                                      torch.cos(2 * torch.pi * self.gait_phase),
                                      self.phase_ratio
                                      ), dim=-1)
            # add perceptive inputs if not blind
            if self.cfg.terrain.measure_heights:
                if self.cfg.terrain.blind_test:
                    heights = torch.clip(self.root_states[:, 2].unsqueeze(
                        1) - self.cfg.rewards.base_height_target - self.measured_heights_blind, -1,
                                         1.) * self.obs_scales.height_measurements
                else:
                    heights = torch.clip(self.root_states[:, 2].unsqueeze(
                        1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                                         1.) * self.obs_scales.height_measurements
                self.obs_buf = torch.cat((self.obs_buf, heights), dim=-1)

        elif self.obs_profile == "min-osu":
            self.obs_buf = torch.cat((
                self.base_ang_vel * self.obs_scales.ang_vel,
                self.projected_gravity,
                self.commands[:, :3] * self.commands_scale,
                (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                self.dof_vel * self.obs_scales.dof_vel,
                self.actions,
                torch.sin(2 * torch.pi * self.gait_phase),
                torch.cos(2 * torch.pi * self.gait_phase),
                self.phase_ratio
            ), dim=-1)

        elif self.obs_profile == "vs-rma-teacher":
            base_height = torch.mean(self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1)
            # print(self.lin_vel_scales)
            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      self.actions,
                                      torch.sin(2 * torch.pi * self.gait_phase),
                                      torch.cos(2 * torch.pi * self.gait_phase),
                                      self.phase_ratio,
                                      base_height.unsqueeze(1),
                                      self.friction.unsqueeze(1),
                                      self.pevils_mass.unsqueeze(1),
                                      self.pevils_com,
                                      self.motor_strength
                                      ), dim=-1)

        elif self.obs_profile == "vs-rma-teacher-2":
            base_height = torch.mean(torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements, dim=1)
            # print(base_height)
            self.avg_x_vel = 2 * self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 0] + (
                    1 - 2 * self.dt / self.cfg.commands.gait_cycle) * self.avg_x_vel
            self.avg_y_vel = self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 1] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_y_vel
            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      # self.delayed_dofvel * self.obs_scales.dof_vel,
                                      self.actions,
                                      torch.sin(2 * torch.pi * self.gait_phase),
                                      torch.cos(2 * torch.pi * self.gait_phase),
                                      self.phase_ratio,
                                      base_height.unsqueeze(1)
                                      ), dim=-1)

        elif self.obs_profile == "vs-rma-teacher-3":
            base_height = torch.mean(torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements, dim=1)
            # print(base_height)
            # print(self.dof_vel[:,17])

            # self.commands_scale[2]= 0.5
            # self.actions[:,[5,11]] =0.0
            # print(self.actions[:,[5,11]])
            self.avg_x_vel = 2 * self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 0] + (
                    1 - 2 * self.dt / self.cfg.commands.gait_cycle) * self.avg_x_vel
            self.avg_y_vel = self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 1] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_y_vel
            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      # self.delayed_dofvel * self.obs_scales.dof_vel,
                                      self.actions,
                                      torch.sin(2 * torch.pi * self.gait_phase),
                                      torch.cos(2 * torch.pi * self.gait_phase),
                                      self.phase_ratio,
                                      base_height.unsqueeze(1),
                                      (self.avg_x_vel * self.lin_vel_scales[:, 0]).unsqueeze(
                                          1) * self.obs_scales.lin_vel,
                                      (self.avg_y_vel * self.lin_vel_scales[:, 1]).unsqueeze(
                                          1) * self.obs_scales.lin_vel
                                      ), dim=-1)

        elif self.obs_profile == "vs-rma-teacher-3-delayedimu":
            base_height = torch.mean(torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements, dim=1)
            # print(base_height)
            # print(self.dof_vel[:,17])

            # self.commands_scale[2]= 0.5
            # self.actions[:,[5,11]] =0.0
            # print(self.actions[:,[5,11]])
            self.avg_x_vel = 2 * self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 0] + (
                    1 - 2 * self.dt / self.cfg.commands.gait_cycle) * self.avg_x_vel
            self.avg_y_vel = self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 1] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_y_vel
            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      # self.delayed_dofvel * self.obs_scales.dof_vel,
                                      self.actions,
                                      torch.sin(2 * torch.pi * self.gait_phase),
                                      torch.cos(2 * torch.pi * self.gait_phase),
                                      self.phase_ratio,
                                      base_height.unsqueeze(1),
                                      (self.avg_x_vel * self.lin_vel_scales[:, 0]).unsqueeze(
                                          1) * self.obs_scales.lin_vel,
                                      (self.avg_y_vel * self.lin_vel_scales[:, 1]).unsqueeze(
                                          1) * self.obs_scales.lin_vel
                                      ), dim=-1)

        elif self.obs_profile == "sonnie-2":
            # print(self.root_states[:, 2])
            base_height = torch.mean(torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements, dim=1)
            # print(base_height)
            # print(self.dof_vel[:,17])

            # self.commands_scale[2]= 0.5
            # self.actions[:,[5,11]] =0.0
            # print(self.actions[:,[5,11]])
            self.avg_x_vel = 2 * self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 0] + (
                    1 - 2 * self.dt / self.cfg.commands.gait_cycle) * self.avg_x_vel
            self.avg_y_vel = self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 1] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_y_vel
            self.avg_yaw_vel = self.dt / self.cfg.commands.gait_cycle * self.base_ang_vel[:, 2] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_yaw_vel
            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      # self.delayed_dofvel * self.obs_scales.dof_vel,
                                      self.actions,
                                      torch.sin(2 * torch.pi * self.gait_phase),
                                      torch.cos(2 * torch.pi * self.gait_phase),
                                      self.phase_ratio,
                                      base_height.unsqueeze(1),
                                      (self.avg_x_vel * self.lin_vel_scales[:, 0]).unsqueeze(
                                          1) * self.obs_scales.lin_vel,
                                      (self.avg_y_vel * self.lin_vel_scales[:, 1]).unsqueeze(
                                          1) * self.obs_scales.lin_vel,
                                      self.avg_yaw_vel.unsqueeze(1) * self.obs_scales.ang_vel
                                      ), dim=-1)

        elif self.obs_profile == "sonnie-gym":
            heights = torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements
            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      # self.delayed_dofvel * self.obs_scales.dof_vel,
                                      self.actions,
                                      heights
                                      ), dim=-1)

        elif self.obs_profile == "sonnie-reaching":
            # print(self.root_states[:, 2])
            base_height = torch.mean(torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements, dim=1)
            # print(base_height)
            # print(self.dof_vel[:,17])

            # self.commands_scale[2]= 0.5
            # self.actions[:,[5,11]] =0.0
            # print(self.actions[:,[5,11]])
            self.avg_x_vel = 2 * self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 0] + (
                    1 - 2 * self.dt / self.cfg.commands.gait_cycle) * self.avg_x_vel
            self.avg_y_vel = self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 1] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_y_vel
            self.avg_yaw_vel = self.dt / self.cfg.commands.gait_cycle * self.base_ang_vel[:, 2] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_yaw_vel
            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      # self.delayed_dofvel * self.obs_scales.dof_vel,
                                      self.actions,
                                      torch.sin(2 * torch.pi * self.gait_phase),
                                      torch.cos(2 * torch.pi * self.gait_phase),
                                      self.phase_ratio,
                                      base_height.unsqueeze(1),
                                      (self.avg_x_vel * self.lin_vel_scales[:, 0]).unsqueeze(
                                          1) * self.obs_scales.lin_vel,
                                      (self.avg_y_vel * self.lin_vel_scales[:, 1]).unsqueeze(
                                          1) * self.obs_scales.lin_vel,
                                      self.avg_yaw_vel.unsqueeze(1) * self.obs_scales.ang_vel,
                                      self.reaching_target
                                      ), dim=-1)

        elif self.obs_profile == "acc":

            base_height = torch.mean(torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements, dim=1)
            # print(base_height)

            self.obs_buf = torch.cat((self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                                      self.base_ang_vel * self.obs_scales.ang_vel,
                                      self.projected_gravity,
                                      self.commands[:, :3] * self.commands_scale,
                                      (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                      self.dof_vel * self.obs_scales.dof_vel,
                                      # self.delayed_dofvel * self.obs_scales.dof_vel,
                                      self.actions,
                                      torch.sin(2 * torch.pi * self.gait_phase),
                                      torch.cos(2 * torch.pi * self.gait_phase),
                                      self.phase_ratio,
                                      base_height.unsqueeze(1)
                                      ), dim=-1)

        elif self.obs_profile == "min-osu-contact":
            left_contact = self.contact_forces[:, self.feet_indices][:, 0, 2] > 10.
            right_contact = self.contact_forces[:, self.feet_indices][:, 1, 2] > 10.
            self.obs_buf = torch.cat((
                self.base_ang_vel * self.obs_scales.ang_vel,
                self.projected_gravity,
                self.commands[:, :3] * self.commands_scale,
                (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                self.dof_vel * self.obs_scales.dof_vel,
                self.actions,
                torch.sin(2 * torch.pi * self.gait_phase),
                torch.cos(2 * torch.pi * self.gait_phase),
                self.phase_ratio,
                left_contact.unsqueeze(1), right_contact.unsqueeze(1)
            ), dim=-1)

        elif self.obs_profile == "rma-teacher":
            self.obs_buf = torch.cat(
                (
                    self.base_ang_vel * self.obs_scales.ang_vel,
                    self.projected_gravity,
                    self.commands[:, :3] * self.commands_scale,
                    (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                    self.dof_vel * self.obs_scales.dof_vel,
                    self.actions,
                    torch.sin(2 * torch.pi * self.gait_phase),
                    torch.cos(2 * torch.pi * self.gait_phase),
                    self.phase_ratio
                ), dim=-1)
            # privileged obs
            base_height = torch.mean(self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1)
            left_contact = self.contact_forces[:, self.feet_indices][:, 0, 2] > 10.
            right_contact = self.contact_forces[:, self.feet_indices][:, 1, 2] > 10.
            self.privileged_obs_buf = torch.cat(
                (
                    base_height.unsqueeze(1),
                    self.base_lin_vel,
                    self.friction.unsqueeze(1),
                    left_contact.unsqueeze(1),
                    right_contact.unsqueeze(1),
                    self.pevils_mass.unsqueeze(1),
                    self.pevils_com,
                    self.motor_strength
                ), dim=-1)

        elif self.obs_profile == "GR1":
            # print(self.root_states[:, 2])
            base_height = torch.mean(torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements, dim=1)
            # print(base_height)
            # print(self.dof_vel[:,17])

            # self.commands_scale[2]= 0.5
            # self.actions[:,[5,11]] =0.0
            # print(self.actions[:,[5,11]])
            self.avg_x_vel = 2 * self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 0] + (
                    1 - 2 * self.dt / self.cfg.commands.gait_cycle) * self.avg_x_vel
            self.avg_y_vel = self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 1] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_y_vel
            self.avg_yaw_vel = self.dt / self.cfg.commands.gait_cycle * self.base_ang_vel[:, 2] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_yaw_vel
            self.obs_buf = torch.cat(
                (
                    self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                    self.base_ang_vel * self.obs_scales.ang_vel,
                    self.projected_gravity,
                    self.commands[:, :3] * self.commands_scale,
                    (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                    # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                    self.dof_vel * self.obs_scales.dof_vel,
                    # self.delayed_dofvel * self.obs_scales.dof_vel,
                    self.actions,
                    torch.sin(2 * torch.pi * self.gait_phase),
                    torch.cos(2 * torch.pi * self.gait_phase),
                    self.phase_ratio,
                    base_height.unsqueeze(1),
                    (self.avg_x_vel * self.lin_vel_scales[:, 0]).unsqueeze(1) * self.obs_scales.lin_vel,
                    (self.avg_y_vel * self.lin_vel_scales[:, 1]).unsqueeze(1) * self.obs_scales.lin_vel,
                    self.avg_yaw_vel.unsqueeze(1) * self.obs_scales.ang_vel
                ), dim=-1)

        elif self.obs_profile == "GR1L":
            base_height = torch.mean(torch.clip(
                self.root_states[:, 2].unsqueeze(1) - self.cfg.rewards.base_height_target - self.measured_heights, -1,
                1.) * self.obs_scales.height_measurements, dim=1)
            self.avg_x_vel = 2 * self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 0] + (
                    1 - 2 * self.dt / self.cfg.commands.gait_cycle) * self.avg_x_vel
            self.avg_y_vel = self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 1] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_y_vel
            self.avg_yaw_vel = self.dt / self.cfg.commands.gait_cycle * self.base_ang_vel[:, 2] + (
                    1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_yaw_vel

            self.obs_buf = torch.cat(
                (
                    self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                    self.base_ang_vel * self.obs_scales.ang_vel,
                    self.projected_gravity,
                    self.commands[:, :3] * self.commands_scale,
                    (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                    # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                    self.dof_vel * self.obs_scales.dof_vel,
                    # self.delayed_dofvel * self.obs_scales.dof_vel,
                    self.actions,
                    torch.sin(2 * torch.pi * self.gait_phase),
                    torch.cos(2 * torch.pi * self.gait_phase),
                    self.phase_ratio,
                    base_height.unsqueeze(1),
                    (self.avg_x_vel * self.lin_vel_scales[:, 0]).unsqueeze(1) * self.obs_scales.lin_vel,
                    (self.avg_y_vel * self.lin_vel_scales[:, 1]).unsqueeze(1) * self.obs_scales.lin_vel,
                    self.avg_yaw_vel.unsqueeze(1) * self.obs_scales.ang_vel
                ), dim=-1)

    def computer_observation_noise(self):
        # add noise if needed
        if self.cfg.noise.add_noise:
            self.obs_buf += (2 * torch.rand_like(self.obs_buf) - 1) * self.noise_scale_vec

    def computer_noise_scale_vec(self):
        """ Sets a vector used to scale the noise added to the observations.
            [NOTE]: Must be adapted when changing the observations structure

        Args:
            cfg (Dict): Environment config file

        Returns:
            [torch.Tensor]: Vector of scales used to multiply a uniform distribution in [-1, 1]
        """
        noise_vec = torch.zeros_like(self.obs_buf[0])
        noise_scales = self.cfg.noise.noise_scales
        noise_level = self.cfg.noise.noise_level

        noise_vec = self.computer_noise_scale_vec_profile(noise_vec, noise_scales, noise_level)

        return noise_vec

    def computer_noise_scale_vec_profile(self, noise_vec, noise_scales, noise_level):

        if self.cfg.env.obs_profile == "full-osu":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:24] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[24:36] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[36:48] = 0.  # previous actions
            noise_vec[48:54] = 0.  # leg phase and ratio
            if self.cfg.terrain.measure_heights:
                noise_vec[
                54:self.cfg.env.num_observations] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements

        elif self.cfg.env.obs_profile == "full":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:24] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[24:36] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[36:48] = 0.  # previous actions
            if self.cfg.terrain.measure_heights:
                noise_vec[
                48:self.cfg.env.num_observations] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements

        elif self.cfg.env.obs_profile == "min-osu":
            noise_vec[:3] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[3:6] = noise_scales.gravity * noise_level
            noise_vec[6:9] = 0.  # commands
            noise_vec[9:21] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[21:33] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[33:45] = 0.  # previous actions
            noise_vec[45:51] = 0.  # leg phase and ratio

        elif self.cfg.env.obs_profile == "vs-rma-teacher":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:24] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[24:36] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[36:48] = 0.  # previous actions
            noise_vec[48:54] = 0.  # leg phase and ratio
            noise_vec[54] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements
            noise_vec[55:71] = 0.

        elif self.cfg.env.obs_profile == "vs-rma-teacher-2":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:24] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[24:36] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[36:48] = 0.  # previous actions
            noise_vec[48:54] = 0.  # leg phase and ratio
            noise_vec[54] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements

        elif self.cfg.env.obs_profile == "vs-rma-teacher-3":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:35] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[35:58] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[58:81] = 0.  # previous actions
            noise_vec[81:87] = 0.  # leg phase and ratio
            noise_vec[87] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements
            noise_vec[88:90] = 0.

        elif self.cfg.env.obs_profile == "sonnie-2":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:35] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[35:58] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[58:81] = 0.  # previous actions
            noise_vec[81:87] = 0.  # leg phase and ratio
            noise_vec[87] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements
            noise_vec[88:91] = 0.

        elif self.cfg.env.obs_profile == "sonnie-gym":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:35] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[35:58] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[58:81] = 0.  # previous actions
            noise_vec[81:202] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements

        elif self.cfg.env.obs_profile == "sonnie-reaching":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:35] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[35:58] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[58:81] = 0.  # previous actions
            noise_vec[81:87] = 0.  # leg phase and ratio
            noise_vec[87] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements
            noise_vec[88:97] = 0.

        elif self.cfg.env.obs_profile == "min-osu-contact":
            noise_vec[:3] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[3:6] = noise_scales.gravity * noise_level
            noise_vec[6:9] = 0.  # commands
            noise_vec[9:21] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[21:33] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[33:45] = 0.  # previous actions
            noise_vec[45:53] = 0.  # leg phase and ratio and contact

        elif self.cfg.env.obs_profile == "rma-teacher":
            noise_vec[:3] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[3:6] = noise_scales.gravity * noise_level
            noise_vec[6:9] = 0.  # commands
            noise_vec[9:21] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[21:33] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[33:45] = 0.  # previous actions
            noise_vec[45:51] = 0.  # leg phase and ratio

        elif self.cfg.env.obs_profile == "GR1":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:35] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[35:58] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[58:81] = 0.  # previous actions
            noise_vec[81:87] = 0.  # leg phase and ratio
            noise_vec[87] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements
            noise_vec[88:91] = 0.

        elif self.cfg.env.obs_profile == "GR1L":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:32] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[32:52] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[52:72] = 0.  # previous actions
            noise_vec[72:78] = 0.  # leg phase and ratio
            noise_vec[78] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements
            noise_vec[79:82] = 0.

        return noise_vec

    def get_amp_observations(self):
        self.left_leg_pose = self.dof_pos[:, :6]
        self.right_leg_pos = self.dof_pos[:, 6:12]
        self.left_leg_vel = self.dof_vel[:, :6]
        self.right_leg_vel = self.dof_vel[:, 6:12]
        self.amp_obs = torch.cat((self.right_leg_pos, self.left_leg_pose, self.right_leg_vel, self.left_leg_vel),
                                 dim=-1)
        return self.amp_obs

    def create_sim(self):
        """ Creates simulation, terrain and evironments
        """
        self.up_axis_idx = 2  # 2 for z, 1 for y -> adapt gravity accordingly
        self.sim = self.gym.create_sim(self.sim_device_id, self.graphics_device_id, self.physics_engine,
                                       self.sim_params)

        mesh_type = self.cfg.terrain.mesh_type
        if mesh_type in ['heightfield', 'trimesh']:
            self.terrain = Terrain(self.cfg.terrain, self.num_envs)
        if mesh_type == 'plane':
            self._create_ground_plane()
        elif mesh_type == 'heightfield':
            self._create_heightfield()
        elif mesh_type == 'trimesh':
            self._create_trimesh()
        elif mesh_type is not None:
            raise ValueError("Terrain mesh type not recognised. Allowed types are [None, plane, heightfield, trimesh]")

        self._create_envs()

    def set_camera(self, position, lookat):
        """ Set camera position and direction
        """
        cam_pos = gymapi.Vec3(position[0], position[1], position[2])
        cam_target = gymapi.Vec3(lookat[0], lookat[1], lookat[2])
        self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

    # ------------- Callbacks --------------
    def _process_rigid_shape_props(self, props, env_id):
        """ Callback allowing to store/change/randomize the rigid shape properties of each environment.
            Called During environment creation.
            Base behavior: randomizes the friction of each environment
        Args:
            props (List[gymapi.RigidShapeProperties]): Properties of each shape of the asset
            env_id (int): Environment id
        Returns:
            [List[gymapi.RigidShapeProperties]]: Modified rigid shape properties
        """
        if self.cfg.domain_rand.randomize_friction:
            if env_id == 0:
                # prepare friction randomization
                friction_range = self.cfg.domain_rand.friction_range
                restituion_range = self.cfg.domain_rand.restitution_range
                num_buckets = 64
                bucket_ids = torch.randint(0, num_buckets, (self.num_envs, 1))
                friction_buckets = torch_rand_float(friction_range[0], friction_range[1], (num_buckets, 1),
                                                    device='cpu')
                self.friction_coeffs = friction_buckets[bucket_ids]
                restitution_buckets = torch_rand_float(restituion_range[0], restituion_range[1], (num_buckets, 1),
                                                       device='cpu')
                self.restitution_coeffs = restitution_buckets[bucket_ids]

            for s in range(len(props)):
                props[s].friction = self.friction_coeffs[env_id]
                props[s].restitution = self.restitution_coeffs[env_id]
        return props

    def _process_dof_props(self, props, env_id):
        """ Callback allowing to store/change/randomize the DOF properties of each environment.
            Called During environment creation.
            Base behavior: stores position, velocity and torques limits defined in the URDF
        Args:
            props (numpy.array): Properties of each DOF of the asset
            env_id (int): Environment id
        Returns:
            [numpy.array]: Modified DOF properties
        """
        if env_id == 0:
            self.dof_pos_limits = torch.zeros(self.num_dof, 2, dtype=torch.float, device=self.device,
                                              requires_grad=False)
            self.dof_vel_limits = torch.zeros(self.num_dof, dtype=torch.float, device=self.device, requires_grad=False)
            self.torque_limits = torch.zeros(self.num_dof, dtype=torch.float, device=self.device, requires_grad=False)
            for i in range(len(props)):
                self.dof_pos_limits[i, 0] = props["lower"][i].item()
                self.dof_pos_limits[i, 1] = props["upper"][i].item()
                self.dof_vel_limits[i] = props["velocity"][i].item()
                self.torque_limits[i] = props["effort"][i].item()
                # soft limits
                m = (self.dof_pos_limits[i, 0] + self.dof_pos_limits[i, 1]) / 2
                r = self.dof_pos_limits[i, 1] - self.dof_pos_limits[i, 0]
                self.dof_pos_limits[i, 0] = m - 0.5 * r * self.cfg.rewards.soft_dof_pos_limit
                self.dof_pos_limits[i, 1] = m + 0.5 * r * self.cfg.rewards.soft_dof_pos_limit
        return props

    def _process_rigid_body_props(self, props, env_id):
        # if env_id==0:
        #     sum = 0
        #     for i, p in enumerate(props):
        #         sum += p.mass
        #         print(f"Mass of body {i}: {p.mass} (before randomization)")
        #     print(f"Total mass {sum} (before randomization)")
        # randomize base mass
        rng = self.cfg.domain_rand.added_mass_range
        rng_com_x = self.cfg.domain_rand.added_com_range_x
        rng_com_y = self.cfg.domain_rand.added_com_range_y
        rng_com_z = self.cfg.domain_rand.added_com_range_z

        if self.cfg.domain_rand.randomize_base_mass:
            props[0].mass += props[0].mass * np.random.uniform(rng[0], rng[1])
            props[0].invMass = 1 / props[0].mass

        if self.cfg.domain_rand.randomize_base_com:
            # print(torch.ones_like(props[0].com))
            props[0].com += gymapi.Vec3(np.random.uniform(rng_com_x[0], rng_com_x[1]),
                                        np.random.uniform(rng_com_y[0], rng_com_y[1]),
                                        np.random.uniform(rng_com_z[0], rng_com_z[1]))

        if self.cfg.domain_rand.randomize_thigh_mass:
            props[self.thigh_indices[0]].mass += props[self.thigh_indices[0]].mass * np.random.uniform(rng[0], rng[1])
            props[self.thigh_indices[1]].mass += props[self.thigh_indices[1]].mass * np.random.uniform(rng[0], rng[1])
            props[self.thigh_indices[0]].invMass = 1 / props[self.thigh_indices[0]].mass
            props[self.thigh_indices[1]].invMass = 1 / props[self.thigh_indices[1]].mass

        if self.cfg.domain_rand.randomize_shin_mass:
            props[self.shin_indices[0]].mass += props[self.shin_indices[0]].mass * np.random.uniform(rng[0], rng[1])
            props[self.shin_indices[1]].mass += props[self.shin_indices[1]].mass * np.random.uniform(rng[0], rng[1])
            props[self.shin_indices[0]].invMass = 1 / props[self.shin_indices[0]].mass
            props[self.shin_indices[1]].invMass = 1 / props[self.shin_indices[1]].mass

        if self.cfg.domain_rand.randomize_torso_mass:
            props[self.torso_indices].mass += props[self.torso_indices[0]].mass * np.random.uniform(rng[0], rng[1])
            props[self.torso_indices].invMass = 1 / props[self.torso_indices[0]].mass

        if self.cfg.domain_rand.randomize_upper_arm_mass:
            props[self.upper_arm_indices[0]].mass += props[self.upper_arm_indices[0]].mass * np.random.uniform(rng[0],
                                                                                                               rng[1])
            props[self.upper_arm_indices[1]].mass += props[self.upper_arm_indices[1]].mass * np.random.uniform(rng[0],
                                                                                                               rng[1])
            props[self.upper_arm_indices[0]].invMass = 1 / props[self.upper_arm_indices[0]].mass
            props[self.upper_arm_indices[1]].invMass = 1 / props[self.upper_arm_indices[1]].mass

        if self.cfg.domain_rand.randomize_lower_arm_mass:
            props[self.lower_arm_indices[0]].mass += props[self.lower_arm_indices[0]].mass * np.random.uniform(rng[0],
                                                                                                               rng[1])
            props[self.lower_arm_indices[1]].mass += props[self.lower_arm_indices[1]].mass * np.random.uniform(rng[0],
                                                                                                               rng[1])
            props[self.lower_arm_indices[0]].invMass = 1 / props[self.lower_arm_indices[0]].mass
            props[self.lower_arm_indices[1]].invMass = 1 / props[self.lower_arm_indices[1]].mass

        return props

    def _post_physics_step_callback(self):
        """ Callback called before computing terminations, rewards, and observations
            Default behaviour: Compute ang vel command based on target and heading, compute measured terrain heights and randomly push robots
        """
        # 
        env_ids = (self.episode_length_buf % int(self.cfg.commands.resampling_time / self.dt) == 0).nonzero(
            as_tuple=False).flatten()
        self._resample_commands(env_ids,
                                self.cfg.commands.resample_command_profile,
                                self.cfg.commands.resample_command_profile_randomize,
                                self.cfg.commands.resample_command_log)

        if self.cfg.commands.heading_command:
            forward = quat_apply(self.base_quat, self.forward_vec)
            heading = torch.atan2(forward[:, 1], forward[:, 0])
            self.commands[:, 2] = torch.clip(0.5 * wrap_to_pi(self.commands[:, 3] - heading),
                                             self.command_ranges["ang_vel_yaw"][0],
                                             self.command_ranges["ang_vel_yaw"][1])

        if self.cfg.terrain.measure_heights:
            self.measured_heights = self._get_heights()
            if self.cfg.terrain.blind_test:
                self.measured_heights_blind = self._get_heights_blind()

        if self.cfg.domain_rand.push_robots and (self.common_step_counter % self.cfg.domain_rand.push_interval == 0):
            self._push_robots()

        if self.cfg.domain_rand.apply_forces and (self.common_step_counter % self.cfg.domain_rand.push_interval == 0):
            self.continuous_force_step_counter = 0

        if self.common_step_counter > self.cfg.domain_rand.push_interval and self.cfg.domain_rand.apply_forces and self.continuous_force_step_counter * self.dt < self.cfg.domain_rand.continue_time_s:
            self._apply_robots_forces(self.ex_forces, self.ex_torques)
            # print("apply force")
            # print(self.ex_torques)

        self.continuous_force_step_counter += 1
        self._calculate_gait_phase()
        # lefthand = self.rigid_body_states[:,self.lower_arm_indices][:, 0, :3]- self.root_states[:,0:3] + quat_rotate(self.rigid_body_states[:,self.lower_arm_indices][:, 0, 3:7],self.arm_left_local_vec) 
        # righthand = self.rigid_body_states[:,self.lower_arm_indices][:, 1, :3]- self.root_states[:,0:3] + quat_rotate(self.rigid_body_states[:,self.lower_arm_indices][:, 1, 3:7],self.arm_right_local_vec) 
        # lefthand = quat_apply(quat_conjugate(self.root_states[:,3:7]),lefthand)
        # righthand = quat_apply(quat_conjugate(self.root_states[:,3:7]),righthand)

        # erro_left = torch.sum(torch.square(lefthand-self.reaching_target[:,:3]),dim=1)
        # erro_right = torch.sum(torch.square(righthand-self.reaching_target[:,3:6]),dim=1)

        # env_ids_reached_left = (erro_left < 0.0001).nonzero(as_tuple=False).flatten()
        # env_ids_reached_right = (erro_right < 0.0001).nonzero(as_tuple=False).flatten()
        # self._resample_reaching_target(env_ids_reached_left,env_ids_reached_right)

    def _calculate_gait_phase(self):
        # print(self.dt)
        # change gait_cycle
        # u0 = (torch.abs(self.avg_x_vel) - 0.75)/0.75
        # u0 = (1.0 - u0.clip(min=0,max=1.0))*(1.0 - self.cfg.commands.gait_cycle_percent) + self.cfg.commands.gait_cycle_percent
        # left_gait_phase = self.episode_length_buf*self.dt/(self.cfg.commands.gait_cycle*u0)- torch.floor(self.episode_length_buf*self.dt/(self.cfg.commands.gait_cycle*u0)) + self.cfg.commands.theta_left
        # right_gait_phase = self.episode_length_buf*self.dt/(self.cfg.commands.gait_cycle*u0) - torch.floor(self.episode_length_buf*self.dt/(self.cfg.commands.gait_cycle*u0)) + self.cfg.commands.theta_right

        # left_gait_phase = self.episode_length_buf * self.dt / self.cfg.commands.gait_cycle - torch.floor(
        #     self.episode_length_buf * self.dt / self.cfg.commands.gait_cycle) + self.theta_left
        # right_gait_phase = self.episode_length_buf * self.dt / self.cfg.commands.gait_cycle - torch.floor(
        #     self.episode_length_buf * self.dt / self.cfg.commands.gait_cycle) + self.theta_right

        left_gait_phase = self.episode_length_buf * self.dt / self.cfg.commands.gait_cycle - torch.floor(
            self.episode_length_buf * self.dt / self.cfg.commands.gait_cycle) + (self.rand_init_phases==1)*self.theta_left + \
            (self.rand_init_phases==-1)*self.theta_right
        right_gait_phase = self.episode_length_buf * self.dt / self.cfg.commands.gait_cycle - torch.floor(
            self.episode_length_buf * self.dt / self.cfg.commands.gait_cycle) + (self.rand_init_phases==1)*self.theta_right + \
            (self.rand_init_phases==-1)*self.theta_left
        left_gait_phase -= torch.floor(left_gait_phase)
        right_gait_phase -= torch.floor(right_gait_phase)

        self.gait_phase[:, 0] = left_gait_phase
        self.gait_phase[:, 1] = right_gait_phase

        # print("self.episode_length_buf")
        # print(self.episode_length_buf)
        # print("self.left_gait_phase")
        # print(left_gait_phase)
        # print(self.gait_phase)

    def _resample_reaching_target(self, env_ids_reached_left, env_ids_reached_right):
        # print(env_ids_reached_left)
        self.reaching_target[env_ids_reached_left, 0] = torch_rand_float(self.command_ranges["reaching_x"][0],
                                                                         self.command_ranges["reaching_x"][1],
                                                                         (len(env_ids_reached_left), 1),
                                                                         device=self.device).squeeze(1)
        self.reaching_target[env_ids_reached_left, 1] = torch_rand_float(self.command_ranges["reaching_y"][0],
                                                                         self.command_ranges["reaching_y"][1],
                                                                         (len(env_ids_reached_left), 1),
                                                                         device=self.device).squeeze(1)
        self.reaching_target[env_ids_reached_left, 2] = torch_rand_float(self.command_ranges["reaching_z"][0],
                                                                         self.command_ranges["reaching_z"][1],
                                                                         (len(env_ids_reached_left), 1),
                                                                         device=self.device).squeeze(1)
        self.reaching_target[env_ids_reached_left, :3] += self.root_states[env_ids_reached_left, :3]
        self.reaching_target[env_ids_reached_right, 3] = torch_rand_float(self.command_ranges["reaching_x"][0],
                                                                          self.command_ranges["reaching_x"][1],
                                                                          (len(env_ids_reached_right), 1),
                                                                          device=self.device).squeeze(1)
        self.reaching_target[env_ids_reached_right, 4] = -torch_rand_float(self.command_ranges["reaching_y"][0],
                                                                           self.command_ranges["reaching_y"][1],
                                                                           (len(env_ids_reached_right), 1),
                                                                           device=self.device).squeeze(1)
        self.reaching_target[env_ids_reached_right, 5] = torch_rand_float(self.command_ranges["reaching_z"][0],
                                                                          self.command_ranges["reaching_z"][1],
                                                                          (len(env_ids_reached_right), 1),
                                                                          device=self.device).squeeze(1)
        self.reaching_target[env_ids_reached_right, 3:6] += self.root_states[env_ids_reached_right, :3]

    def _resample_commands(self,
                           env_ids,
                           resample_command_profile=None,
                           resample_command_profile_randomize=False,
                           resample_command_log=False):
        """ Randommly select commands of some environments
        Args:
            env_ids (List[int]): Environments ids for which new commands are needed
        """
        self.commands[env_ids, 0] = torch_rand_float(self.command_ranges["lin_vel_x"][0],
                                                     self.command_ranges["lin_vel_x"][1], (len(env_ids), 1),
                                                     device=self.device).squeeze(1)
        self.commands[env_ids, 1] = torch_rand_float(self.command_ranges["lin_vel_y"][0],
                                                     self.command_ranges["lin_vel_y"][1], (len(env_ids), 1),
                                                     device=self.device).squeeze(1)

        # set small commands to zero
        self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.05).unsqueeze(1)

        if self.cfg.commands.heading_command:
            self.commands[env_ids, 3] = torch_rand_float(self.command_ranges["heading"][0],
                                                         self.command_ranges["heading"][1], (len(env_ids), 1),
                                                         device=self.device).squeeze(1)
        else:
            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges["ang_vel_yaw"][0],
                                                         self.command_ranges["ang_vel_yaw"][1], (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

        # gait phase
        self.phase_ratio[env_ids, 0] = self.cfg.commands.left_phase_ratio
        self.phase_ratio[env_ids, 1] = self.cfg.commands.right_phase_ratio
        self.theta_left[env_ids] = self.cfg.commands.theta_left
        self.theta_right[env_ids] = self.cfg.commands.theta_right

    def _compute_torques(self, actions):
        """ Compute torques from actions.
            Actions can be interpreted as position or velocity targets given to a PD controller, or directly as scaled torques.
            [NOTE]: torques must have the same dimension as the number of DOFs, even if some DOFs are not actuated.
        Args:
            actions (torch.Tensor): Actions
        Returns:
            [torch.Tensor]: Torques sent to the simulation
        """
        # pd controller
        actions_scaled = actions * self.cfg.control.action_scale
        # print(self.default_dof_pos)
        control_type = self.cfg.control.control_type
        if control_type == "P":
            torques = self.p_gains_rand * (
                    actions_scaled + self.default_dof_pos - self.dof_pos) - self.d_gains_rand * self.dof_vel
        elif control_type == "V":
            torques = self.p_gains_rand * (actions_scaled - self.dof_vel) - self.d_gains_rand * (
                    self.dof_vel - self.last_dof_vel) / self.sim_params.dt
        elif control_type == "T":
            torques = actions_scaled
        else:
            raise NameError(f"Unknown controller type: {control_type}")
        torques *= self.motor_strength
        # torques[:,[5,11]] *= self.motor_strength[:,[5,11]]
        self.friction_0_6()
        self.friction_1_7()
        self.friction_2_3_8_9()
        # self.friction_arm()
        # self.friction_4_10()
        # self.friction_5_11()
        # print(self.fric)
        torques -= self.fric
        return torch.clip(torques, -self.torque_limits, self.torque_limits)

    def _reset_dofs(self, env_ids):
        """ Resets DOF position and velocities of selected environmments
        Positions are randomly selected within 0.5:1.5 x default positions.
        Velocities are set to zero.
        Args:
            env_ids (List[int]): Environemnt ids
        """
        self.dof_pos[env_ids] = self.default_dof_pos * torch_rand_float(0.5, 1.5, (len(env_ids), self.num_dof),
                                                                        device=self.device)
        # self.dof_pos[env_ids] = self.default_dof_pos * torch_rand_float(1.0, 1.0, (len(env_ids), self.num_dof), device=self.device)
        self.dof_vel[env_ids] = 0.
        self.delayed_dofpos[env_ids] = self.dof_pos[env_ids].clone()
        self.delayed_dofvel[env_ids] = self.dof_vel[env_ids].clone()
        self.delayed_actions[env_ids] = self.actions[env_ids].clone()
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(self.sim,
                                              gymtorch.unwrap_tensor(self.dof_state),
                                              gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))
        """ren xiaoyu : 19940622@126.com
            random PD gains; 
        """
        self.p_gains_rand[env_ids] = self.p_gains * torch_rand_float(0.5, 1.5, (len(env_ids), self.num_dof),
                                                                        device=self.device)
        self.d_gains_rand[env_ids] = self.d_gains * torch_rand_float(0.5, 1.5, (len(env_ids), self.num_dof),
                                                                        device=self.device)
        # print(self.p_gains_rand)
    def _reset_root_states(self, env_ids):
        """ Resets ROOT states position and velocities of selected environmments
            Sets base position based on the curriculum
            Selects randomized base velocities within -0.5:0.5 [m/s, rad/s]
        Args:
            env_ids (List[int]): Environemnt ids
        """
        # base position
        if self.custom_origins:
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
            self.root_states[env_ids, :2] += torch_rand_float(-1., 1., (len(env_ids), 2),
                                                              device=self.device)  # xy position within 1m of the center
        else:
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
        # base velocities
        self.root_states[env_ids, 7:13] = torch_rand_float(-0.5, 0.5, (len(env_ids), 6),
                                                           device=self.device)  # [7:10]: lin vel, [10:13]: ang vel
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(self.sim,
                                                     gymtorch.unwrap_tensor(self.root_states),
                                                     gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))

    def _reset_dofs_amp(self, env_ids, frames):
        """ Resets DOF position and velocities of selected environmments
        Positions are randomly selected within 0.5:1.5 x default positions.
        Velocities are set to zero.

        Args:
            env_ids (List[int]): Environemnt ids
            frames: AMP frames to initialize motion with
        """
        self.dof_pos[env_ids, :12] = AMPLoader.get_joint_pose_batch(frames)
        self.dof_vel[env_ids, :12] = AMPLoader.get_joint_vel_batch(frames)
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(self.sim,
                                              gymtorch.unwrap_tensor(self.dof_state),
                                              gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))

    def _reset_root_states_amp(self, env_ids, frames):
        """ Resets ROOT states position and velocities of selected environmments
            Sets base position based on the curriculum
            Selects randomized base velocities within -0.5:0.5 [m/s, rad/s]
        Args:
            env_ids (List[int]): Environemnt ids
        """
        # base position
        root_pos = AMPLoader.get_root_pos_batch(frames)
        root_pos[:, :2] = root_pos[:, :2] + self.env_origins[env_ids, :2]
        self.root_states[env_ids, :3] = root_pos
        root_orn = AMPLoader.get_root_rot_batch(frames)
        self.root_states[env_ids, 3:7] = root_orn
        self.root_states[env_ids, 7:10] = quat_rotate(root_orn, AMPLoader.get_linear_vel_batch(frames))
        self.root_states[env_ids, 10:13] = quat_rotate(root_orn, AMPLoader.get_angular_vel_batch(frames))

        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(self.sim,
                                                     gymtorch.unwrap_tensor(self.root_states),
                                                     gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))

    def _push_robots(self):
        """ Random pushes the robots. Emulates an impulse by setting a randomized base velocity.
        """
        max_vel = self.cfg.domain_rand.max_push_vel_xy
        self.root_states[:, 7:9] = torch_rand_float(-max_vel, max_vel, (self.num_envs, 2),
                                                    device=self.device)  # lin vel x/y
        self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(self.root_states))

    def _apply_robots_forces(self, forces, torques):
        """ Random pushes the robots. Emulates an impulse by setting a randomized base velocity.
        """

        self.gym.apply_rigid_body_force_tensors(self.sim, gymtorch.unwrap_tensor(forces),
                                                gymtorch.unwrap_tensor(torques), gymapi.ENV_SPACE)

    def _update_terrain_curriculum(self, env_ids):
        """ Implements the game-inspired curriculum.

        Args:
            env_ids (List[int]): ids of environments being reset
        """
        # Implement Terrain curriculum
        if not self.init_done:
            return

        distance = torch.norm(self.root_states[env_ids, :2] - self.env_origins[env_ids, :2], dim=1)

        # robots that walked far enough progress to harder terains
        move_up = distance > self.terrain.env_length / 2

        # robots that walked less than half of their required distance go to simpler terrains
        move_down = (distance < torch.norm(self.commands[env_ids, :2],
                                           dim=1) * self.max_episode_length_s * 0.5) * ~move_up
        self.terrain_levels[env_ids] += 1 * move_up - 1 * move_down

        # Robots that solve the last level are sent to a random one
        self.terrain_levels[env_ids] = torch.where(self.terrain_levels[env_ids] >= self.max_terrain_level,
                                                   torch.randint_like(self.terrain_levels[env_ids],
                                                                      self.max_terrain_level),
                                                   torch.clip(self.terrain_levels[env_ids],
                                                              0))  # (the minumum level is zero)
        self.env_origins[env_ids] = self.terrain_origins[self.terrain_levels[env_ids], self.terrain_types[env_ids]]

    def update_command_curriculum(self, env_ids):
        """ Implements a curriculum of increasing commands

        Args:
            env_ids (List[int]): ids of environments being reset
        """
        # If the tracking reward is above 80% of the maximum, increase the range of commands
        if (torch.mean(self.episode_sums["tracking_lin_vel"][env_ids]) / self.max_episode_length
                > 0.8 * self.reward_scales["tracking_lin_vel"]):
            self.command_ranges["lin_vel_x"][0] = np.clip(self.command_ranges["lin_vel_x"][0] - 0.5,
                                                          -self.cfg.commands.max_curriculum,
                                                          0.0)
            self.command_ranges["lin_vel_x"][1] = np.clip(self.command_ranges["lin_vel_x"][1] + 0.5,
                                                          0.0,
                                                          self.cfg.commands.max_curriculum)

    # ----------------------------------------
    def _init_buffers(self):
        """ Initialize torch tensors which will contain simulation states and processed quantities
        """
        # get gym GPU state tensors
        actor_root_state = self.gym.acquire_actor_root_state_tensor(self.sim)
        dof_state_tensor = self.gym.acquire_dof_state_tensor(self.sim)
        net_contact_forces = self.gym.acquire_net_contact_force_tensor(self.sim)
        rigid_body_state_tensor = self.gym.acquire_rigid_body_state_tensor(self.sim)
        sensor_tensor = self.gym.acquire_force_sensor_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_net_contact_force_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.gym.refresh_force_sensor_tensor(self.sim)
        # create some wrapper tensors for different slices
        self.root_states = gymtorch.wrap_tensor(actor_root_state)
        self.dof_state = gymtorch.wrap_tensor(dof_state_tensor)
        self.dof_pos = self.dof_state.view(self.num_envs, self.num_dof, 2)[..., 0]
        self.dof_vel = self.dof_state.view(self.num_envs, self.num_dof, 2)[..., 1]
        self.delayed_dofpos = torch.zeros(self.num_envs, self.num_dof,
                                          dtype=torch.float, device=self.device, requires_grad=False)
        self.delayed_dofvel = torch.zeros(self.num_envs, self.num_dof,
                                          dtype=torch.float, device=self.device, requires_grad=False)
        self.base_quat = self.root_states[:, 3:7]

        # shape: num_envs, num_bodies, xyz axis
        self.contact_forces = gymtorch.wrap_tensor(net_contact_forces).view(self.num_envs, -1, 3)
        self.rigid_body_states = gymtorch.wrap_tensor(rigid_body_state_tensor).view(self.num_envs, -1, 13)
        self.force_sensor_readings = gymtorch.wrap_tensor(sensor_tensor).view(self.num_envs, -1, 6)
        # self.footstates = self.rigid_body_states[:,self.feet_indices][:, :, 0:13]
        # self.foot_pos = self.rigid_body_states[:,self.feet_indices][:, :, 0:3]
        # self.foot_orien = self.rigid_body_states[:,self.feet_indices][:, :, 3:7]
        # self.left_foot_orien = quat_mul(self.base_quat,self.foot_orien[:,0,:])
        # initialize some data used later on
        self.common_step_counter = 0
        self.extras = {}
        self.noise_scale_vec = self.computer_noise_scale_vec()
        self.gravity_vec = to_torch(get_axis_params(-1., self.up_axis_idx), device=self.device).repeat(
            (self.num_envs, 1))
        self.forward_vec = to_torch([1., 0., 0.], device=self.device).repeat((self.num_envs, 1))
        self.arm_left_local_vec = to_torch([0., 0.2, 0.], device=self.device).repeat((self.num_envs, 1))
        self.arm_right_local_vec = to_torch([0., -0.2, 0.], device=self.device).repeat((self.num_envs, 1))
        self.torques = torch.zeros(self.num_envs, self.num_actions,
                                   dtype=torch.float, device=self.device, requires_grad=False)
        self.fric_para_0 = torch.zeros(self.num_envs, 5,
                                       dtype=torch.float, device=self.device, requires_grad=False)
        self.fric_para_1 = torch.zeros(self.num_envs, 5,
                                       dtype=torch.float, device=self.device, requires_grad=False)
        self.fric_para_2 = torch.zeros(self.num_envs, 5,
                                       dtype=torch.float, device=self.device, requires_grad=False)
        self.fric_para_3 = torch.zeros(self.num_envs, 5,
                                       dtype=torch.float, device=self.device, requires_grad=False)
        self.fric = torch.zeros(self.num_envs, self.num_actions,
                                dtype=torch.float, device=self.device, requires_grad=False)
        self.p_gains = torch.zeros(self.num_actions,
                                   dtype=torch.float, device=self.device, requires_grad=False)
        self.d_gains = torch.zeros(self.num_actions,
                                   dtype=torch.float, device=self.device, requires_grad=False)
        self.p_gains_rand = torch.zeros(self.num_envs, self.num_actions,
                                   dtype=torch.float, device=self.device, requires_grad=False)
        self.d_gains_rand = torch.zeros(self.num_envs, self.num_actions,
                                   dtype=torch.float, device=self.device, requires_grad=False)
        self.actions = torch.zeros(self.num_envs, self.num_actions,
                                   dtype=torch.float, device=self.device, requires_grad=False)
        self.actions_filter = torch.zeros(self.num_envs, self.num_actions,
                                   dtype=torch.float, device=self.device, requires_grad=False)
        self.delayed_actions = torch.zeros(self.num_envs, self.num_actions,
                                           dtype=torch.float, device=self.device, requires_grad=False)
        self.last_actions = torch.zeros(self.num_envs, self.num_actions,
                                        dtype=torch.float, device=self.device, requires_grad=False)
        self.last_dof_vel = torch.zeros_like(self.dof_vel)
        self.last_root_vel = torch.zeros_like(self.root_states[:, 7:13])

        # commands
        self.commands = torch.zeros(self.num_envs, self.cfg.commands.num_commands,
                                    dtype=torch.float, device=self.device, requires_grad=False)
        self.commands_scale = torch.tensor([self.obs_scales.lin_vel,
                                            self.obs_scales.lin_vel,
                                            self.obs_scales.ang_vel],
                                           device=self.device, requires_grad=False, )  # TODO change this

        # reaching target
        self.reaching_target = torch.zeros(self.num_envs, 6,
                                           dtype=torch.float, device=self.device, requires_grad=False)

        # options
        self.num_options = self.cfg.commands.num_options
        self.options = torch.zeros(self.num_envs, self.cfg.commands.num_options,
                                   dtype=torch.float, device=self.device, requires_grad=False)

        # gait phase
        self.gait_phase = torch.zeros(self.num_envs, 2, dtype=torch.float, device=self.device, requires_grad=False)
        self.phase_ratio = torch.ones(self.num_envs, 2, dtype=torch.float, device=self.device, requires_grad=False)
        self.phase_ratio[:, 0] = self.cfg.commands.left_phase_ratio
        self.phase_ratio[:, 1] = self.cfg.commands.right_phase_ratio
        self.theta_left = torch.zeros(self.num_envs, dtype=torch.float, device=self.device, requires_grad=False)
        self.theta_right = torch.zeros(self.num_envs, dtype=torch.float, device=self.device, requires_grad=False)
        self.theta_left[:] = self.cfg.commands.theta_left
        self.theta_right[:] = self.cfg.commands.theta_right
        self.gait_cycle = self.cfg.commands.gait_cycle
        self.rand_init_phases = torch.randint(0,2,(self.num_envs,),dtype=torch.float,device=self.device,requires_grad=False)*2 - 1
        # avg
        self.avg_feet_frc = torch.zeros(self.num_envs, len(self.feet_indices), dtype=torch.float, device=self.device,
                                        requires_grad=False)  # phase_left,phase_right
        self.avg_feet_spd = torch.zeros(self.num_envs, len(self.feet_indices), dtype=torch.float, device=self.device,
                                        requires_grad=False)  # phase_left,phase_right
        self.avg_x_vel = torch.zeros(self.num_envs, dtype=torch.float, device=self.device, requires_grad=False)
        self.avg_y_vel = torch.zeros(self.num_envs, dtype=torch.float, device=self.device, requires_grad=False)
        self.avg_yaw_vel = torch.zeros(self.num_envs, dtype=torch.float, device=self.device, requires_grad=False)
        self.feet_air_time = torch.zeros(self.num_envs, self.feet_indices.shape[0], dtype=torch.float,
                                         device=self.device, requires_grad=False)
        self.continuous_force_step_counter = 0
        self.last_contacts = torch.zeros(self.num_envs, len(self.feet_indices), dtype=torch.bool, device=self.device,
                                         requires_grad=False)
        self.last_feet_air_time = torch.zeros(self.num_envs, self.feet_indices.shape[0], dtype=torch.float,
                                              device=self.device, requires_grad=False)
        self.feet_distance = torch.zeros(self.num_envs, 3, dtype=torch.float, device=self.device, requires_grad=False)
        self.last_feet_distance = torch.zeros(self.num_envs, 3, dtype=torch.float, device=self.device,
                                              requires_grad=False)
        self.base_lin_vel = quat_rotate_inverse(self.base_quat, self.root_states[:, 7:10])
        self.base_ang_vel = quat_rotate_inverse(self.base_quat, self.root_states[:, 10:13])
        self.projected_gravity = quat_rotate_inverse(self.base_quat, self.gravity_vec)
        self.left_foot_orien_projected = quat_rotate_inverse(self.rigid_body_states[:, self.feet_indices][:, 0, 3:7],
                                                             self.gravity_vec)
        self.right_foot_orien_projected = quat_rotate_inverse(self.rigid_body_states[:, self.feet_indices][:, 1, 3:7],
                                                              self.gravity_vec)

        if self.cfg.terrain.measure_heights:
            self.height_points = self._init_height_points()
            # self.height_points_blind = self._init_height_points_blind()
        self.measured_heights = 0

        # joint positions offsets and PD gains
        self.default_dof_pos = torch.zeros(self.num_dof, dtype=torch.float, device=self.device, requires_grad=False)
        for i in range(self.num_dofs):
            name = self.dof_names[i]
            angle = self.cfg.init_state.default_joint_angles[name]
            self.default_dof_pos[i] = angle
            found = False
            for dof_name in self.cfg.control.stiffness.keys():
                if dof_name in name:
                    self.p_gains[i] = self.cfg.control.stiffness[dof_name]
                    self.d_gains[i] = self.cfg.control.damping[dof_name]

                    self.p_gains_rand[:,i] = self.p_gains[i]
                    self.d_gains_rand[:,i] = self.d_gains[i]
                    found = True
            if not found:
                self.p_gains[i] = 0.
                self.d_gains[i] = 0.

                self.p_gains_rand[:,i] = 0.0
                self.d_gains_rand[:,i] = 0.0
                if self.cfg.control.control_type in ["P", "V"]:
                    print(f"PD gain of joint {name} were not defined, setting them to zero")
        self.default_dof_pos = self.default_dof_pos.unsqueeze(0)
    def _prepare_reward_function(self):
        """ Prepares a list of reward functions, whcih will be called to compute the total reward.
            Looks for self._reward_<REWARD_NAME>, where <REWARD_NAME> are names of all non zero reward scales in the cfg.
        """
        # remove zero scales + multiply non-zero ones by dt
        for key in list(self.reward_scales.keys()):
            scale = self.reward_scales[key]
            if scale == 0:
                self.reward_scales.pop(key)
            else:
                self.reward_scales[key] *= self.dt

        # prepare list of functions
        self.reward_functions = []
        self.reward_names = []
        for name, scale in self.reward_scales.items():
            if name == "termination":
                continue
            self.reward_names.append(name)
            name = '_reward_' + name
            self.reward_functions.append(getattr(self, name))

        # reward episode sums
        self.episode_sums = {}
        for name in self.reward_scales.keys():
            self.episode_sums[name] = torch.zeros(self.num_envs,
                                                  dtype=torch.float, device=self.device, requires_grad=False)

    def _create_ground_plane(self):
        """ Adds a ground plane to the simulation, sets friction and restitution based on the cfg.
        """
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        plane_params.static_friction = self.cfg.terrain.static_friction
        plane_params.dynamic_friction = self.cfg.terrain.dynamic_friction
        plane_params.restitution = self.cfg.terrain.restitution
        self.gym.add_ground(self.sim, plane_params)

    def _create_heightfield(self):
        """ Adds a heightfield terrain to the simulation, sets parameters based on the cfg.
        """
        hf_params = gymapi.HeightFieldParams()
        hf_params.column_scale = self.terrain.cfg.horizontal_scale
        hf_params.row_scale = self.terrain.cfg.horizontal_scale
        hf_params.vertical_scale = self.terrain.cfg.vertical_scale
        hf_params.nbRows = self.terrain.tot_cols
        hf_params.nbColumns = self.terrain.tot_rows
        hf_params.transform.p.x = -self.terrain.cfg.border_size
        hf_params.transform.p.y = -self.terrain.cfg.border_size
        hf_params.transform.p.z = 0.0
        hf_params.static_friction = self.cfg.terrain.static_friction
        hf_params.dynamic_friction = self.cfg.terrain.dynamic_friction
        hf_params.restitution = self.cfg.terrain.restitution

        self.gym.add_heightfield(self.sim, self.terrain.heightsamples, hf_params)
        self.height_samples = torch.tensor(self.terrain.heightsamples).view(self.terrain.tot_rows,
                                                                            self.terrain.tot_cols).to(self.device)

    def _create_trimesh(self):
        """ Adds a triangle mesh terrain to the simulation, sets parameters based on the cfg.
        # """
        tm_params = gymapi.TriangleMeshParams()
        tm_params.nb_vertices = self.terrain.vertices.shape[0]
        tm_params.nb_triangles = self.terrain.triangles.shape[0]

        tm_params.transform.p.x = -self.terrain.cfg.border_size
        tm_params.transform.p.y = -self.terrain.cfg.border_size
        tm_params.transform.p.z = 0.0
        tm_params.static_friction = self.cfg.terrain.static_friction
        tm_params.dynamic_friction = self.cfg.terrain.dynamic_friction
        tm_params.restitution = self.cfg.terrain.restitution
        self.gym.add_triangle_mesh(self.sim, self.terrain.vertices.flatten(order='C'),
                                   self.terrain.triangles.flatten(order='C'), tm_params)
        self.height_samples = torch.tensor(self.terrain.heightsamples).view(self.terrain.tot_rows,
                                                                            self.terrain.tot_cols).to(self.device)

    def _create_envs(self):
        """ Creates environments:
             1. loads the robot URDF/MJCF asset,
             2. For each environment
                2.1 creates the environment,
                2.2 calls DOF and Rigid shape properties callbacks,
                2.3 create actor with these properties and add them to the env
             3. Store indices of different bodies of the robot
        """
        print("---------------------------")
        print("Creating environments")

        asset_path = self.cfg.asset.file.format(LEGGED_GYM_ROOT_DIR=LEGGED_GYM_ROOT_DIR)
        asset_root = os.path.dirname(asset_path)
        asset_file = os.path.basename(asset_path)

        asset_options = gymapi.AssetOptions()
        asset_options.default_dof_drive_mode = self.cfg.asset.default_dof_drive_mode
        asset_options.collapse_fixed_joints = self.cfg.asset.collapse_fixed_joints
        asset_options.replace_cylinder_with_capsule = self.cfg.asset.replace_cylinder_with_capsule
        asset_options.flip_visual_attachments = self.cfg.asset.flip_visual_attachments
        asset_options.fix_base_link = self.cfg.asset.fix_base_link
        asset_options.density = self.cfg.asset.density
        asset_options.angular_damping = self.cfg.asset.angular_damping
        asset_options.linear_damping = self.cfg.asset.linear_damping
        asset_options.max_angular_velocity = self.cfg.asset.max_angular_velocity
        asset_options.max_linear_velocity = self.cfg.asset.max_linear_velocity
        asset_options.armature = self.cfg.asset.armature
        asset_options.thickness = self.cfg.asset.thickness
        asset_options.disable_gravity = self.cfg.asset.disable_gravity

        robot_asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)
        self.num_dof = self.gym.get_asset_dof_count(robot_asset)

        box_asset = self.gym.create_box(self.sim, 0.75, 0.75, 0.1)

        self.num_bodies = self.gym.get_asset_rigid_body_count(robot_asset)
        dof_props_asset = self.gym.get_asset_dof_properties(robot_asset)
        rigid_shape_props_asset = self.gym.get_asset_rigid_shape_properties(robot_asset)

        # save body names from the asset
        body_names = self.gym.get_asset_rigid_body_names(robot_asset)
        self.dof_names = self.gym.get_asset_dof_names(robot_asset)
        self.num_bodies = len(body_names)
        self.num_dofs = len(self.dof_names)
        self.motor_strength = torch.ones(self.num_envs, self.num_actions, dtype=torch.float, device=self.device,
                                         requires_grad=False)
        self.lin_vel_scales = torch.ones(self.num_envs, 3, dtype=torch.float, device=self.device, requires_grad=False)

        self.friction = torch.zeros(self.num_envs, dtype=torch.float, device=self.device, requires_grad=False)
        self.pevils_mass = torch.zeros(self.num_envs, dtype=torch.float, device=self.device, requires_grad=False)
        self.pevils_com = torch.zeros(self.num_envs, 3, dtype=torch.float, device=self.device, requires_grad=False)
        self.ex_forces = torch.zeros((self.num_envs, self.num_bodies, 3), device=self.device, dtype=torch.float)
        self.ex_torques = torch.zeros((self.num_envs, self.num_bodies, 3), device=self.device, dtype=torch.float)

        torso_name = [s for s in body_names if self.cfg.asset.torso_name in s]
        thigh_names = [s for s in body_names if self.cfg.asset.thigh_name in s]
        shin_names = [s for s in body_names if self.cfg.asset.shin_name in s]
        feet_names = [s for s in body_names if self.cfg.asset.foot_name in s]
        upper_arm_names = [s for s in body_names if self.cfg.asset.upper_arm_name in s]
        lower_arm_names = [s for s in body_names if self.cfg.asset.lower_arm_name in s]
        hand_names = [s for s in body_names if self.cfg.asset.hand_name in s]

        self.feet_handles = [self.gym.find_asset_rigid_body_index(robot_asset, name) for name in feet_names]
        sensor_pose = gymapi.Transform()
        sensor_pose.p = gymapi.Vec3(0.0, 0.0, -0.3)
        for foot_handle in self.feet_handles:
            sensor_options = gymapi.ForceSensorProperties()
            sensor_options.enable_forward_dynamics_forces = False  # for example gravity
            sensor_options.enable_constraint_solver_forces = True  # for example contacts
            sensor_options.use_world_frame = True  # report forces in world frame (easier to get vertical components)
            self.gym.create_asset_force_sensor(robot_asset, foot_handle, sensor_pose, sensor_options)
        # print("create force sensor")
        penalized_contact_names = []
        for name in self.cfg.asset.penalize_contacts_on:
            penalized_contact_names.extend([s for s in body_names if name in s])
        termination_contact_names = []
        for name in self.cfg.asset.terminate_after_contacts_on:
            termination_contact_names.extend([s for s in body_names if name in s])

        base_init_state_list = self.cfg.init_state.pos + self.cfg.init_state.rot + self.cfg.init_state.lin_vel + self.cfg.init_state.ang_vel
        self.base_init_state = to_torch(base_init_state_list, device=self.device, requires_grad=False)
        start_pose = gymapi.Transform()
        start_pose.p = gymapi.Vec3(*self.base_init_state[:3])

        self._get_env_origins()
        env_lower = gymapi.Vec3(0., 0., 0.)
        env_upper = gymapi.Vec3(0., 0., 0.)
        self.actor_handles = []
        self.box_handles = []
        self.envs = []

        for i in range(self.num_envs):
            # create env instance
            env_handle = self.gym.create_env(self.sim, env_lower, env_upper, int(np.sqrt(self.num_envs)))
            pos = self.env_origins[i].clone()
            pos[:2] += torch_rand_float(-1., 1., (2, 1), device=self.device).squeeze(1)
            start_pose.p = gymapi.Vec3(*pos)
            # self.ex_forces[i] = torch_rand_float(self.cfg.domain_rand.max_ex_forces[0],self.cfg.domain_rand.max_ex_forces[1],(self.num_bodies, 3), device=self.device)
            # self.ex_torques[i] = torch_rand_float(self.cfg.domain_rand.max_ex_torques[0],self.cfg.domain_rand.max_ex_torques[1],(self.num_bodies, 3), device=self.device)

            rigid_shape_props = self._process_rigid_shape_props(rigid_shape_props_asset, i)

            self.friction[i] = rigid_shape_props[4].friction
            self.gym.set_asset_rigid_shape_properties(robot_asset, rigid_shape_props)
            actor_handle = self.gym.create_actor(env_handle, robot_asset, start_pose, self.cfg.asset.name, i,
                                                 self.cfg.asset.self_collisions, 0)

            dof_props = self._process_dof_props(dof_props_asset, i)
            self.gym.set_actor_dof_properties(env_handle, actor_handle, dof_props)

            if self.cfg.domain_rand.randomize_motor_strength:
                rng_motor_strength = self.cfg.domain_rand.motor_strength
                self.motor_strength[i, :] = torch_rand_float(rng_motor_strength[0], rng_motor_strength[1],
                                                             (1, self.num_actions), device=self.device)
                # print("motor_stength: ")
                # print(self.motor_strength)

            if self.cfg.domain_rand.randomize_obs_linvel:
                rng_obs_linvel = self.cfg.domain_rand.obs_linvel
                self.lin_vel_scales[i, :] = torch_rand_float(rng_obs_linvel[0], rng_obs_linvel[1], (1, 3),
                                                             device=self.device)

            if i == 0:
                self.torso_indices = torch.zeros(len(torso_name), dtype=torch.long, device=self.device,
                                                 requires_grad=False)
                for j in range(len(torso_name)):
                    self.torso_indices[j] = self.gym.find_actor_rigid_body_handle(env_handle, actor_handle,
                                                                                  torso_name[j])
                self.thigh_indices = torch.zeros(len(thigh_names), dtype=torch.long, device=self.device,
                                                 requires_grad=False)
                for j in range(len(thigh_names)):
                    self.thigh_indices[j] = self.gym.find_actor_rigid_body_handle(env_handle, actor_handle,
                                                                                  thigh_names[j])
                self.shin_indices = torch.zeros(len(shin_names), dtype=torch.long, device=self.device,
                                                requires_grad=False)
                for j in range(len(shin_names)):
                    self.shin_indices[j] = self.gym.find_actor_rigid_body_handle(env_handle, actor_handle,
                                                                                 shin_names[j])
                self.feet_indices = torch.zeros(len(feet_names), dtype=torch.long, device=self.device,
                                                requires_grad=False)
                for j in range(len(feet_names)):
                    self.feet_indices[j] = self.gym.find_actor_rigid_body_handle(env_handle, actor_handle,
                                                                                 feet_names[j])
                self.upper_arm_indices = torch.zeros(len(upper_arm_names), dtype=torch.long, device=self.device,
                                                     requires_grad=False)
                for j in range(len(upper_arm_names)):
                    self.upper_arm_indices[j] = self.gym.find_actor_rigid_body_handle(env_handle, actor_handle,
                                                                                      upper_arm_names[j])
                self.lower_arm_indices = torch.zeros(len(lower_arm_names), dtype=torch.long, device=self.device,
                                                     requires_grad=False)
                for j in range(len(lower_arm_names)):
                    self.lower_arm_indices[j] = self.gym.find_actor_rigid_body_handle(env_handle, actor_handle,
                                                                                      lower_arm_names[j])
                self.hand_indices = torch.zeros(len(hand_names), dtype=torch.long, device=self.device,
                                                requires_grad=False)
                for j in range(len(hand_names)):
                    self.hand_indices[j] = self.gym.find_actor_rigid_body_handle(env_handle, actor_handle,
                                                                                 hand_names[j])

                print("torso_indices: " + str(self.torso_indices))
                print("thigh_indices: " + str(self.thigh_indices))
                print("shin_indices: " + str(self.shin_indices))
                print("feet_indices: " + str(self.feet_indices))
                print("upper_arm_indices: " + str(self.upper_arm_indices))
                print("lower_arm_indices: " + str(self.lower_arm_indices))
                print("hand_indices: " + str(self.hand_indices))

            self.ex_forces[i, self.torso_indices, :2] = torch_rand_float(self.cfg.domain_rand.max_ex_forces[0],
                                                                         self.cfg.domain_rand.max_ex_forces[1], (1, 2),
                                                                         device=self.device)
            self.ex_torques[i, self.torso_indices, :3] = torch_rand_float(self.cfg.domain_rand.max_ex_torques[0],
                                                                          self.cfg.domain_rand.max_ex_torques[1],
                                                                          (1, 3), device=self.device)
            body_props = self.gym.get_actor_rigid_body_properties(env_handle, actor_handle)
            body_props = self._process_rigid_body_props(body_props, i)
            self.pevils_com[i, 0] = body_props[0].com.x
            self.pevils_com[i, 1] = body_props[0].com.y
            self.pevils_com[i, 2] = body_props[0].com.z
            self.pevils_mass[i] = body_props[0].mass
            self.gym.set_actor_rigid_body_properties(env_handle, actor_handle, body_props, recomputeInertia=True)
            self.envs.append(env_handle)
            self.actor_handles.append(actor_handle)
            # box_handle = self.gym.create_actor(env_handle, box_asset, start_pose, "ball", i, 0, 0)
            # self.box_handles.append(box_handle)

        self.penalised_contact_indices = torch.zeros(len(penalized_contact_names), dtype=torch.long, device=self.device,
                                                     requires_grad=False)
        for i in range(len(penalized_contact_names)):
            self.penalised_contact_indices[i] = self.gym.find_actor_rigid_body_handle(self.envs[0],
                                                                                      self.actor_handles[0],
                                                                                      penalized_contact_names[i])

        self.termination_contact_indices = torch.zeros(len(termination_contact_names), dtype=torch.long,
                                                       device=self.device, requires_grad=False)
        for i in range(len(termination_contact_names)):
            self.termination_contact_indices[i] = self.gym.find_actor_rigid_body_handle(self.envs[0],
                                                                                        self.actor_handles[0],
                                                                                        termination_contact_names[i])

    def _get_env_origins(self):
        """ Sets environment origins. On rough terrain the origins are defined by the terrain platforms.
            Otherwise create a grid.
        """
        if self.cfg.terrain.mesh_type in ["heightfield", "trimesh"]:
            self.custom_origins = True
            self.env_origins = torch.zeros(self.num_envs, 3, device=self.device, requires_grad=False)
            # put robots at the origins defined by the terrain
            max_init_level = self.cfg.terrain.max_init_terrain_level
            if not self.cfg.terrain.curriculum: max_init_level = self.cfg.terrain.num_rows - 1
            self.terrain_levels = torch.randint(0, max_init_level + 1, (self.num_envs,), device=self.device)
            self.terrain_types = torch.div(torch.arange(self.num_envs, device=self.device),
                                           (self.num_envs / self.cfg.terrain.num_cols), rounding_mode='floor').to(
                torch.long)
            self.max_terrain_level = self.cfg.terrain.num_rows
            self.terrain_origins = torch.from_numpy(self.terrain.env_origins).to(self.device).to(torch.float)
            self.env_origins[:] = self.terrain_origins[self.terrain_levels, self.terrain_types]
        else:
            self.custom_origins = False
            self.env_origins = torch.zeros(self.num_envs, 3, device=self.device, requires_grad=False)
            # create a grid of robots
            num_cols = np.floor(np.sqrt(self.num_envs))
            num_rows = np.ceil(self.num_envs / num_cols)
            xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols))
            spacing = self.cfg.env.env_spacing
            self.env_origins[:, 0] = spacing * xx.flatten()[:self.num_envs]
            self.env_origins[:, 1] = spacing * yy.flatten()[:self.num_envs]
            self.env_origins[:, 2] = 0.

    def _parse_cfg(self):
        self.dt = self.cfg.control.decimation * self.sim_params.dt
        self.obs_scales = self.cfg.normalization.obs_scales
        self.reward_scales = class_to_dict(self.cfg.rewards.scales)
        self.command_ranges = class_to_dict(self.cfg.commands.ranges)

        if self.cfg.terrain.mesh_type not in ['heightfield', 'trimesh']:
            self.cfg.terrain.curriculum = False

        self.max_episode_length_s = self.cfg.env.episode_length_s
        self.max_episode_length = np.ceil(self.max_episode_length_s / self.dt)

        self.cfg.domain_rand.push_interval = np.ceil(self.cfg.domain_rand.push_interval_s / self.dt)

    def _draw_debug_vis(self):
        """ Draws visualizations for dubugging (slows down simulation a lot).
            Default behaviour: draws height measurement points
        """
        # draw height lines
        if not self.terrain.cfg.measure_heights:
            return
        self.gym.clear_lines(self.viewer)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        sphere_geom = gymutil.WireframeSphereGeometry(0.02, 4, 4, None, color=(1, 1, 0))
        for i in range(self.num_envs):
            base_pos = (self.root_states[i, :3]).cpu().numpy()
            heights = self.measured_heights[i].cpu().numpy()
            height_points = quat_apply_yaw(self.base_quat[i].repeat(heights.shape[0]),
                                           self.height_points[i]).cpu().numpy()
            for j in range(heights.shape[0]):
                x = height_points[j, 0] + base_pos[0]
                y = height_points[j, 1] + base_pos[1]
                z = heights[j]
                sphere_pose = gymapi.Transform(gymapi.Vec3(x, y, z), r=None)
                gymutil.draw_lines(sphere_geom, self.gym, self.viewer, self.envs[i], sphere_pose)

    def _init_height_points(self):
        """ Returns points at which the height measurments are sampled (in base frame)
        Returns:
            [torch.Tensor]: Tensor of shape (num_envs, self.num_height_points, 3)
        """
        y = torch.tensor(self.cfg.terrain.measured_points_y, device=self.device, requires_grad=False)
        x = torch.tensor(self.cfg.terrain.measured_points_x, device=self.device, requires_grad=False)
        grid_x, grid_y = torch.meshgrid(x, y)

        self.num_height_points = grid_x.numel()
        points = torch.zeros(self.num_envs, self.num_height_points, 3, device=self.device, requires_grad=False)
        points[:, :, 0] = grid_x.flatten()
        points[:, :, 1] = grid_y.flatten()
        return points

    # def _init_height_points_blind(self):
    #     """ Returns points at which the height measurments are sampled (in base frame)
    #     Returns:
    #         [torch.Tensor]: Tensor of shape (num_envs, self.num_height_points, 3)
    #     """
    #     y = torch.tensor(self.cfg.terrain.measured_points_y_blind, device=self.device, requires_grad=False)
    #     x = torch.tensor(self.cfg.terrain.measured_points_x_blind, device=self.device, requires_grad=False)
    #     grid_x, grid_y = torch.meshgrid(x, y)

    #     self.num_height_points = grid_x.numel()
    #     points = torch.zeros(self.num_envs, self.num_height_points, 3, device=self.device, requires_grad=False)
    #     points[:, :, 0] = grid_x.flatten()
    #     points[:, :, 1] = grid_y.flatten()
    #     return points
    def _get_heights(self, env_ids=None):
        """ Samples heights of the terrain at required points around each robot.
            The points are offset by the base's position and rotated by the base's yaw
        Args:
            env_ids (List[int], optional): Subset of environments for which to return the heights. Defaults to None.
        Raises:
            NameError: [description]
        Returns:
            [type]: [description]
        """
        if self.cfg.terrain.mesh_type == 'plane':
            return torch.zeros(self.num_envs, self.num_height_points, device=self.device, requires_grad=False)
        elif self.cfg.terrain.mesh_type == 'none':
            raise NameError("Can't measure height with terrain mesh type 'none'")

        if env_ids:
            points = quat_apply_yaw(self.base_quat[env_ids].repeat(1, self.num_height_points),
                                    self.height_points[env_ids]) + (self.root_states[env_ids, :3]).unsqueeze(1)
        else:
            points = quat_apply_yaw(self.base_quat.repeat(1, self.num_height_points), self.height_points) + (
                self.root_states[:, :3]).unsqueeze(1)

        points += self.terrain.cfg.border_size
        points = (points / self.terrain.cfg.horizontal_scale).long()
        px = points[:, :, 0].view(-1)
        py = points[:, :, 1].view(-1)
        px = torch.clip(px, 0, self.height_samples.shape[0] - 2)
        py = torch.clip(py, 0, self.height_samples.shape[1] - 2)

        heights1 = self.height_samples[px, py]
        heights2 = self.height_samples[px + 1, py]
        heights3 = self.height_samples[px, py + 1]
        heights = torch.min(heights1, heights2)
        heights = torch.min(heights, heights3)

        return heights.view(self.num_envs, -1) * self.terrain.cfg.vertical_scale

    def _get_heights_blind(self, env_ids=None):
        """ Samples heights of the terrain at required points around each robot.
            The points are offset by the base's position and rotated by the base's yaw
        Args:
            env_ids (List[int], optional): Subset of environments for which to return the heights. Defaults to None.
        Raises:
            NameError: [description]
        Returns:
            [type]: [description]
        """
        if self.cfg.terrain.mesh_type == 'plane':
            return torch.zeros(self.num_envs, self.num_height_points, device=self.device, requires_grad=False)
        elif self.cfg.terrain.mesh_type == 'none':
            raise NameError("Can't measure height with terrain mesh type 'none'")

        if env_ids:
            points = quat_apply_yaw(self.base_quat[env_ids].repeat(1, self.num_height_points),
                                    self.height_points_blind[env_ids]) + (self.root_states[env_ids, :3]).unsqueeze(1)
        else:
            points = quat_apply_yaw(self.base_quat.repeat(1, self.num_height_points), self.height_points_blind) + (
                self.root_states[:, :3]).unsqueeze(1)

        points += self.terrain.cfg.border_size
        points = (points / self.terrain.cfg.horizontal_scale).long()
        px = points[:, :, 0].view(-1)
        py = points[:, :, 1].view(-1)
        px = torch.clip(px, 0, self.height_samples.shape[0] - 2)
        py = torch.clip(py, 0, self.height_samples.shape[1] - 2)

        heights1 = self.height_samples[px, py]
        heights2 = self.height_samples[px + 1, py]
        heights3 = self.height_samples[px, py + 1]
        heights = torch.min(heights1, heights2)
        heights = torch.min(heights, heights3)

        return heights.view(self.num_envs, -1) * self.terrain.cfg.vertical_scale

    def _reset_fric_para(self, env_ids):
        self.fric_para_0[env_ids, 0] = torch_rand_float(3.7, 6.6, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_0[env_ids, 1] = torch_rand_float(3.3, 5.0, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_0[env_ids, 2] = torch_rand_float(-5.0, -3.3, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_0[env_ids, 3] = torch_rand_float(0.7, 0.9, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_0[env_ids, 4] = torch_rand_float(0.7, 0.9, (len(env_ids), 1), device=self.device).squeeze(1)

        self.fric_para_1[env_ids, 0] = torch_rand_float(1.2, 2.75, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_1[env_ids, 1] = torch_rand_float(1.0, 1.55, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_1[env_ids, 2] = torch_rand_float(-1.55, -1.0, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_1[env_ids, 3] = torch_rand_float(0.4, 0.65, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_1[env_ids, 4] = torch_rand_float(0.4, 0.65, (len(env_ids), 1), device=self.device).squeeze(1)

        self.fric_para_2[env_ids, 0] = torch_rand_float(1.9, 3.3, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_2[env_ids, 1] = torch_rand_float(1.15, 2.0, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_2[env_ids, 2] = torch_rand_float(-2.0, -1.3, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_2[env_ids, 3] = torch_rand_float(0.14, 0.18, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_2[env_ids, 4] = torch_rand_float(0.14, 0.18, (len(env_ids), 1), device=self.device).squeeze(1)

        self.fric_para_3[env_ids, 0] = torch_rand_float(0.25, 1.25, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_3[env_ids, 1] = torch_rand_float(0.2, 1.0, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_3[env_ids, 2] = torch_rand_float(-1.0, -0.2, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_3[env_ids, 3] = torch_rand_float(0.14, 0.18, (len(env_ids), 1), device=self.device).squeeze(1)
        self.fric_para_3[env_ids, 4] = torch_rand_float(0.14, 0.18, (len(env_ids), 1), device=self.device).squeeze(1)

    def friction_0_6(self):
        # Fst = torch_rand_float(3.7,6.6,(self.num_envs,1),device=self.device)
        # Fc0 = torch_rand_float(3.3,5.0,(self.num_envs,1),device=self.device)
        # Fc1 = torch_rand_float(-5.0,-3.3,(self.num_envs,1),device=self.device)
        # kc0 = torch_rand_float(0.7,0.9,(self.num_envs,1),device=self.device)
        # kc1 = torch_rand_float(0.7,0.9,(self.num_envs,1),device=self.device)
        flag_0 = (self.dof_vel[:, 0] <= 0.002) & (self.dof_vel[:, 0] >= -0.002)
        flag_1 = ((self.dof_vel[:, 0] > 0.002) & (self.dof_vel[:, 0] <= 0.16))
        flag_2 = (self.dof_vel[:, 0] > 0.16)
        flag_3 = ((self.dof_vel[:, 0] < -0.002) & (self.dof_vel[:, 0] >= -0.16))
        flag_4 = (self.dof_vel[:, 0] < -0.16)

        self.fric[:, 0] = self.fric_para_0[:, 0] / 0.002 * self.dof_vel[:, 0] * flag_0 + \
                          ((self.fric_para_0[:, 1] - self.fric_para_0[:, 0]) / (0.16 - 0.002) * (
                                  self.dof_vel[:, 0] - 0.002) + self.fric_para_0[:, 0]) * flag_1 + \
                          (self.fric_para_0[:, 1] + self.fric_para_0[:, 3] * (self.dof_vel[:, 0] - 0.16)) * flag_2 + \
                          ((self.fric_para_0[:, 2] + self.fric_para_0[:, 0]) / (-0.16 + 0.002) * (
                                  self.dof_vel[:, 0] + 0.002) - self.fric_para_0[:, 0]) * flag_3 + \
                          (self.fric_para_0[:, 2] + self.fric_para_0[:, 4] * (self.dof_vel[:, 0] + 0.16)) * flag_4

        flag_0 = (self.dof_vel[:, 6] <= 0.002) & (self.dof_vel[:, 6] >= -0.002)
        flag_1 = ((self.dof_vel[:, 6] > 0.002) & (self.dof_vel[:, 6] <= 0.16))
        flag_2 = (self.dof_vel[:, 6] > 0.16)
        flag_3 = ((self.dof_vel[:, 6] < -0.002) & (self.dof_vel[:, 6] >= -0.16))
        flag_4 = (self.dof_vel[:, 6] < -0.16)

        self.fric[:, 6] = self.fric_para_0[:, 0] / 0.002 * self.dof_vel[:, 6] * flag_0 + \
                          ((self.fric_para_0[:, 1] - self.fric_para_0[:, 0]) / (0.16 - 0.002) * (
                                  self.dof_vel[:, 6] - 0.002) + self.fric_para_0[:, 0]) * flag_1 + \
                          (self.fric_para_0[:, 1] + self.fric_para_0[:, 3] * (self.dof_vel[:, 6] - 0.16)) * flag_2 + \
                          ((self.fric_para_0[:, 2] + self.fric_para_0[:, 0]) / (-0.16 + 0.002) * (
                                  self.dof_vel[:, 6] + 0.002) - self.fric_para_0[:, 0]) * flag_3 + \
                          (self.fric_para_0[:, 2] + self.fric_para_0[:, 4] * (self.dof_vel[:, 6] + 0.16)) * flag_4
        # print("self.dof_vel[:,[0,6]]")
        # print(self.dof_vel[:,[0,6]])
        # print(self.fric[:,[0,6]])

    def friction_1_7(self):
        # Fst = torch_rand_float(1.2,2.75,(self.num_envs,1),device=self.device)
        # Fc0 = torch_rand_float(1.0,1.55,(self.num_envs,1),device=self.device)
        # Fc1 = torch_rand_float(-1.55,-1.0,(self.num_envs,1),device=self.device)
        # kc0 = torch_rand_float(0.4,0.65,(self.num_envs,1),device=self.device)
        # kc1 = torch_rand_float(0.4,0.65,(self.num_envs,1),device=self.device)
        flag_0 = (self.dof_vel[:, 1] <= 0.002) & (self.dof_vel[:, 1] >= -0.002)
        flag_1 = ((self.dof_vel[:, 1] > 0.002) & (self.dof_vel[:, 1] <= 0.16))
        flag_2 = (self.dof_vel[:, 1] > 0.16)
        flag_3 = ((self.dof_vel[:, 1] < -0.002) & (self.dof_vel[:, 1] >= -0.16))
        flag_4 = (self.dof_vel[:, 1] < -0.16)

        self.fric[:, 1] = self.fric_para_1[:, 0] / 0.002 * self.dof_vel[:, 1] * flag_0 + \
                          ((self.fric_para_1[:, 1] - self.fric_para_1[:, 0]) / (0.16 - 0.002) * (
                                  self.dof_vel[:, 1] - 0.002) + self.fric_para_1[:, 0]) * flag_1 + \
                          (self.fric_para_1[:, 1] + self.fric_para_1[:, 3] * (self.dof_vel[:, 1] - 0.16)) * flag_2 + \
                          ((self.fric_para_1[:, 2] + self.fric_para_1[:, 0]) / (-0.16 + 0.002) * (
                                  self.dof_vel[:, 1] + 0.002) - self.fric_para_1[:, 0]) * flag_3 + \
                          (self.fric_para_1[:, 2] + self.fric_para_1[:, 4] * (self.dof_vel[:, 1] + 0.16)) * flag_4

        flag_0 = (self.dof_vel[:, 7] <= 0.002) & (self.dof_vel[:, 7] >= -0.002)
        flag_1 = ((self.dof_vel[:, 7] > 0.002) & (self.dof_vel[:, 7] <= 0.16))
        flag_2 = (self.dof_vel[:, 7] > 0.16)
        flag_3 = ((self.dof_vel[:, 7] < -0.002) & (self.dof_vel[:, 7] >= -0.16))
        flag_4 = (self.dof_vel[:, 7] < -0.16)

        self.fric[:, 7] = self.fric_para_1[:, 0] / 0.002 * self.dof_vel[:, 7] * flag_0 + \
                          ((self.fric_para_1[:, 1] - self.fric_para_1[:, 0]) / (0.16 - 0.002) * (
                                  self.dof_vel[:, 7] - 0.002) + self.fric_para_1[:, 0]) * flag_1 + \
                          (self.fric_para_1[:, 1] + self.fric_para_1[:, 3] * (self.dof_vel[:, 7] - 0.16)) * flag_2 + \
                          ((self.fric_para_1[:, 2] + self.fric_para_1[:, 0]) / (-0.16 + 0.002) * (
                                  self.dof_vel[:, 7] + 0.002) - self.fric_para_1[:, 0]) * flag_3 + \
                          (self.fric_para_1[:, 2] + self.fric_para_1[:, 4] * (self.dof_vel[:, 7] + 0.16)) * flag_4
        # print("self.dof_vel[:,[1,7]]")
        # print(self.dof_vel[:,[1,7]])
        # print(self.fric[:,[1,7]])

    def friction_2_3_8_9(self):
        # Fst = torch_rand_float(1.9,3.3,(self.num_envs,1),device=self.device)
        # Fc0 = torch_rand_float(1.15,2.0,(self.num_envs,1),device=self.device)
        # Fc1 = torch_rand_float(-2.0,-1.3,(self.num_envs,1),device=self.device)
        # kc0 = torch_rand_float(0.14,0.18,(self.num_envs,1),device=self.device)
        # kc1 = torch_rand_float(0.14,0.18,(self.num_envs,1),device=self.device)
        flag_0 = (self.dof_vel[:, 2] <= 0.002) & (self.dof_vel[:, 2] >= -0.002)
        flag_1 = ((self.dof_vel[:, 2] > 0.002) & (self.dof_vel[:, 2] <= 0.16))
        flag_2 = (self.dof_vel[:, 2] > 0.16)
        flag_3 = ((self.dof_vel[:, 2] < -0.002) & (self.dof_vel[:, 2] >= -0.16))
        flag_4 = (self.dof_vel[:, 2] < -0.16)

        self.fric[:, 2] = self.fric_para_2[:, 0] / 0.002 * self.dof_vel[:, 2] * flag_0 + \
                          ((self.fric_para_2[:, 1] - self.fric_para_2[:, 0]) / (0.16 - 0.002) * (
                                  self.dof_vel[:, 2] - 0.002) + self.fric_para_2[:, 0]) * flag_1 + \
                          (self.fric_para_2[:, 1] + self.fric_para_2[:, 3] * (self.dof_vel[:, 2] - 0.16)) * flag_2 + \
                          ((self.fric_para_2[:, 2] + self.fric_para_2[:, 0]) / (-0.16 + 0.002) * (
                                  self.dof_vel[:, 2] + 0.002) - self.fric_para_2[:, 0]) * flag_3 + \
                          (self.fric_para_2[:, 2] + self.fric_para_2[:, 4] * (self.dof_vel[:, 2] + 0.16)) * flag_4

        flag_0 = (self.dof_vel[:, 3] <= 0.002) & (self.dof_vel[:, 3] >= -0.002)
        flag_1 = ((self.dof_vel[:, 3] > 0.002) & (self.dof_vel[:, 3] <= 0.16))
        flag_2 = (self.dof_vel[:, 3] > 0.16)
        flag_3 = ((self.dof_vel[:, 3] < -0.002) & (self.dof_vel[:, 3] >= -0.16))
        flag_4 = (self.dof_vel[:, 3] < -0.16)

        self.fric[:, 3] = self.fric_para_2[:, 0] / 0.002 * self.dof_vel[:, 3] * flag_0 + \
                          ((self.fric_para_2[:, 1] - self.fric_para_2[:, 0]) / (0.16 - 0.002) * (
                                  self.dof_vel[:, 3] - 0.002) + self.fric_para_2[:, 0]) * flag_1 + \
                          (self.fric_para_2[:, 1] + self.fric_para_2[:, 3] * (self.dof_vel[:, 3] - 0.16)) * flag_2 + \
                          ((self.fric_para_2[:, 2] + self.fric_para_2[:, 0]) / (-0.16 + 0.002) * (
                                  self.dof_vel[:, 3] + 0.002) - self.fric_para_2[:, 0]) * flag_3 + \
                          (self.fric_para_2[:, 2] + self.fric_para_2[:, 4] * (self.dof_vel[:, 3] + 0.16)) * flag_4

        flag_0 = (self.dof_vel[:, 8] <= 0.002) & (self.dof_vel[:, 8] >= -0.002)
        flag_1 = ((self.dof_vel[:, 8] > 0.002) & (self.dof_vel[:, 8] <= 0.16))
        flag_2 = (self.dof_vel[:, 8] > 0.16)
        flag_3 = ((self.dof_vel[:, 8] < -0.002) & (self.dof_vel[:, 8] >= -0.16))
        flag_4 = (self.dof_vel[:, 8] < -0.16)

        self.fric[:, 8] = self.fric_para_2[:, 0] / 0.002 * self.dof_vel[:, 8] * flag_0 + \
                          ((self.fric_para_2[:, 1] - self.fric_para_2[:, 0]) / (0.16 - 0.002) * (
                                  self.dof_vel[:, 8] - 0.002) + self.fric_para_2[:, 0]) * flag_1 + \
                          (self.fric_para_2[:, 1] + self.fric_para_2[:, 3] * (self.dof_vel[:, 8] - 0.16)) * flag_2 + \
                          ((self.fric_para_2[:, 2] + self.fric_para_2[:, 0]) / (-0.16 + 0.002) * (
                                  self.dof_vel[:, 8] + 0.002) - self.fric_para_2[:, 0]) * flag_3 + \
                          (self.fric_para_2[:, 2] + self.fric_para_2[:, 4] * (self.dof_vel[:, 8] + 0.16)) * flag_4

        flag_0 = (self.dof_vel[:, 9] <= 0.002) & (self.dof_vel[:, 9] >= -0.002)
        flag_1 = ((self.dof_vel[:, 9] > 0.002) & (self.dof_vel[:, 9] <= 0.16))
        flag_2 = (self.dof_vel[:, 9] > 0.16)
        flag_3 = ((self.dof_vel[:, 9] < -0.002) & (self.dof_vel[:, 9] >= -0.16))
        flag_4 = (self.dof_vel[:, 9] < -0.16)

        self.fric[:, 9] = self.fric_para_2[:, 0] / 0.002 * self.dof_vel[:, 9] * flag_0 + \
                          ((self.fric_para_2[:, 1] - self.fric_para_2[:, 0]) / (0.16 - 0.002) * (
                                  self.dof_vel[:, 9] - 0.002) + self.fric_para_2[:, 0]) * flag_1 + \
                          (self.fric_para_2[:, 1] + self.fric_para_2[:, 3] * (self.dof_vel[:, 9] - 0.16)) * flag_2 + \
                          ((self.fric_para_2[:, 2] + self.fric_para_2[:, 0]) / (-0.16 + 0.002) * (
                                  self.dof_vel[:, 9] + 0.002) - self.fric_para_2[:, 0]) * flag_3 + \
                          (self.fric_para_2[:, 2] + self.fric_para_2[:, 4] * (self.dof_vel[:, 9] + 0.16)) * flag_4
        # print("self.dof_vel[:,[2,3,8,9]]")
        # print(self.dof_vel[:,[2,3,8,9]])
        # print(self.fric[:,[2,3,8,9]])

    def friction_arm(self):
        Fst = torch_rand_float(0.25, 1.25, (self.num_envs, 1), device=self.device)
        Fc0 = torch_rand_float(0.2, 1.0, (self.num_envs, 1), device=self.device)
        Fc1 = torch_rand_float(-1.0, -0.2, (self.num_envs, 1), device=self.device)
        kc0 = torch_rand_float(0.14, 0.18, (self.num_envs, 1), device=self.device)
        kc1 = torch_rand_float(0.14, 0.18, (self.num_envs, 1), device=self.device)
        # 15
        flag_0 = (self.dof_vel[:, 15] <= 0.002) & (self.dof_vel[:, 15] >= -0.002)
        flag_1 = ((self.dof_vel[:, 15] > 0.002) & (self.dof_vel[:, 15] <= 0.16))
        flag_2 = (self.dof_vel[:, 15] > 0.16)
        flag_3 = ((self.dof_vel[:, 15] < -0.002) & (self.dof_vel[:, 15] >= -0.16))
        flag_4 = (self.dof_vel[:, 15] < -0.16)
        self.fric[:, 15] = self.fric_para_3[:, 0] / 0.002 * self.dof_vel[:, 15] * flag_0 + \
                           ((self.fric_para_3[:, 1] - self.fric_para_3[:, 0]) / (0.16 - 0.002) * (
                                   self.dof_vel[:, 15] - 0.002) + self.fric_para_3[:, 0]) * flag_1 + \
                           (self.fric_para_3[:, 1] + self.fric_para_3[:, 3] * (self.dof_vel[:, 15] - 0.16)) * flag_2 + \
                           ((self.fric_para_3[:, 2] + self.fric_para_3[:, 0]) / (-0.16 + 0.002) * (
                                   self.dof_vel[:, 15] + 0.002) - self.fric_para_3[:, 0]) * flag_3 + \
                           (self.fric_para_3[:, 2] + self.fric_para_3[:, 4] * (self.dof_vel[:, 15] + 0.16)) * flag_4
        # 16
        flag_0 = (self.dof_vel[:, 16] <= 0.002) & (self.dof_vel[:, 16] >= -0.002)
        flag_1 = ((self.dof_vel[:, 16] > 0.002) & (self.dof_vel[:, 16] <= 0.16))
        flag_2 = (self.dof_vel[:, 16] > 0.16)
        flag_3 = ((self.dof_vel[:, 16] < -0.002) & (self.dof_vel[:, 16] >= -0.16))
        flag_4 = (self.dof_vel[:, 16] < -0.16)
        self.fric[:, 16] = self.fric_para_3[:, 0] / 0.002 * self.dof_vel[:, 16] * flag_0 + \
                           ((self.fric_para_3[:, 1] - self.fric_para_3[:, 0]) / (0.16 - 0.002) * (
                                   self.dof_vel[:, 16] - 0.002) + self.fric_para_3[:, 0]) * flag_1 + \
                           (self.fric_para_3[:, 1] + self.fric_para_3[:, 3] * (self.dof_vel[:, 16] - 0.16)) * flag_2 + \
                           ((self.fric_para_3[:, 2] + self.fric_para_3[:, 0]) / (-0.16 + 0.002) * (
                                   self.dof_vel[:, 16] + 0.002) - self.fric_para_3[:, 0]) * flag_3 + \
                           (self.fric_para_3[:, 2] + self.fric_para_3[:, 4] * (self.dof_vel[:, 16] + 0.16)) * flag_4
        # 17
        flag_0 = (self.dof_vel[:, 17] <= 0.002) & (self.dof_vel[:, 17] >= -0.002)
        flag_1 = ((self.dof_vel[:, 17] > 0.002) & (self.dof_vel[:, 17] <= 0.16))
        flag_2 = (self.dof_vel[:, 17] > 0.16)
        flag_3 = ((self.dof_vel[:, 17] < -0.002) & (self.dof_vel[:, 17] >= -0.16))
        flag_4 = (self.dof_vel[:, 17] < -0.16)
        self.fric[:, 17] = self.fric_para_3[:, 0] / 0.002 * self.dof_vel[:, 17] * flag_0 + \
                           ((self.fric_para_3[:, 1] - self.fric_para_3[:, 0]) / (0.16 - 0.002) * (
                                   self.dof_vel[:, 17] - 0.002) + self.fric_para_3[:, 0]) * flag_1 + \
                           (self.fric_para_3[:, 1] + self.fric_para_3[:, 3] * (self.dof_vel[:, 17] - 0.16)) * flag_2 + \
                           ((self.fric_para_3[:, 2] + self.fric_para_3[:, 0]) / (-0.16 + 0.002) * (
                                   self.dof_vel[:, 17] + 0.002) - self.fric_para_3[:, 0]) * flag_3 + \
                           (self.fric_para_3[:, 2] + self.fric_para_3[:, 4] * (self.dof_vel[:, 17] + 0.16)) * flag_4
        # 18
        flag_0 = (self.dof_vel[:, 18] <= 0.002) & (self.dof_vel[:, 18] >= -0.002)
        flag_1 = ((self.dof_vel[:, 18] > 0.002) & (self.dof_vel[:, 18] <= 0.16))
        flag_2 = (self.dof_vel[:, 18] > 0.16)
        flag_3 = ((self.dof_vel[:, 18] < -0.002) & (self.dof_vel[:, 18] >= -0.16))
        flag_4 = (self.dof_vel[:, 18] < -0.16)
        self.fric[:, 18] = self.fric_para_3[:, 0] / 0.002 * self.dof_vel[:, 18] * flag_0 + \
                           ((self.fric_para_3[:, 1] - self.fric_para_3[:, 0]) / (0.16 - 0.002) * (
                                   self.dof_vel[:, 18] - 0.002) + self.fric_para_3[:, 0]) * flag_1 + \
                           (self.fric_para_3[:, 1] + self.fric_para_3[:, 3] * (self.dof_vel[:, 18] - 0.16)) * flag_2 + \
                           ((self.fric_para_3[:, 2] + self.fric_para_3[:, 0]) / (-0.16 + 0.002) * (
                                   self.dof_vel[:, 18] + 0.002) - self.fric_para_3[:, 0]) * flag_3 + \
                           (self.fric_para_3[:, 2] + self.fric_para_3[:, 4] * (self.dof_vel[:, 18] + 0.16)) * flag_4
        # 19
        flag_0 = (self.dof_vel[:, 19] <= 0.002) & (self.dof_vel[:, 19] >= -0.002)
        flag_1 = ((self.dof_vel[:, 19] > 0.002) & (self.dof_vel[:, 19] <= 0.16))
        flag_2 = (self.dof_vel[:, 19] > 0.16)
        flag_3 = ((self.dof_vel[:, 19] < -0.002) & (self.dof_vel[:, 19] >= -0.16))
        flag_4 = (self.dof_vel[:, 19] < -0.16)
        self.fric[:, 19] = self.fric_para_3[:, 0] / 0.002 * self.dof_vel[:, 19] * flag_0 + \
                           ((self.fric_para_3[:, 1] - self.fric_para_3[:, 0]) / (0.16 - 0.002) * (
                                   self.dof_vel[:, 19] - 0.002) + self.fric_para_3[:, 0]) * flag_1 + \
                           (self.fric_para_3[:, 1] + self.fric_para_3[:, 3] * (self.dof_vel[:, 19] - 0.16)) * flag_2 + \
                           ((self.fric_para_3[:, 2] + self.fric_para_3[:, 0]) / (-0.16 + 0.002) * (
                                   self.dof_vel[:, 19] + 0.002) - self.fric_para_3[:, 0]) * flag_3 + \
                           (self.fric_para_3[:, 2] + self.fric_para_3[:, 4] * (self.dof_vel[:, 19] + 0.16)) * flag_4
        # 20
        flag_0 = (self.dof_vel[:, 20] <= 0.002) & (self.dof_vel[:, 20] >= -0.002)
        flag_1 = ((self.dof_vel[:, 20] > 0.002) & (self.dof_vel[:, 20] <= 0.16))
        flag_2 = (self.dof_vel[:, 20] > 0.16)
        flag_3 = ((self.dof_vel[:, 20] < -0.002) & (self.dof_vel[:, 20] >= -0.16))
        flag_4 = (self.dof_vel[:, 20] < -0.16)
        self.fric[:, 20] = self.fric_para_3[:, 0] / 0.002 * self.dof_vel[:, 20] * flag_0 + \
                           ((self.fric_para_3[:, 1] - self.fric_para_3[:, 0]) / (0.16 - 0.002) * (
                                   self.dof_vel[:, 20] - 0.002) + self.fric_para_3[:, 0]) * flag_1 + \
                           (self.fric_para_3[:, 1] + self.fric_para_3[:, 3] * (self.dof_vel[:, 20] - 0.16)) * flag_2 + \
                           ((self.fric_para_3[:, 2] + self.fric_para_3[:, 0]) / (-0.16 + 0.002) * (
                                   self.dof_vel[:, 20] + 0.002) - self.fric_para_3[:, 0]) * flag_3 + \
                           (self.fric_para_3[:, 2] + self.fric_para_3[:, 4] * (self.dof_vel[:, 20] + 0.16)) * flag_4
        # 21
        flag_0 = (self.dof_vel[:, 21] <= 0.002) & (self.dof_vel[:, 21] >= -0.002)
        flag_1 = ((self.dof_vel[:, 21] > 0.002) & (self.dof_vel[:, 21] <= 0.16))
        flag_2 = (self.dof_vel[:, 21] > 0.16)
        flag_3 = ((self.dof_vel[:, 21] < -0.002) & (self.dof_vel[:, 21] >= -0.16))
        flag_4 = (self.dof_vel[:, 21] < -0.16)
        self.fric[:, 21] = self.fric_para_3[:, 0] / 0.002 * self.dof_vel[:, 21] * flag_0 + \
                           ((self.fric_para_3[:, 1] - self.fric_para_3[:, 0]) / (0.16 - 0.002) * (
                                   self.dof_vel[:, 21] - 0.002) + self.fric_para_3[:, 0]) * flag_1 + \
                           (self.fric_para_3[:, 1] + self.fric_para_3[:, 3] * (self.dof_vel[:, 21] - 0.16)) * flag_2 + \
                           ((self.fric_para_3[:, 2] + self.fric_para_3[:, 0]) / (-0.16 + 0.002) * (
                                   self.dof_vel[:, 21] + 0.002) - self.fric_para_3[:, 0]) * flag_3 + \
                           (self.fric_para_3[:, 2] + self.fric_para_3[:, 4] * (self.dof_vel[:, 21] + 0.16)) * flag_4
        # 22
        flag_0 = (self.dof_vel[:, 22] <= 0.002) & (self.dof_vel[:, 22] >= -0.002)
        flag_1 = ((self.dof_vel[:, 22] > 0.002) & (self.dof_vel[:, 22] <= 0.16))
        flag_2 = (self.dof_vel[:, 22] > 0.16)
        flag_3 = ((self.dof_vel[:, 22] < -0.002) & (self.dof_vel[:, 22] >= -0.16))
        flag_4 = (self.dof_vel[:, 22] < -0.16)
        self.fric[:, 22] = self.fric_para_3[:, 0] / 0.002 * self.dof_vel[:, 22] * flag_0 + \
                           ((self.fric_para_3[:, 1] - self.fric_para_3[:, 0]) / (0.16 - 0.002) * (
                                   self.dof_vel[:, 22] - 0.002) + self.fric_para_3[:, 0]) * flag_1 + \
                           (self.fric_para_3[:, 1] + self.fric_para_3[:, 3] * (self.dof_vel[:, 22] - 0.16)) * flag_2 + \
                           ((self.fric_para_3[:, 2] + self.fric_para_3[:, 0]) / (-0.16 + 0.002) * (
                                   self.dof_vel[:, 22] + 0.002) - self.fric_para_3[:, 0]) * flag_3 + \
                           (self.fric_para_3[:, 2] + self.fric_para_3[:, 4] * (self.dof_vel[:, 22] + 0.16)) * flag_4

    def friction_5_11(self):
        Fst = torch_rand_float(0.1875, 0.9375, (self.num_envs, 1), device=self.device)
        Fc0 = torch_rand_float(0.15, 0.75, (self.num_envs, 1), device=self.device)
        Fc1 = torch_rand_float(-0.75, -0.15, (self.num_envs, 1), device=self.device)
        kc0 = torch_rand_float(0.04, 0.0513, (self.num_envs, 1), device=self.device)
        kc1 = torch_rand_float(0.04, 0.0513, (self.num_envs, 1), device=self.device)
        flag_0 = (self.dof_vel[:, [5, 11]] <= 0.002) & (self.dof_vel[:, [5, 11]] >= -0.002)
        flag_1 = ((self.dof_vel[:, [5, 11]] > 0.002) & (self.dof_vel[:, [5, 11]] <= 0.16))
        flag_2 = (self.dof_vel[:, [5, 11]] > 0.16)
        flag_3 = ((self.dof_vel[:, [5, 11]] < -0.002) & (self.dof_vel[:, [5, 11]] >= -0.16))
        flag_4 = (self.dof_vel[:, [5, 11]] < -0.16)

        self.fric[:, [5, 11]] = Fst / 0.002 * self.dof_vel[:, [5, 11]] * flag_0 + \
                                ((Fc0 - Fst) / (0.16 - 0.002) * (self.dof_vel[:, [5, 11]] - 0.002) + Fst) * flag_1 + \
                                (Fc0 + kc0 * (self.dof_vel[:, [5, 11]] - 0.16)) * flag_2 + \
                                ((Fc1 + Fst) / (-0.16 + 0.002) * (self.dof_vel[:, [5, 11]] + 0.002) - Fst) * flag_3 + \
                                (Fc1 + kc1 * (self.dof_vel[:, [5, 11]] + 0.16)) * flag_4

    # ------------ reward functions----------------
    def _reward_lin_vel_z(self):
        # Penalize z axis base linear velocity
        # print(self.base_lin_vel[:, 2])
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self):
        # Penalize xy axes base angular velocity
        # print(self.base_ang_vel[:, :2])
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self):
        # Penalize non flat base orientation
        # print(torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1))
        return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)

    def _reward_base_height(self):
        # Penalize base height away from target
        base_height = torch.mean(self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1)
        # print(base_height)
        return torch.square(base_height - self.cfg.rewards.base_height_target)

    def _reward_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        return torch.sum(torch.square(self.torques), dim=1)

    def _reward_dof_vel(self):
        # Penalize dof velocities
        # print(torch.sum(torch.square(self.dof_vel), dim=1))
        # print(torch.sum(torch.square(self.dof_vel[:,[0,6]]), dim=1))
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self):
        # Penalize dof accelerations
        # print(torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1))
        return torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1)

    def _reward_action_rate(self):
        # Penalize changes in actions
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_collision(self):
        # Penalize collisions on selected bodies
        return torch.sum(1. * (torch.norm(self.contact_forces[:, self.penalised_contact_indices, :], dim=-1) > 0.1),
                         dim=1)

    def _reward_termination(self):
        # Terminal reward / penalty
        return self.reset_buf * ~self.time_out_buf

    def _reward_dof_pos_limits(self):
        # Penalize dof positions too close to the limit
        out_of_limits = -(self.dof_pos - self.dof_pos_limits[:, 0]).clip(max=0.)  # lower limit
        out_of_limits += (self.dof_pos - self.dof_pos_limits[:, 1]).clip(min=0.)
        # print(out_of_limits)
        return torch.sum(out_of_limits, dim=1)

    def _reward_dof_vel_limits(self):
        # Penalize dof velocities too close to the limit
        # clip to max error = 1 rad/s per joint to avoid huge penalties
        return torch.sum(
            (torch.abs(self.dof_vel) - self.dof_vel_limits * self.cfg.rewards.soft_dof_vel_limit).clip(min=0., max=1.),
            dim=1)

    def _reward_torque_limits(self):
        # penalize torques too close to the limit
        return torch.sum(
            (torch.abs(self.torques) - self.torque_limits * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1)

    def _reward_tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.cfg.rewards.tracking_sigma)

    def _reward_tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw)
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.cfg.rewards.tracking_sigma)

    def _reward_feet_air_time(self):
        # Reward long steps
        # Need to filter the contacts because the contact reporting of PhysX is unreliable on meshes
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        # contact = self.force_sensor_readings[0, :, 2] > 1.
        # print(self.contact_forces[0, self.feet_indices, 2])
        contact_filt = torch.logical_or(contact, self.last_contacts)
        self.last_contacts = contact
        first_contact = (self.feet_air_time > 0.) * contact_filt
        self.feet_air_time += self.dt
        self.last_feet_air_time = self.last_feet_air_time + (self.feet_air_time - self.last_feet_air_time) * (
                (self.feet_air_time * first_contact) > 0.01)
        rew_airTime = torch.sum((self.feet_air_time - 0.5) * first_contact, dim=1) - torch.sum(
            torch.abs(self.feet_air_time - 0.5) * first_contact, dim=1)  # reward only on first contact with the ground
        # rew_airTime = torch.sum((self.feet_air_time - 0.5) * first_contact, dim=1) # reward only on first contact with the ground
        # rew_airTime = torch.sum((torch.clamp(self.feet_air_time - 0.5,-0.5,0.0)) * first_contact, dim=1) # reward only on first contact with the ground
        # print(self.last_feet_air_time)
        rew_airTime *= torch.norm(self.commands[:, :2], dim=1) > 0.1  # no reward for zero command
        # print(rew_airTime)
        self.feet_air_time *= ~contact_filt
        # print(rew_airTime)
        return rew_airTime

    def _reward_stumble(self):
        # Penalize feet hitting vertical surfaces
        # print(torch.any(torch.norm(self.contact_forces[:, self.feet_indices, :2], dim=2) >\
        #      5 *torch.abs(self.contact_forces[:, self.feet_indices, 2]), dim=1))
        return torch.any(torch.norm(self.contact_forces[:, self.feet_indices, :2], dim=2) > \
                         5 * torch.abs(self.contact_forces[:, self.feet_indices, 2]), dim=1)

    def _reward_stand_still(self):
        # Penalize motion at zero commands
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1) * (
                torch.norm(self.commands[:, :2], dim=1) < 0.1)

    def _reward_feet_contact_forces(self):
        # penalize high contact forces
        # print(torch.norm(self.contact_forces[:, self.feet_indices, :], dim=-1))
        return torch.sum((torch.norm(self.contact_forces[:, self.feet_indices, :],
                                     dim=-1) - self.cfg.rewards.max_contact_force).clip(min=0.), dim=1)
