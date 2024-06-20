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

from time import time
import numpy as np
import os

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
from typing import Tuple, Dict
from legged_gym.envs import LeggedRobot
from legged_gym.utils.helpers import class_to_dict

from .clockgait import trapezoidclockgait as gait_clock
from .gr1l_config_option import GR1LCfg


class GR1L(LeggedRobot):

    def init_cfg(self, cfg):
        self.cfg: GR1LCfg = cfg

    def _parse_cfg(self):
        super()._parse_cfg()

        # stand --------------------------------------------

        self.reward_scales_stand = class_to_dict(self.cfg.rewards.scales_stand)
        self.command_ranges_stand = class_to_dict(self.cfg.commands.ranges_stand)

        print("self.reward_scales_stand: \n", self.reward_scales_stand)
        print("self.command_ranges_stand: \n", self.command_ranges_stand)

        # stand --------------------------------------------

        # walk --------------------------------------------

        self.reward_scales_walk = class_to_dict(self.cfg.rewards.scales_walk)
        self.command_ranges_walk = class_to_dict(self.cfg.commands.ranges_walk)
        self.command_ranges_walk_forward = class_to_dict(self.cfg.commands.ranges_walk_forward)
        self.command_ranges_walk_backward = class_to_dict(self.cfg.commands.ranges_walk_backward)

        print("self.reward_scales_walk: \n", self.reward_scales_walk)
        print("self.command_ranges_walk: \n", self.command_ranges_walk)
        print("self.command_ranges_walk_forward: \n", self.command_ranges_walk_forward)
        print("self.command_ranges_walk_backward: \n", self.command_ranges_walk_backward)

        # walk --------------------------------------------

        # run --------------------------------------------

        self.reward_scales_run = class_to_dict(self.cfg.rewards.scales_run)
        self.command_ranges_run = class_to_dict(self.cfg.commands.ranges_run)

        print("self.reward_scales_run: \n", self.reward_scales_run)
        print("self.command_ranges_run: \n", self.command_ranges_run)

        # run --------------------------------------------

    def update_command_curriculum(self, env_ids):
        if self.cfg.commands.curriculum_profile == "GR1L-option":
            pass

    def _prepare_reward_function(self):
        """ Prepares a list of reward functions, whcih will be called to compute the total reward.
            Looks for self._reward_<REWARD_NAME>, where <REWARD_NAME> are names of all non zero reward scales in the cfg.
        """
        super()._prepare_reward_function()

        # walk --------------------------------------------

        if self.num_options >= 1:
            # remove zero scales + multiply non-zero ones by dt
            for key in list(self.reward_scales_walk.keys()):
                scale = self.reward_scales_walk[key]
                if scale == 0:
                    self.reward_scales_walk.pop(key)
                else:
                    self.reward_scales_walk[key] *= self.dt

            # prepare list of functions
            self.reward_functions_walk = []
            self.reward_names_walk = []
            for name, scale in self.reward_scales_walk.items():
                if name == "termination":
                    continue
                self.reward_names_walk.append(name)
                name = '_reward_' + name
                self.reward_functions_walk.append(getattr(self, name))

            # reward episode sums
            # get name in self.reward_scales_walk.keys(), but not in self.reward_scales.keys()
            for name in set(self.reward_scales_walk.keys()) - set(self.reward_scales.keys()):
                self.episode_sums[name] = torch.zeros(self.num_envs,
                                                      dtype=torch.float, device=self.device, requires_grad=False)

        # walk --------------------------------------------

        # stand --------------------------------------------

        if self.num_options >= 2:
            # remove zero scales + multiply non-zero ones by dt
            for key in list(self.reward_scales_stand.keys()):
                scale = self.reward_scales_stand[key]
                if scale == 0:
                    self.reward_scales_stand.pop(key)
                else:
                    self.reward_scales_stand[key] *= self.dt

            # prepare list of functions
            self.reward_functions_stand = []
            self.reward_names_stand = []
            for name, scale in self.reward_scales_stand.items():
                if name == "termination":
                    continue
                self.reward_names_stand.append(name)
                name = '_reward_' + name
                self.reward_functions_stand.append(getattr(self, name))

            # reward episode sums
            # get name in self.reward_scales_stand.keys(), but not in self.reward_scales.keys()
            for name in set(self.reward_scales_stand.keys()) - set(self.reward_scales.keys()):
                self.episode_sums[name] = torch.zeros(self.num_envs,
                                                      dtype=torch.float, device=self.device, requires_grad=False)

        # stand --------------------------------------------

        # run --------------------------------------------

        if self.num_options >= 3:
            # remove zero scales + multiply non-zero ones by dt
            for key in list(self.reward_scales_run.keys()):
                scale = self.reward_scales_run[key]
                if scale == 0:
                    self.reward_scales_run.pop(key)
                else:
                    self.reward_scales_run[key] *= self.dt

            # prepare list of functions
            self.reward_functions_run = []
            self.reward_names_run = []
            for name, scale in self.reward_scales_run.items():
                if name == "termination":
                    continue
                self.reward_names_run.append(name)
                name = '_reward_' + name
                self.reward_functions_run.append(getattr(self, name))

            # reward episode sums
            # get name in self.reward_scales_run.keys(), but not in self.reward_scales.keys()
            for name in set(self.reward_scales_run.keys()) - set(self.reward_scales.keys()):
                self.episode_sums[name] = torch.zeros(self.num_envs,
                                                      dtype=torch.float, device=self.device, requires_grad=False)

        # run --------------------------------------------

    def compute_reward(self):
        """ Compute rewards
            Calls each reward function which had a non-zero scale (processed in self._prepare_reward_function())
            adds each terms to the episode sums and to the total reward
        """
        # super().compute_reward()

        self.rew_buf[:] = 0.0

        # walk --------------------------------------------

        if self.num_options >= 1:
            for i in range(len(self.reward_functions_walk)):
                name = self.reward_names_walk[i]
                rew = self.reward_functions_walk[i]() * self.reward_scales_walk[name]
                rew = rew * self.options[:, 0]  # walk
                self.rew_buf += rew
                self.episode_sums[name] += rew

        # walk --------------------------------------------

        # stand --------------------------------------------

        if self.num_options >= 2:
            for i in range(len(self.reward_functions_stand)):
                name = self.reward_names_stand[i]
                rew = self.reward_functions_stand[i]() * self.reward_scales_stand[name]
                rew = rew * self.options[:, 1]  # stand
                self.rew_buf += rew
                self.episode_sums[name] += rew

        # stand --------------------------------------------

        # run --------------------------------------------

        if self.num_options >= 3:
            for i in range(len(self.reward_functions_run)):
                name = self.reward_names_run[i]
                rew = self.reward_functions_run[i]() * self.reward_scales_run[name]
                rew = rew * self.options[:, 2]  # run
                self.rew_buf += rew
                self.episode_sums[name] += rew

        # run --------------------------------------------

        # add termination reward after clipping
        if "termination" in self.reward_scales:
            rew = self._reward_termination() * self.reward_scales["termination"]
            self.rew_buf += rew
            self.episode_sums["termination"] += rew

    def computer_observation_profile(self):
        if self.obs_profile == "GR1L-option":
            base_height = (
                torch.mean(torch.clip(
                    self.root_states[:, 2].unsqueeze(1)
                    - self.cfg.rewards.base_height_target
                    - self.measured_heights, -1, 1.) * self.obs_scales.height_measurements, dim=1))
            self.avg_x_vel = (2 * self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 0]
                              + (1 - 2 * self.dt / self.cfg.commands.gait_cycle) * self.avg_x_vel)
            self.avg_y_vel = (self.dt / self.cfg.commands.gait_cycle * self.base_lin_vel[:, 1]
                              + (1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_y_vel)
            self.avg_yaw_vel = (self.dt / self.cfg.commands.gait_cycle * self.base_ang_vel[:, 2]
                                + (1 - self.dt / self.cfg.commands.gait_cycle) * self.avg_yaw_vel)

            self.obs_buf = torch.cat(
                (
                    self.base_lin_vel * self.obs_scales.lin_vel * self.lin_vel_scales,
                    self.base_ang_vel * self.obs_scales.ang_vel,
                    self.projected_gravity,
                    self.commands[:, :3] * self.commands_scale,

                    # dof related
                    (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                    # (self.delayed_dofpos - self.default_dof_pos) * self.obs_scales.dof_pos,
                    self.dof_vel * self.obs_scales.dof_vel,
                    # self.delayed_dofvel * self.obs_scales.dof_vel,
                    self.actions,

                    # phase related
                    torch.sin(2 * torch.pi * self.gait_phase),
                    torch.cos(2 * torch.pi * self.gait_phase),
                    self.phase_ratio,

                    base_height.unsqueeze(1),
                    (self.avg_x_vel * self.lin_vel_scales[:, 0]).unsqueeze(1) * self.obs_scales.lin_vel,
                    (self.avg_y_vel * self.lin_vel_scales[:, 1]).unsqueeze(1) * self.obs_scales.lin_vel,
                    self.avg_yaw_vel.unsqueeze(1) * self.obs_scales.ang_vel,

                    # options
                    self.options,
                ), dim=-1)

    def computer_noise_scale_vec_profile(self, noise_vec, noise_scales, noise_level):
        if self.obs_profile == "GR1L-option":
            noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
            noise_vec[3:6] = noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel
            noise_vec[6:9] = noise_scales.gravity * noise_level
            noise_vec[9:12] = 0.  # commands
            noise_vec[12:32] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
            noise_vec[32:52] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
            noise_vec[52:72] = 0.  # previous actions
            noise_vec[72:78] = 0.  # leg phase and ratio
            noise_vec[78] = noise_scales.height_measurements * noise_level * self.obs_scales.height_measurements
            noise_vec[79:82] = 0.  # avg_x_vel, avg_y_vel, avg_yaw_vel
            noise_vec[82:] = 0.  # options

        return noise_vec

    def _resample_commands(self,
                           env_ids,
                           resample_command_profile=None,
                           resample_command_profile_randomize=False,
                           resample_command_log=False):

        # if no env_ids are provided, return
        if len(env_ids) == 0:
            # print("no env_ids provided for resample_commands")
            return

        # select command profile
        select_command_profile = resample_command_profile

        if select_command_profile is None:
            pass
        else:
            if len(resample_command_profile) == 1 or resample_command_profile_randomize is False:
                select_command_profile = resample_command_profile[0]
            else:
                # 给予随机种子，以当前时间为种子，确保每次随机的结果都不一样
                np.random.seed(seed=int(time()))
                random_index = np.random.randint(0, len(resample_command_profile))
                select_command_profile = resample_command_profile[random_index]

        if select_command_profile is None or select_command_profile == "GR1L-random":
            # commands
            self.commands[env_ids, 0] = torch_rand_float(self.command_ranges["lin_vel_x"][0],
                                                         self.command_ranges["lin_vel_x"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)
            self.commands[env_ids, 1] = torch_rand_float(self.command_ranges["lin_vel_y"][0],
                                                         self.command_ranges["lin_vel_y"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # set small commands to zero
            self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.1).unsqueeze(1)

            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges["ang_vel_yaw"][0],
                                                         self.command_ranges["ang_vel_yaw"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # options
            self.options[env_ids, :] = 0  # clear all options
            self.options[env_ids, 0] = 1  # walk

            # gait phase
            self.phase_ratio[env_ids, 0] = torch_rand_float(self.command_ranges["phase_ratio"][0],
                                                            self.command_ranges["phase_ratio"][1],
                                                            (len(env_ids), 1), device=self.device).squeeze(1)
            self.phase_ratio[env_ids, 1] = self.phase_ratio[env_ids, 0]
            self.theta_left[env_ids] = self.phase_ratio[env_ids, 0]
            self.theta_right[env_ids] = (self.phase_ratio[env_ids, 0] + 0.5) % 1.0

        if select_command_profile == "GR1L-walk":
            # commands
            self.commands[env_ids, 0] = torch_rand_float(self.command_ranges_walk["lin_vel_x"][0],
                                                         self.command_ranges_walk["lin_vel_x"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)
            self.commands[env_ids, 1] = torch_rand_float(self.command_ranges_walk["lin_vel_y"][0],
                                                         self.command_ranges_walk["lin_vel_y"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # set small commands to zero
            self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.1).unsqueeze(1)

            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges_walk["ang_vel_yaw"][0],
                                                         self.command_ranges_walk["ang_vel_yaw"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # options
            self.options[env_ids, :] = 0  # clear all options
            self.options[env_ids, 0] = 1  # walk

            # gait phase
            self.phase_ratio[env_ids, 0] = 0.35
            self.phase_ratio[env_ids, 1] = 0.35
            self.theta_left[env_ids] = 0.35
            self.theta_right[env_ids] = 0.85

        if select_command_profile == "GR1L-walk-forward":
            # commands
            self.commands[env_ids, 0] = torch_rand_float(self.command_ranges_walk_forward["lin_vel_x"][0],
                                                         self.command_ranges_walk_forward["lin_vel_x"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)
            self.commands[env_ids, 1] = torch_rand_float(self.command_ranges_walk_forward["lin_vel_y"][0],
                                                         self.command_ranges_walk_forward["lin_vel_y"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # set small commands to zero
            self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.1).unsqueeze(1)

            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges_walk_forward["ang_vel_yaw"][0],
                                                         self.command_ranges_walk_forward["ang_vel_yaw"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # options
            self.options[env_ids, :] = 0  # clear all options
            self.options[env_ids, 0] = 1  # walk

            # gait phase
            self.phase_ratio[env_ids, 0] = 0.35
            self.phase_ratio[env_ids, 1] = 0.35
            self.theta_left[env_ids] = 0.35
            self.theta_right[env_ids] = 0.85

        if select_command_profile == "GR1L-walk-backward":
            # commands
            self.commands[env_ids, 0] = torch_rand_float(self.command_ranges_walk_backward["lin_vel_x"][0],
                                                         self.command_ranges_walk_backward["lin_vel_x"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)
            self.commands[env_ids, 1] = torch_rand_float(self.command_ranges_walk_backward["lin_vel_y"][0],
                                                         self.command_ranges_walk_backward["lin_vel_y"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # set small commands to zero
            self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.1).unsqueeze(1)

            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges_walk_backward["ang_vel_yaw"][0],
                                                         self.command_ranges_walk_backward["ang_vel_yaw"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # options
            self.options[env_ids, :] = 0  # clear all options
            self.options[env_ids, 0] = 1  # walk

            # gait phase
            self.phase_ratio[env_ids, 0] = 0.35
            self.phase_ratio[env_ids, 1] = 0.35
            self.theta_left[env_ids] = 0.35
            self.theta_right[env_ids] = 0.85

        if select_command_profile == "GR1L-stand":
            # commands
            self.commands[env_ids, 0] = torch_rand_float(self.command_ranges_stand["lin_vel_x"][0],
                                                         self.command_ranges_stand["lin_vel_x"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)
            self.commands[env_ids, 1] = torch_rand_float(self.command_ranges_stand["lin_vel_y"][0],
                                                         self.command_ranges_stand["lin_vel_y"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # set small commands to zero
            self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.1).unsqueeze(1)

            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges_stand["ang_vel_yaw"][0],
                                                         self.command_ranges_stand["ang_vel_yaw"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # options
            self.options[env_ids, :] = 0  # clear all options
            self.options[env_ids, 1] = 1  # stand

            # gait phase
            self.phase_ratio[env_ids, 0] = 0
            self.phase_ratio[env_ids, 1] = 0
            self.theta_left[env_ids] = 0
            self.theta_right[env_ids] = 0

        if select_command_profile == "GR1L-run":
            # commands
            self.commands[env_ids, 0] = torch_rand_float(self.command_ranges_run["lin_vel_x"][0],
                                                         self.command_ranges_run["lin_vel_x"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)
            self.commands[env_ids, 1] = torch_rand_float(self.command_ranges_run["lin_vel_y"][0],
                                                         self.command_ranges_run["lin_vel_y"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # set small commands to zero
            self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.1).unsqueeze(1)

            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges_run["ang_vel_yaw"][0],
                                                         self.command_ranges_run["ang_vel_yaw"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # options
            self.options[env_ids, :] = 0  # clear all options
            self.options[env_ids, 2] = 1  # run

            # gait phase
            self.phase_ratio[env_ids, 0] = 0.65
            self.phase_ratio[env_ids, 1] = 0.65
            self.theta_left[env_ids] = 0.65
            self.theta_right[env_ids] = 0.15

        if select_command_profile == "GR1L-jump":
            # commands
            self.commands[env_ids, 0] = torch_rand_float(self.command_ranges["lin_vel_x"][0],
                                                         self.command_ranges["lin_vel_x"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)
            self.commands[env_ids, 1] = torch_rand_float(self.command_ranges["lin_vel_y"][0],
                                                         self.command_ranges["lin_vel_y"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # set small commands to zero
            self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.1).unsqueeze(1)

            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges["ang_vel_yaw"][0],
                                                         self.command_ranges["ang_vel_yaw"][1],
                                                         (len(env_ids), 1),
                                                         device=self.device).squeeze(1)

            # options
            self.options[env_ids, :] = 0  # clear all options
            self.options[env_ids, 3] = 1  # jump

            # gait phase
            self.phase_ratio[env_ids, 0] = 0.5
            self.phase_ratio[env_ids, 1] = 0.5
            self.theta_left[env_ids] = 0.5
            self.theta_right[env_ids] = 0.5

        if resample_command_log:
            print("select_command_profile: ", select_command_profile)
            print("self.commands: ", self.commands)
            print("self.phase_ratio: ", self.phase_ratio)
            print("self.theta_left: ", self.theta_left)
            print("self.theta_right: ", self.theta_right)

    # ----------------------------------------------

    # 奖赏 单脚着地
    def _reward_no_fly(self):
        contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
        single_contact = torch.sum(1. * contacts, dim=1) >= 1
        return 1. * single_contact

    # 奖赏 抬脚时的脚朝向
    def _reward_feet_orien(self):
        left_foot_height = torch.mean(
            self.rigid_body_states[:, self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(
            self.rigid_body_states[:, self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        return (torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1)
                * torch.abs(0.2 - left_foot_height)
                + torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1)
                * torch.abs(0.2 - right_foot_height))

    # 奖励 两脚之间朝向
    def _reward_feet_clearence(self):
        feet_clearence = torch.sum(torch.square(self.rigid_body_states[:, self.feet_indices][:, 0, 0:2]
                                                - self.rigid_body_states[:, self.feet_indices][:, 1, 0:2]), dim=1)
        # print((feet_clearence<=0.01)*1)
        return (feet_clearence <= 0.01) * 1

    # 惩罚 hip yaw
    def _reward_hip_yaw(self):
        # print(torch.sum(torch.abs(self.dof_pos[:,[1,7]]- self.default_dof_pos[:,[1,7]]),dim=1))
        # print(torch.abs(self.commands[:, 2]))
        return torch.sum(torch.abs(self.dof_pos[:, [1, 7]] - self.default_dof_pos[:, [1, 7]]), dim=1)

    # 惩罚 脚离地高度
    def _reward_feet_height(self):
        feet_height = self.rigid_body_states[:, self.feet_indices, 2]
        left_feet_height = torch.mean(feet_height[:, 0].unsqueeze(1) - self.measured_heights, dim=1)
        right_feet_height = torch.mean(feet_height[:, 1].unsqueeze(1) - self.measured_heights, dim=1)

        # contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
        # print(left_feet_height)
        # print(right_feet_height)
        left_za = torch.clamp(left_feet_height, 0,
                              self.cfg.rewards.feet_height_target) / self.cfg.rewards.feet_height_target
        left_za_minus_zd = left_za - 1.0

        right_za = torch.clamp(right_feet_height, 0,
                               self.cfg.rewards.feet_height_target) / self.cfg.rewards.feet_height_target
        right_za_minus_zd = right_za - 1.0

        left_r = 5 * torch.sum(
            1.5576 - (torch.exp(-(left_za_minus_zd * left_za_minus_zd)) + torch.exp(-left_za * left_za))) / len(left_za)
        right_r = 5 * torch.sum(
            1.5576 - (torch.exp(-(right_za_minus_zd * right_za_minus_zd)) + torch.exp(-right_za * right_za))) / len(
            right_za)
        # print(left_r)
        # print(right_r)
        return 0.5 * (left_r + right_r)

    # 惩罚 两脚同步触地
    def _reward_symmetric_contact(self):
        # print(self.last_feet_air_time)
        return torch.abs(self.last_feet_air_time[:, 0] - self.last_feet_air_time[:, 1])

    # 惩罚 身体高度
    def _reward_base_height_new(self):
        base_height = torch.mean(self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1)
        # print(base_height)
        # print(torch.square((base_height - self.cfg.rewards.base_height_target).clip(max=0.)))
        return torch.square((base_height - self.cfg.rewards.base_height_target).clip(max=0.))

    # 奖励 膝关节伸直
    def _reward_knee_straight(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        # print((torch.abs(self.dof_pos[:,[3,9]]) * ~contact_filt))
        return torch.sum(torch.abs(self.dof_pos[:, [3, 9]]) * ~contact_filt, dim=1)

    # 奖励 feet air time
    def _reward_last_feet_airtime(self):
        # print(self.last_feet_air_time)
        # print(torch.sum(torch.abs(self.last_feet_air_time-0.5),dim=1))
        return torch.sum(torch.abs(self.last_feet_air_time - 0.5), dim=1)

    # 惩罚 双脚离地
    def _reward_no_fly_new(self):
        contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
        command_slow = torch.norm(self.commands[:, :2], dim=1) < 0.1
        command_fast = torch.norm(self.commands[:, :2], dim=1) > 3.0
        command_mid = ~command_slow & ~command_fast
        single_contact = (torch.sum(1. * contacts, dim=1) >= 1) * command_slow + (
                torch.sum(1. * contacts, dim=1) == 1) * command_mid + (
                                 torch.sum(1. * contacts, dim=1) <= 1) * command_fast
        # print(single_contact)
        return 1. * single_contact

    #############################################################

    # 奖励 落地时的脚部接触力
    def _reward_feet_frc_osu(self):
        max_foot_frc = 350
        normed_left_frc = self.avg_feet_frc[:, 0].clip(max=max_foot_frc) / max_foot_frc
        normed_right_frc = self.avg_feet_frc[:, 1].clip(max=max_foot_frc) / max_foot_frc
        left_frc_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[0]
        right_frc_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[0]
        # left_frc_score = left_frc_clock*(1.0-torch.exp(- 10*torch.square(normed_left_frc)))
        # right_frc_score = right_frc_clock*(1.0-torch.exp(- 10*torch.square(normed_right_frc)))
        left_frc_score = left_frc_clock * (torch.exp(- 10 * torch.square(self.avg_feet_frc[:, 0])))
        right_frc_score = right_frc_clock * (torch.exp(- 10 * torch.square(self.avg_feet_frc[:, 1])))

        # print("self.avg_feet_frc")
        # print(self.avg_feet_frc)
        # print("left_frc_clock")
        # print(left_frc_clock)
        # print("right_frc_clock")
        # print(right_frc_clock)
        # print("left_frc_score + right_frc_score")
        # print(left_frc_score + right_frc_score)
        return left_frc_score + right_frc_score

    # 奖励 抬腿时的脚部速度
    def _reward_feet_spd_osu(self):
        left_spd_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[1]
        right_spd_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[1]
        left_spd_score = left_spd_clock * (torch.exp(- 5 * torch.square(self.avg_feet_spd[:, 0])))
        right_spd_score = right_spd_clock * (torch.exp(- 5 * torch.square(self.avg_feet_spd[:, 1])))
        # print(left_foot_speed)
        # print("left_spd_clock")
        # print(left_spd_score + right_spd_score)
        return left_spd_score + right_spd_score

    # 奖励 落地时的脚部朝向 Orientation Forward
    def _reward_feet_orien_osu(self):
        left_frc_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[0]
        right_frc_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[0]
        left_feet_forward = quat_apply(self.rigid_body_states[:, self.feet_indices][:, 0, 3:7], self.forward_vec)
        right_feet_forward = quat_apply(self.rigid_body_states[:, self.feet_indices][:, 1, 3:7], self.forward_vec)
        left_fori_score = left_frc_clock * (
            torch.exp(- 10 * torch.square(torch.cos(self.commands[:, 3]) - left_feet_forward[:, 0])
                      - 10 * torch.square(torch.sin(self.commands[:, 3]) - left_feet_forward[:, 1])))
        right_fori_score = right_frc_clock * (
            torch.exp(- 10 * torch.square(torch.cos(self.commands[:, 3]) - right_feet_forward[:, 0])
                      - 10 * torch.square(torch.sin(self.commands[:, 3]) - right_feet_forward[:, 1])))
        # print(left_fori_score)
        # print(right_fori_score)
        # print(torch.square(torch.cos(self.commands[:,3])-left_feet_forward[:,0])-10*torch.square(torch.sin(self.commands[:,3])-left_feet_forward[:,1]))
        return left_fori_score + right_fori_score

    # 奖励 身体朝向 Orientation Forward
    def _reward_orientation_yaw_diff_osu(self):
        # print(self.projected_gravity[:, :2])
        # print(1.0-torch.exp(-10*torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)))
        root_forward = quat_apply(self.root_states[:, 3:7], self.forward_vec)
        return (torch.exp(- 10 * torch.square(torch.cos(self.commands[:, 3]) - root_forward[:, 0])
                          - 10 * torch.square(torch.sin(self.commands[:, 3]) - root_forward[:, 1])))

    # 惩罚 身体 Orientation 水平差异
    def _reward_orientation_diff_osu(self):
        # print(self.projected_gravity[:, :2])
        # print(1.0-torch.exp(-10*torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)))
        return torch.exp(-10 * torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1))

    # 惩罚 x 速度差异
    def _reward_x_vel_diff_osu(self):
        # step 1 / 2
        # return torch.exp(-3 * (torch.abs(self.commands[:, 0] - self.avg_x_vel)))
        # step 3
        return torch.exp(-5 * (torch.abs(self.commands[:, 0] - self.avg_x_vel)))

    # 惩罚 y 速度差异
    def _reward_y_vel_diff_osu(self):
        # step 1 / 2
        # return torch.exp(-2 * (torch.abs(self.commands[:, 1] - self.avg_y_vel)))
        # step 3
        return torch.exp(-5 * (torch.abs(self.commands[:, 1] - self.avg_y_vel)))

    #
    def _reward_y_vel_lip_osu(self):
        s0 = torch.where(self.phase_ratio[:, 0] != 0, self.gait_phase[:, 0] / self.phase_ratio[:, 0], 1.0)
        s0 = s0.clip(min=0., max=1.)
        s1 = torch.where(self.phase_ratio[:, 1] != 0, self.gait_phase[:, 1] / self.phase_ratio[:, 1], 1.0)
        s1 = s1.clip(min=0., max=1.)

        v0 = 0.2
        v = v0 + s0 * 2 * v0 - s1 * 2 * v0 - 2 * v0 * (self.gait_phase[:, 0] - self.gait_phase[:, 1] + 0.5)
        # print(v)
        # print(1.0-torch.exp(-2*(torch.abs(self.commands[:, 1] - self.base_lin_vel[:, 1]))))
        return torch.exp(-5 * (torch.abs(v + self.commands[:, 1] - self.base_lin_vel[:, 1])))

    # 惩罚 z 速度差异
    def _reward_z_vel_diff_osu(self):
        # print(1.0-torch.exp(-1*(torch.abs(self.base_lin_vel[:, 2]))))
        return torch.exp(-2 * (torch.abs(self.base_lin_vel[:, 2])))

    # 惩罚 roll pitch 速度
    def _reward_ang_vel_xy_osu(self):
        ang_vel_error = torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=-1)
        # print('anvel')
        # print(ang_vel_error)
        # print(1.0-torch.exp(-ang_vel_error))
        return torch.exp(-1 * ang_vel_error)

    # 惩罚 yaw 速度差异
    def _reward_ang_vel_diff_osu(self):
        # ang_vel_error = torch.sum(torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2]),dim=-1)
        ang_vel_error = torch.abs(self.commands[:, 2] - self.avg_yaw_vel)
        # print(ang_vel_error)
        return torch.exp(-1 * ang_vel_error)

    # 惩罚 action 差异
    def _reward_action_diff_osu(self):
        # print(1.0-torch.exp(-0.05*(torch.norm(self.last_actions - self.actions, dim=-1))))
        return torch.exp(-0.05 * (torch.norm(self.last_actions - self.actions, dim=-1)))

    # 惩罚上半身 action 差异
    def _reward_upper_body_action_diff_osu(self):
        return torch.exp(-0.05 * (torch.norm(self.last_actions[:, 12:20] - self.actions[:, 12:20], dim=-1)))

    # 惩罚 髋关节 action 差异
    def _reward_hip_action_diff_osu(self):
        return torch.exp(
            -0.05 * (torch.norm(self.last_actions[:, [0, 1, 6, 7]] - self.actions[:, [0, 1, 6, 7]], dim=-1)))

    # 惩罚 膝关节 action 差异
    def _reward_knee_action_diff_osu(self):
        return torch.exp(
            -0.05 * (torch.norm(self.last_actions[:, [2, 3, 8, 9]] - self.actions[:, [2, 3, 8, 9]], dim=-1)))

    # 惩罚 踝关节 action 差异
    def _reward_ankle_action_diff_osu(self):
        return torch.exp(
            -0.05 * (torch.norm(self.last_actions[:, [4, 5, 10, 11]] - self.actions[:, [4, 5, 10, 11]], dim=-1)))

    # 惩罚 关节输出力矩
    def _reward_torque_diff_osu(self):
        # print(1.0-torch.exp(-0.0005*torch.norm(self.torques, dim=1)))
        return torch.exp(-0.0005 * torch.norm(self.torques, dim=1))

    # 惩罚 身体高度差异
    def _reward_base_height_diff_osu(self):
        # Penalize base height away from target
        left_spd_clock = 0.5 * gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0],
                                          0.02)[1] + 0.5 * \
                         gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1],
                                    0.02)[0]
        right_spd_clock = 0.5 * \
                          gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0],
                                     0.02)[0] + 0.5 * \
                          gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1],
                                     0.02)[1]
        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 2]
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 2]
        foot_height = left_spd_clock * leftfoot + right_spd_clock * rightfoot
        base_height = self.root_states[:, 2] - foot_height
        # print(base_height)
        # print(torch.abs(base_height - self.cfg.rewards.base_height_target))
        # print(1.0-torch.exp(-12*torch.norm(base_height - self.cfg.rewards.base_height_target,dim=-1)))
        # return torch.exp(-12*torch.norm(base_height - self.cfg.rewards.base_height_target,dim=-1))
        return torch.exp(-20 * torch.abs(base_height - self.cfg.rewards.base_height_target))

    # 惩罚 脚部非水平
    def _reward_feet_orien_diff_osu(self):
        left_foot_height = torch.mean(
            self.rigid_body_states[:, self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(
            self.rigid_body_states[:, self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        # print(1.0-torch.exp(-5.0*(torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1)*torch.abs(0.2-left_foot_height)+torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1)*torch.abs(0.2-right_foot_height))))
        return torch.exp(-5.0 * (torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1) * torch.abs(
            0.2 - left_foot_height)
                                 + torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1) * torch.abs(
                    0.2 - right_foot_height)))

    # 惩罚 速度太大
    def _reward_dof_vel_diff_osu(self):
        # Penalize dof velocities
        # print(torch.exp(-0.0001*torch.sum(torch.square(self.dof_vel), dim=1)))
        return torch.exp(-0.0001 * torch.sum(torch.square(self.dof_vel), dim=1))

    # 惩罚 加速度太大
    def _reward_dof_acc_diff_osu(self):
        # Penalize dof accelerations
        # print(1.0-torch.exp(-0.0000001*torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1)))
        return torch.exp(-0.0000001 * torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1))

    # 惩罚 两腿水平间距
    def _reward_feet_clearence_osu(self):
        feet_clearence = torch.sum(torch.square(
            self.rigid_body_states[:, self.feet_indices][:, 0, 0:2] - self.rigid_body_states[:, self.feet_indices][:, 1,
                                                                      0:2]), dim=1)
        # print((feet_clearence<=0.01)*1)
        return (feet_clearence <= 0.01) * 1

    # 惩罚 hip yaw
    def _reward_hip_yaw_osu(self):
        # left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],0.02)[0]
        # right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],0.02)[0]

        # left_score = left_frc_clock*(torch.exp(-3.0*torch.sum(torch.abs(self.actions[:,[1]] - self.default_dof_pos[:,[1]]),dim=1)))
        # right_score = right_frc_clock*(torch.exp(-3.0*torch.sum(torch.abs(self.actions[:,[7]] - self.default_dof_pos[:,[7]]),dim=1)))
        # return left_score+right_score
        left_frc_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[0]
        right_frc_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[0]

        left_yaw_stance = (self.gait_phase[:, 0] - 0.5)
        left_yaw_stance = left_yaw_stance.clip(min=0., max=1.) * self.cfg.commands.gait_cycle
        right_yaw_stance = (self.gait_phase[:, 1] - 0.5)
        right_yaw_stance = right_yaw_stance.clip(min=0., max=1.) * self.cfg.commands.gait_cycle
        left_yaw_score = left_frc_clock * torch.exp(
            -3.0 * torch.abs(self.actions[:, 1] - self.default_dof_pos[:, 1])) + (1 - left_frc_clock) * torch.exp(
            -3.0 * torch.abs(self.dof_pos[:, 1] + left_yaw_stance * self.commands[:, 2]))
        right_yaw_score = right_frc_clock * torch.exp(
            -3.0 * torch.abs(self.actions[:, 7] - self.default_dof_pos[:, 7])) + (1 - right_frc_clock) * torch.exp(
            -3.0 * torch.abs(self.dof_pos[:, 7] + right_yaw_stance * self.commands[:, 2]))
        return left_yaw_score + right_yaw_score

    # 惩罚 接近位置上限
    def _reward_dof_pos_limits_osu(self):
        # Penalize dof positions too close to the limit
        # out_of_limits = -(self.dof_pos - self.dof_pos_limits[:, 0]).clip(max=0.) # lower limit
        # out_of_limits += (self.dof_pos - self.dof_pos_limits[:, 1]).clip(min=0.)
        out_of_limits = -(self.dof_pos[:, [0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19]] -
                          self.dof_pos_limits[:, 0][[0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19]]).clip(
            max=0.)  # lower limit
        out_of_limits += (self.dof_pos[:, [0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19]] -
                          self.dof_pos_limits[:, 1][[0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19]]).clip(
            min=0.)
        # print(1.0-torch.exp(-2.0*torch.sum(out_of_limits, dim=1)))
        return torch.exp(-2.0 * torch.sum(out_of_limits, dim=1))

    # 惩罚 超过速度上限
    def _reward_dof_vel_limits_osu(self):
        # Penalize dof velocities too close to the limit
        # clip to max error = 1 rad/s per joint to avoid huge penalties
        # print(1.0-torch.exp(-0.1*torch.sum((torch.abs(self.dof_vel) - self.dof_vel_limits*self.cfg.rewards.soft_dof_vel_limit).clip(min=0., max=1.), dim=1)))
        return torch.exp(-0.1 * torch.sum(
            (torch.abs(self.dof_vel) - self.dof_vel_limits * self.cfg.rewards.soft_dof_vel_limit).clip(min=0., max=1.),
            dim=1))

    # 惩罚 超过力矩上限
    def _reward_torque_limits_osu(self):
        # penalize torques too close to the limit
        # print(1.0-torch.exp(-0.005*torch.sum((torch.abs(self.torques) - self.torque_limits*self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1)))
        return torch.exp(-0.005 * torch.sum(
            (torch.abs(self.torques) - self.torque_limits * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))

    # 惩罚 落地 速度
    def _reward_swing_tracking(self):
        s0 = torch.where(self.phase_ratio[:, 0] != 0, self.gait_phase[:, 0] / self.phase_ratio[:, 0] - 0.5, 1.0)
        s0 = s0.clip(min=0., max=1.)
        s0 = s0 * (s0 < 0.6)
        s0 = s0.clip(min=0., max=0.5)
        s1 = torch.where(self.phase_ratio[:, 1] != 0, self.gait_phase[:, 1] / self.phase_ratio[:, 1] - 0.5, 1.0)
        s1 = s1.clip(min=0., max=1.)
        s1 = s1 * (s1 < 0.6)
        s1 = s1.clip(min=0., max=0.5)

        # h0 = 0.96*s0*s0 - 1.92*s0*s0*s0 + 0.96*s0*s0*s0*s0
        # h1 = 0.96*s1*s1 - 1.92*s1*s1*s1 + 0.96*s1*s1*s1*s1
        # v0 = (2.0*0.96*s0 - 3.0*1.92*s0*s0 + 4.0*0.96*s0*s0*s0)/(self.cfg.commands.gait_cycle*self.phase_ratio[:,0])
        # v1 = (2.0*0.96*s1 - 3.0*1.92*s1*s1 + 4.0*0.96*s1*s1*s1)/(self.cfg.commands.gait_cycle*self.phase_ratio[:,1])
        # left_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],0.02)[0]
        # right_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],0.02)[0]
        # leftfoot_h = self.rigid_body_states[:,self.feet_indices][:, 0, 2] - 0.032#torch.mean(self.measured_heights, dim=-1) - 0.032
        # rightfoot_h = self.rigid_body_states[:,self.feet_indices][:, 1, 2] - torch.mean(self.measured_heights, dim=-1) - 0.032
        leftfoot_v = self.rigid_body_states[:, self.feet_indices][:, 0, 9]
        rightfoot_v = self.rigid_body_states[:, self.feet_indices][:, 1, 9]

        # left_score = left_clock*(1-torch.exp(-(100.*torch.abs(h0 - leftfoot_h)+10.*torch.abs(v0 - leftfoot_v))))
        # right_score = right_clock*(1-torch.exp(-(100.*torch.abs(h1 - rightfoot_h)+10.*torch.abs(v1 - rightfoot_v))))
        # left_score = 4*s0*(torch.exp(-20.*torch.abs(0 - leftfoot_v)))
        # right_score = 4*s1*(torch.exp(-20.*torch.abs(0 - rightfoot_v)))
        left_score = 16 * s0 * s0 * leftfoot_v * leftfoot_v
        right_score = 16 * s1 * s1 * rightfoot_v * rightfoot_v

        # print(v1)
        # print("Height")
        # print(rightfoot_h)
        # print("Velocity")
        # print(rightfoot_v)
        # print(left_score+right_score)
        # print(1-torch.exp(-1*(10.*torch.abs(v0 - leftfoot_v))))

        return left_score + right_score

    # 奖励 两脚距离差
    def _reward_feet_distance(self):
        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 0:3] - self.root_states[:, 0:3]
        leftfoot = quat_apply(quat_conjugate(self.root_states[:, 3:7]), leftfoot)
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 0:3] - self.root_states[:, 0:3]
        rightfoot = quat_apply(quat_conjugate(self.root_states[:, 3:7]), rightfoot)
        feet_distance_x = torch.abs(leftfoot[:, 0] - rightfoot[:, 0])
        feet_distance_y = torch.abs(leftfoot[:, 1] - rightfoot[:, 1] - 0.22)
        # print(leftfoot[:,1]-rightfoot[:,1])
        return 0.5 * torch.exp(-10 * feet_distance_x) + 0.5 * torch.exp(-10 * feet_distance_y)

    # 奖励 抬腿时的两脚高度差
    def _reward_swing_height(self):
        s0 = torch.where(self.phase_ratio[:, 0] != 0, self.gait_phase[:, 0] / self.phase_ratio[:, 0], 1.0)
        s0 = s0.clip(min=0., max=1.)
        s0 = s0 * (s0 < 0.5)
        s1 = torch.where(self.phase_ratio[:, 1] != 0, self.gait_phase[:, 1] / self.phase_ratio[:, 1], 1.0)
        s1 = s1.clip(min=0., max=1.)
        s1 = s1 * (s1 < 0.5)

        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 2]
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 2]
        # print(leftfoot-rightfoot)
        # print(s0*(leftfoot - rightfoot - 0.1) + s1*(rightfoot - leftfoot - 0.1))
        left_score = 2 * s0 * (torch.exp(-25. * torch.abs(leftfoot - rightfoot - 0.04)))
        right_score = 2 * s1 * (torch.exp(-25. * torch.abs(rightfoot - leftfoot - 0.04)))
        # print(left_score + right_score)
        return left_score + right_score

    #
    def _reward_swing_symmetric(self):
        left_spd_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[
                1].unsqueeze(1)
        right_spd_clock = \
            gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[
                1].unsqueeze(1)
        feet_distance = self.rigid_body_states[:, self.feet_indices][:, 0, 0:3] - self.rigid_body_states[:,
                                                                                  self.feet_indices][:, 1, 0:3]
        feet_distance = quat_apply(quat_conjugate(self.root_states[:, 3:7]), feet_distance)
        db_flag = (left_spd_clock > 0.5) * (right_spd_clock > 0.5)
        score = torch.abs(feet_distance[:, 0] + self.last_feet_distance[:, 0]).unsqueeze(1)
        self.feet_distance = db_flag * (feet_distance) + (~db_flag) * self.feet_distance
        self.last_feet_distance = (~db_flag) * (self.feet_distance) + db_flag * self.last_feet_distance
        # print((db_flag*torch.exp(-10*score)).size())
        # print(self.feet_distance[:,0])
        # print(self.last_feet_distance[:,0])
        return (3.3 * db_flag * torch.exp(-10 * score)).squeeze(1)

    # 惩罚 feet 离地以后的速度
    def _reward_feet_speed_diff_osu(self):
        left_foot_height = torch.mean(
            self.rigid_body_states[:, self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(
            self.rigid_body_states[:, self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        # print("left_foot_height")
        # print(left_foot_height)
        # print(self.rigid_body_states[:,self.feet_indices][:, 0, 7:10])
        # print(self.avg_feet_spd[:,0])
        # print(torch.exp(-1.0*(torch.square(self.avg_feet_spd[:,0])*20.0*torch.abs(0.052-left_foot_height)+torch.square(self.avg_feet_spd[:,1])*20.0*torch.abs(0.052-right_foot_height))))
        return torch.exp(-1.0 * (torch.square(self.avg_feet_spd[:, 0]) * 20.0 * torch.abs(0.052 - left_foot_height)
                                 + torch.square(self.avg_feet_spd[:, 1]) * 20.0 * torch.abs(0.052 - right_foot_height)))

    # 惩罚 ankle 扭矩
    def _reward_ankle_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        return torch.sum(torch.square(self.torques[:, [4, 10]]), dim=1)

    # 惩罚 knee 扭矩
    def _reward_knee_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        # return torch.sum(torch.square(self.torques[:,[3,9]]), dim=1)
        return torch.sum(torch.square(self.torques[:, [3, 9]]), dim=1) \
            * torch.max(torch.exp(-15 * torch.abs(self.avg_x_vel)), torch.abs(self.avg_x_vel) < 0.2)

    # 惩罚 arm 的运动
    def _reward_arm_pose(self):
        # print(torch.sum(torch.abs(self.actions[:,[16,17,18,20,21,22]]), dim=1))
        return torch.sum(torch.abs(self.actions[:, [13, 14, 15, 17, 18, 19]]), dim=1)

    # 奖励 手和初始位置之间的距离
    def _reward_arm_swing(self):
        # print(torch.sum(torch.abs(self.actions[:,[15,19]] - 2*self.dof_pos[:,[8,2]]), dim=1))
        return torch.sum(torch.abs(self.actions[:, [12, 16]] - 2 * self.dof_pos[:, [8, 2]]), dim=1)

    # 奖励 手和脚之间的 x 轴向距离
    def _reward_swing_arm(self):
        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 0:3] - self.root_states[:, 0:3]
        leftfoot = quat_apply(quat_conjugate(self.root_states[:, 3:7]), leftfoot)
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 0:3] - self.root_states[:, 0:3]
        rightfoot = quat_apply(quat_conjugate(self.root_states[:, 3:7]), rightfoot)

        lefthand = (self.rigid_body_states[:, self.lower_arm_indices][:, 0, 0:3]
                    - self.root_states[:, 0:3]
                    + quat_rotate(self.rigid_body_states[:, self.lower_arm_indices][:, 0, 3:7],
                                  self.arm_left_local_vec))
        righthand = (self.rigid_body_states[:, self.lower_arm_indices][:, 1, 0:3]
                     - self.root_states[:, 0:3]
                     + quat_rotate(self.rigid_body_states[:, self.lower_arm_indices][:, 1, 3:7],
                                   self.arm_right_local_vec))
        lefthand = quat_apply(quat_conjugate(self.root_states[:, 3:7]), lefthand)
        righthand = quat_apply(quat_conjugate(self.root_states[:, 3:7]), righthand)

        left_distance = torch.abs(lefthand[:, 0] - 0.9 * rightfoot[:, 0])
        right_distance = torch.abs(righthand[:, 0] - 0.9 * leftfoot[:, 0])
        # print("hand")
        # print(lefthand[:,0])
        # print('foot')
        # print(0.9*rightfoot[:,0])
        return 0.5 * torch.exp(-10 * left_distance) + 0.5 * torch.exp(-10 * right_distance)

    # 惩罚 上半身 Orientation 不水平
    def _reward_torso_orientation_diff_osu(self):
        # print(self.rigid_body_states[:,self.torso_indices])

        torso_projected_gravity = quat_rotate_inverse(self.rigid_body_states[:, self.torso_indices][:, 0, 3:7],
                                                      self.gravity_vec)

        return 10.0 * torch.sum(torch.square(torso_projected_gravity[:, :2]), dim=1)

    # 惩罚 上半身 Ang Vel
    def _reward_torso_ang_vel_xy_osu(self):
        torso_ang_vel_error = torch.sum(torch.square(self.rigid_body_states[:, self.torso_indices][:, 0, 10:12]),
                                        dim=-1)
        # print('anvel')
        # print(self.rigid_body_states[:,self.torso_indices][:,0,10:13])
        # print(torso_ang_vel_error)
        return torso_ang_vel_error

    # 惩罚 上半身 Pitch
    def _reward_orientation_pitch_osu(self):
        pitch = get_euler_xyz(self.root_states[:, 3:7])[1]
        return torch.sin(0.5 * pitch)

    # 奖励 训练手移动
    def _reward_reaching(self):
        erro_left = torch.sum(
            torch.square(self.rigid_body_states[:, self.hand_indices][:, 0, :3] - self.reaching_target[:, :3]), dim=1)
        erro_right = torch.sum(
            torch.square(self.rigid_body_states[:, self.hand_indices][:, 1, :3] - self.reaching_target[:, 3:6]), dim=1)
        return erro_left + erro_right
