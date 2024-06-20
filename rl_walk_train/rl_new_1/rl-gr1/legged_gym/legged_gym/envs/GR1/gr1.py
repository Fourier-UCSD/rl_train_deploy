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
from .clockgait import trapezoidclockgait as gait_clock


class GR1(LeggedRobot):

    def _reward_no_fly(self):
        contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
        single_contact = torch.sum(1. * contacts, dim=1) >= 1
        return 1. * single_contact

    def _reward_feet_orien(self):
        left_foot_height = torch.mean(self.rigid_body_states[:, self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(self.rigid_body_states[:, self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        return torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1) * torch.abs(0.2 - left_foot_height) + torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1) * torch.abs(0.2 - right_foot_height)

    def _reward_feet_clearence(self):
        feet_clearence = torch.sum(torch.square(self.rigid_body_states[:, self.feet_indices][:, 0, 0:2] - self.rigid_body_states[:, self.feet_indices][:, 1, 0:2]), dim=1)
        # print((feet_clearence<=0.01)*1)
        return (feet_clearence <= 0.01) * 1

    def _reward_hipyaw(self):
        # print(torch.sum(torch.abs(self.dof_pos[:,[1,7]]- self.default_dof_pos[:,[1,7]]),dim=1))
        # print(torch.abs(self.commands[:, 2]))
        return torch.sum(torch.abs(self.dof_pos[:, [1, 7]] - self.default_dof_pos[:, [1, 7]]), dim=1)

    def _reward_feetheight(self):
        feetheight = self.rigid_body_states[:, self.feet_indices, 2]
        leftfeetheight = torch.mean(feetheight[:, 0].unsqueeze(1) - self.measured_heights, dim=1)
        rightfeetheight = torch.mean(feetheight[:, 1].unsqueeze(1) - self.measured_heights, dim=1)

        # contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1  
        # print(leftfeetheight)
        # print(rightfeetheight)
        left_za = torch.clamp(leftfeetheight, 0, self.cfg.rewards.feetheight_target) / self.cfg.rewards.feetheight_target
        left_za_minus_zd = left_za - 1.0
        # print(left_za)

        right_za = torch.clamp(rightfeetheight, 0, self.cfg.rewards.feetheight_target) / self.cfg.rewards.feetheight_target
        right_za_minus_zd = right_za - 1.0
        left_r = 5 * torch.sum(1.5576 - (torch.exp(-(left_za_minus_zd * left_za_minus_zd)) + torch.exp(-left_za * left_za))) / len(left_za)
        right_r = 5 * torch.sum(1.5576 - (torch.exp(-(right_za_minus_zd * right_za_minus_zd)) + torch.exp(-right_za * right_za))) / len(right_za);
        # print(left_r)
        # print(right_r)
        return 0.5 * (left_r + right_r)

    def _reward_symmetric_contact(self):
        # print(self.last_feet_air_time)
        return torch.abs(self.last_feet_air_time[:, 0] - self.last_feet_air_time[:, 1])

    def _reward_base_height_new(self):
        base_height = torch.mean(self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1)
        # print(base_height)
        # print(torch.square((base_height - self.cfg.rewards.base_height_target).clip(max=0.)))
        return torch.square((base_height - self.cfg.rewards.base_height_target).clip(max=0.))

    def _reward_knee_straight(self):
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.
        contact_filt = torch.logical_or(contact, self.last_contacts)
        # print((torch.abs(self.dof_pos[:,[3,9]]) * ~contact_filt))
        return torch.sum(torch.abs(self.dof_pos[:, [3, 9]]) * ~contact_filt, dim=1)

    def _reward_last_feet_airtime(self):
        # print(self.last_feet_air_time)
        # print(torch.sum(torch.abs(self.last_feet_air_time-0.5),dim=1))
        return torch.sum(torch.abs(self.last_feet_air_time - 0.5), dim=1)

    def _reward_no_fly_new(self):
        contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
        command_slow = torch.norm(self.commands[:, :2], dim=1) < 0.1
        command_fast = torch.norm(self.commands[:, :2], dim=1) > 3.0
        command_mid = ~command_slow & ~command_fast
        single_contact = (torch.sum(1. * contacts, dim=1) >= 1) * command_slow + (torch.sum(1. * contacts, dim=1) == 1) * command_mid + (torch.sum(1. * contacts, dim=1) <= 1) * command_fast
        # print(single_contact)
        return 1. * single_contact

    #############################################################

    def _reward_feet_frc_osu(self):
        max_foot_frc = 350
        normed_left_frc = self.avg_feet_frc[:, 0].clip(max=max_foot_frc) / max_foot_frc
        normed_right_frc = self.avg_feet_frc[:, 1].clip(max=max_foot_frc) / max_foot_frc
        left_frc_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[0]
        right_frc_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[0]
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

    def _reward_feet_spd_osu(self):
        left_spd_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[1]
        right_spd_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[1]
        left_spd_score = left_spd_clock * (torch.exp(- 2 * torch.square(self.avg_feet_spd[:, 0])))
        right_spd_score = right_spd_clock * (torch.exp(- 2 * torch.square(self.avg_feet_spd[:, 1])))
        # print(left_foot_speed)
        # print("left_spd_clock")
        # print(left_spd_score + right_spd_score)
        return left_spd_score + right_spd_score

    def _reward_feet_orien_osu(self):
        left_frc_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[0]
        right_frc_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[0]
        left_feet_forward = quat_apply(self.rigid_body_states[:, self.feet_indices][:, 0, 3:7], self.forward_vec)
        right_feet_forward = quat_apply(self.rigid_body_states[:, self.feet_indices][:, 1, 3:7], self.forward_vec)
        left_fori_score = left_frc_clock * (torch.exp(- 10 * torch.square(torch.cos(self.commands[:, 3]) - left_feet_forward[:, 0]) - 10 * torch.square(torch.sin(self.commands[:, 3]) - left_feet_forward[:, 1])))
        right_fori_score = right_frc_clock * (torch.exp(- 10 * torch.square(torch.cos(self.commands[:, 3]) - right_feet_forward[:, 0]) - 10 * torch.square(torch.sin(self.commands[:, 3]) - right_feet_forward[:, 1])))
        # print(left_fori_score)
        # print(right_fori_score)
        # print(torch.square(torch.cos(self.commands[:,3])-left_feet_forward[:,0])-10*torch.square(torch.sin(self.commands[:,3])-left_feet_forward[:,1]))
        return left_fori_score + right_fori_score

    def _reward_orientation_yaw_diff_osu(self):
        # print(self.projected_gravity[:, :2])
        # print(1.0-torch.exp(-10*torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)))
        root_forward = quat_apply(self.root_states[:, 3:7], self.forward_vec)
        return (torch.exp(- 10 * torch.square(torch.cos(self.commands[:, 3]) - root_forward[:, 0]) - 10 * torch.square(torch.sin(self.commands[:, 3]) - root_forward[:, 1])))

    def _reward_orientation_diff_osu(self):
        # print(self.projected_gravity[:, :2])
        # print(1.0-torch.exp(-10*torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)))
        return torch.exp(-10 * torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1))

    def _reward_x_vel_diff_osu(self):
        # print("velerr")
        # print(torch.abs(self.commands[:, 0] - self.base_lin_vel[:, 0]))
        # print(torch.exp(-2*(torch.abs(self.commands[:, 0] - self.avg_x_vel))))
        # return torch.exp(-5*(torch.abs(self.commands[:, 0] - self.base_lin_vel[:, 0])))
        return torch.exp(-3 * (torch.abs(self.commands[:, 0] - self.avg_x_vel)))

    def _reward_y_vel_diff_osu(self):
        # print(1.0-torch.exp(-2*(torch.abs(self.commands[:, 1] - self.base_lin_vel[:, 1]))))
        # return torch.exp(-5*(torch.abs(self.commands[:, 1] - self.base_lin_vel[:, 1])))
        return torch.exp(-2 * (torch.abs(self.commands[:, 1] - self.avg_y_vel)))

    def _reward_y_vel_lip_osu(self):
        s0 = self.gait_phase[:, 0] / self.phase_ratio[:, 0]
        s0 = s0.clip(min=0., max=1.)
        s1 = self.gait_phase[:, 1] / self.phase_ratio[:, 1]
        s1 = s1.clip(min=0., max=1.)
        v0 = 0.2
        v = v0 + s0 * 2 * v0 - s1 * 2 * v0 - 2 * v0 * (self.gait_phase[:, 0] - self.gait_phase[:, 1] + 0.5)
        # print(v)
        # print(1.0-torch.exp(-2*(torch.abs(self.commands[:, 1] - self.base_lin_vel[:, 1]))))
        return torch.exp(-5 * (torch.abs(v + self.commands[:, 1] - self.base_lin_vel[:, 1])))

    def _reward_z_vel_diff_osu(self):
        # print(1.0-torch.exp(-1*(torch.abs(self.base_lin_vel[:, 2]))))
        return torch.exp(-2 * (torch.abs(self.base_lin_vel[:, 2])))

    def _reward_ang_vel_xy_osu(self):
        ang_vel_error = torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=-1)
        # print('anvel')
        # print(ang_vel_error)
        # print(1.0-torch.exp(-ang_vel_error))
        return torch.exp(-1 * ang_vel_error)

    def _reward_ang_vel_diff_osu(self):
        # ang_vel_error = torch.sum(torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2]),dim=-1)
        ang_vel_error = torch.abs(self.commands[:, 2] - self.avg_yaw_vel)
        # print(ang_vel_error)
        return torch.exp(-1 * ang_vel_error)

    def _reward_action_diff_osu(self):
        # print(1.0-torch.exp(-0.05*(torch.norm(self.last_actions - self.actions, dim=-1))))
        return torch.exp(-0.05 * (torch.norm(self.last_actions - self.actions, dim=-1)))

    def _reward_ankleaction_diff_osu(self):
        return torch.exp(-0.05 * (torch.norm(self.last_actions[:, [4, 5, 10, 11]] - self.actions[:, [4, 5, 10, 11]], dim=-1)))

    def _reward_upperbody_action_diff_osu(self):
        return torch.exp(-0.05 * (torch.norm(self.last_actions[:, 12:23] - self.actions[:, 12:23], dim=-1)))

    def _reward_hipaction_diff_osu(self):
        return torch.exp(-0.05 * (torch.norm(self.last_actions[:, [0, 1, 6, 7]] - self.actions[:, [0, 1, 6, 7]], dim=-1)))

    def _reward_kneeaction_diff_osu(self):
        return torch.exp(-0.05 * (torch.norm(self.last_actions[:, [2, 3, 8, 9]] - self.actions[:, [2, 3, 8, 9]], dim=-1)))

    def _reward_torque_diff_osu(self):
        # print(1.0-torch.exp(-0.0005*torch.norm(self.torques, dim=1)))
        return torch.exp(-0.0005 * torch.norm(self.torques, dim=1))

    def _reward_base_height_diff_osu(self):
        # Penalize base height away from target
        left_spd_clock = 0.5 * gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[1] + 0.5 * gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[0]
        right_spd_clock = 0.5 * gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[0] + 0.5 * gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[1]
        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 2]
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 2]
        foot_height = left_spd_clock * leftfoot + right_spd_clock * rightfoot
        base_height = self.root_states[:, 2] - foot_height
        # print(base_height)
        # print(torch.abs(base_height - self.cfg.rewards.base_height_target))
        # print(1.0-torch.exp(-12*torch.norm(base_height - self.cfg.rewards.base_height_target,dim=-1)))
        # return torch.exp(-12*torch.norm(base_height - self.cfg.rewards.base_height_target,dim=-1))
        return torch.exp(-20 * torch.abs(base_height - self.cfg.rewards.base_height_target))

    def _reward_feet_orien_diff_osu(self):
        left_foot_height = torch.mean(self.rigid_body_states[:, self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(self.rigid_body_states[:, self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        # print(1.0-torch.exp(-5.0*(torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1)*torch.abs(0.2-left_foot_height)+torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1)*torch.abs(0.2-right_foot_height))))
        return torch.exp(-5.0 * (torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1) * torch.abs(0.2 - left_foot_height) + torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1) * torch.abs(0.2 - right_foot_height)))

    def _reward_dof_vel_diff_osu(self):
        # Penalize dof velocities
        # print(torch.exp(-0.0001*torch.sum(torch.square(self.dof_vel), dim=1)))
        return torch.exp(-0.0001 * torch.sum(torch.square(self.dof_vel), dim=1))

    def _reward_dof_acc_diff_osu(self):
        # Penalize dof accelerations
        # print(1.0-torch.exp(-0.0000001*torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1)))
        return torch.exp(-0.0000001 * torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1))

    def _reward_feet_clearence_osu(self):
        feet_clearence = torch.sum(torch.square(self.rigid_body_states[:, self.feet_indices][:, 0, 0:2] - self.rigid_body_states[:, self.feet_indices][:, 1, 0:2]), dim=1)
        # print((feet_clearence<=0.01)*1)
        return (feet_clearence <= 0.01) * 1

    def _reward_hipyaw_osu(self):
        # left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],0.02)[0]
        # right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],0.02)[0]

        # left_score = left_frc_clock*(torch.exp(-3.0*torch.sum(torch.abs(self.actions[:,[1]] - self.default_dof_pos[:,[1]]),dim=1)))
        # right_score = right_frc_clock*(torch.exp(-3.0*torch.sum(torch.abs(self.actions[:,[7]] - self.default_dof_pos[:,[7]]),dim=1))) 
        # return left_score+right_score
        left_frc_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[0]
        right_frc_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[0]
        left_yaw_stance = (self.gait_phase[:, 0] - 0.5)
        left_yaw_stance = left_yaw_stance.clip(min=0., max=1.) * self.cfg.commands.gait_cycle
        right_yaw_stance = (self.gait_phase[:, 1] - 0.5)
        right_yaw_stance = right_yaw_stance.clip(min=0., max=1.) * self.cfg.commands.gait_cycle
        left_yaw_score = left_frc_clock * torch.exp(-3.0 * torch.abs(self.actions[:, 1] - self.default_dof_pos[:, 1])) + (1 - left_frc_clock) * torch.exp(-3.0 * torch.abs(self.dof_pos[:, 1] + left_yaw_stance * self.commands[:, 2]))
        right_yaw_score = right_frc_clock * torch.exp(-3.0 * torch.abs(self.actions[:, 7] - self.default_dof_pos[:, 7])) + (1 - right_frc_clock) * torch.exp(-3.0 * torch.abs(self.dof_pos[:, 7] + right_yaw_stance * self.commands[:, 2]))
        return left_yaw_score + right_yaw_score

    def _reward_dof_pos_limits_osu(self):
        # Penalize dof positions too close to the limit
        # out_of_limits = -(self.dof_pos - self.dof_pos_limits[:, 0]).clip(max=0.) # lower limit
        # out_of_limits += (self.dof_pos - self.dof_pos_limits[:, 1]).clip(min=0.)
        out_of_limits = -(self.dof_pos[:, [0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]] - self.dof_pos_limits[:, 0][[0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]]).clip(max=0.)  # lower limit
        out_of_limits += (self.dof_pos[:, [0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]] - self.dof_pos_limits[:, 1][[0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]]).clip(min=0.)
        # print(1.0-torch.exp(-2.0*torch.sum(out_of_limits, dim=1)))
        return torch.exp(-2.0 * torch.sum(out_of_limits, dim=1))

    def _reward_dof_vel_limits_osu(self):
        # Penalize dof velocities too close to the limit
        # clip to max error = 1 rad/s per joint to avoid huge penalties
        # print(1.0-torch.exp(-0.1*torch.sum((torch.abs(self.dof_vel) - self.dof_vel_limits*self.cfg.rewards.soft_dof_vel_limit).clip(min=0., max=1.), dim=1)))
        return torch.exp(-0.1 * torch.sum((torch.abs(self.dof_vel) - self.dof_vel_limits * self.cfg.rewards.soft_dof_vel_limit).clip(min=0., max=1.), dim=1))

    def _reward_torque_limits_osu(self):
        # penalize torques too close to the limit
        # print(1.0-torch.exp(-0.005*torch.sum((torch.abs(self.torques) - self.torque_limits*self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1)))
        return torch.exp(-0.005 * torch.sum((torch.abs(self.torques) - self.torque_limits * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))

    def _reward_swing_tracking(self):
        s0 = self.gait_phase[:, 0] / self.phase_ratio[:, 0] - 0.5
        s0 = s0.clip(min=0., max=1.)
        s0 = s0 * (s0 < 0.6)
        s0 = s0.clip(min=0., max=0.5)
        s1 = self.gait_phase[:, 1] / self.phase_ratio[:, 1] - 0.5
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

    def _reward_feet_distance(self):
        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 0:3] - self.root_states[:, 0:3]
        leftfoot = quat_apply(quat_conjugate(self.root_states[:, 3:7]), leftfoot)
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 0:3] - self.root_states[:, 0:3]
        rightfoot = quat_apply(quat_conjugate(self.root_states[:, 3:7]), rightfoot)
        feet_distance_x = torch.abs(leftfoot[:, 0] - rightfoot[:, 0])
        feet_distance_y = torch.abs(leftfoot[:, 1] - rightfoot[:, 1] - 0.22)
        # print(leftfoot[:,1]-rightfoot[:,1])
        return 0.5 * torch.exp(-10 * feet_distance_x) + 0.5 * torch.exp(-10 * feet_distance_y)

    def _reward_swing_height(self):
        s0 = self.gait_phase[:, 0] / self.phase_ratio[:, 0]
        # s0 = s0.clip(min=0., max=1.)
        s0 = s0 * (s0 < 0.5)
        s1 = self.gait_phase[:, 1] / self.phase_ratio[:, 1]
        # s1 = s1.clip(min=0., max=1.)
        s1 = s1 * (s1 < 0.5)
        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 2]
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 2]
        # print(leftfoot-rightfoot)
        # print(s0*(leftfoot - rightfoot - 0.1) + s1*(rightfoot - leftfoot - 0.1))
        left_score = 2 * s0 * (torch.exp(-25. * torch.abs(leftfoot - rightfoot - 0.04)))
        right_score = 2 * s1 * (torch.exp(-25. * torch.abs(rightfoot - leftfoot - 0.04)))
        # print(left_score + right_score)
        return left_score + right_score

    def _reward_swing_symmetric(self):
        left_spd_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[1].unsqueeze(1)
        right_spd_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[1].unsqueeze(1)
        feet_distance = self.rigid_body_states[:, self.feet_indices][:, 0, 0:3] - self.rigid_body_states[:, self.feet_indices][:, 1, 0:3]
        feet_distance = quat_apply(quat_conjugate(self.root_states[:, 3:7]), feet_distance)
        db_flag = (left_spd_clock > 0.5) * (right_spd_clock > 0.5)
        score = torch.abs(feet_distance[:, 0] + self.last_feet_distance[:, 0]).unsqueeze(1)
        self.feet_distance = db_flag * (feet_distance) + (~db_flag) * self.feet_distance
        self.last_feet_distance = (~db_flag) * (self.feet_distance) + db_flag * self.last_feet_distance
        # print((db_flag*torch.exp(-10*score)).size())
        # print(self.feet_distance[:,0])
        # print(self.last_feet_distance[:,0])
        return (3.3 * db_flag * torch.exp(-10 * score)).squeeze(1)

    def _reward_feet_speed_diff_osu(self):
        left_foot_height = torch.mean(self.rigid_body_states[:, self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(self.rigid_body_states[:, self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        # print("left_foot_height")
        # print(left_foot_height)
        # print(self.rigid_body_states[:,self.feet_indices][:, 0, 7:10])
        # print(self.avg_feet_spd[:,0])
        # print(torch.exp(-1.0*(torch.square(self.avg_feet_spd[:,0])*20.0*torch.abs(0.052-left_foot_height)+torch.square(self.avg_feet_spd[:,1])*20.0*torch.abs(0.052-right_foot_height))))
        return torch.exp(-1.0 * (torch.square(self.avg_feet_spd[:, 0]) * 20.0 * torch.abs(0.052 - left_foot_height) + torch.square(self.avg_feet_spd[:, 1]) * 20.0 * torch.abs(0.052 - right_foot_height)))

    def _reward_ankle_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        return torch.sum(torch.square(self.torques[:, [4, 10]]), dim=1)

    def _reward_knee_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        # return torch.sum(torch.square(self.torques[:,[3,9]]), dim=1)
        return torch.sum(torch.square(self.torques[:, [3, 9]]), dim=1) * torch.max(torch.exp(-15 * torch.abs(self.avg_x_vel)), torch.abs(self.avg_x_vel) < 0.2)

    def _reward_arm_pose(self):
        # print(torch.sum(torch.abs(self.actions[:,[16,17,18,20,21,22]]), dim=1))
        return torch.sum(torch.abs(self.actions[:, [16, 17, 18, 20, 21, 22]]), dim=1)

    def _reward_arm_swing(self):
        # print(torch.sum(torch.abs(self.actions[:,[15,19]] - 2*self.dof_pos[:,[8,2]]), dim=1))
        return torch.sum(torch.abs(self.actions[:, [15, 19]] - 2 * self.dof_pos[:, [8, 2]]), dim=1)

    def _reward_swing_arm(self):
        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 0:3] - self.root_states[:, 0:3]
        leftfoot = quat_apply(quat_conjugate(self.root_states[:, 3:7]), leftfoot)
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 0:3] - self.root_states[:, 0:3]
        rightfoot = quat_apply(quat_conjugate(self.root_states[:, 3:7]), rightfoot)
        lefthand = self.rigid_body_states[:, self.lower_arm_indices][:, 0, :3] - self.root_states[:, 0:3] + quat_rotate(self.rigid_body_states[:, self.lower_arm_indices][:, 0, 3:7], self.arm_left_local_vec)
        righthand = self.rigid_body_states[:, self.lower_arm_indices][:, 1, :3] - self.root_states[:, 0:3] + quat_rotate(self.rigid_body_states[:, self.lower_arm_indices][:, 1, 3:7], self.arm_right_local_vec)
        lefthand = quat_apply(quat_conjugate(self.root_states[:, 3:7]), lefthand)
        righthand = quat_apply(quat_conjugate(self.root_states[:, 3:7]), righthand)

        left_distance = torch.abs(lefthand[:, 0] - 0.9 * rightfoot[:, 0])
        right_distance = torch.abs(righthand[:, 0] - 0.9 * leftfoot[:, 0])
        # print("hand")
        # print(lefthand[:,0])
        # print('foot')
        # print(0.9*rightfoot[:,0])
        return 0.5 * torch.exp(-10 * left_distance) + 0.5 * torch.exp(-10 * right_distance)

    def _reward_torso_yaw(self):
        return torch.abs(self.actions[:, 12])

    def _reward_torso_orientation_diff_osu(self):
        # print(self.rigid_body_states[:,self.torso_indices])

        torso_projected_gravity = quat_rotate_inverse(self.rigid_body_states[:, self.torso_indices][:, 0, 3:7], self.gravity_vec)

        return 10.0 * torch.sum(torch.square(torso_projected_gravity[:, :2]), dim=1)

    def _reward_torso_ang_vel_xy_osu(self):
        torso_ang_vel_error = torch.sum(torch.square(self.rigid_body_states[:, self.torso_indices][:, 0, 10:12]), dim=-1)
        # print('anvel')
        # print(self.rigid_body_states[:,self.torso_indices][:,0,10:13])
        # print(torso_ang_vel_error)
        return torso_ang_vel_error

    def _reward_orientation_pitch_osu(self):
        pitch = get_euler_xyz(self.root_states[:, 3:7])[1]
        return torch.sin(0.5 * pitch)

    def _reward_reaching(self):
        erro_left = torch.sum(torch.square(self.rigid_body_states[:, self.hand_indices][:, 0, :3] - self.reaching_target[:, :3]), dim=1)
        erro_right = torch.sum(torch.square(self.rigid_body_states[:, self.hand_indices][:, 1, :3] - self.reaching_target[:, 3:6]), dim=1)
        return erro_left + erro_right
    # def _reward_feet_frcyaw(self):
    #     print("sensor")
    #     print(self.force_sensor_readings[:, 1, :])
    #     print(self.contact_forces[:, self.feet_indices[1], :])
    #     return 1
    # def _reward_termination_osu(self):

    #     return super()._reward_action_rate()

    # def _reward_hopsys_diff_osu(self):

    #     return super()._reward_action_rate()

    # def _reward_standing_cost_osu(self):

    #     return super()._reward_action_rate()
