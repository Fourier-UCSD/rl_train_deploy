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

class GR1L(LeggedRobot):      

# standard osu reward
#############################################################

    def _reward_feet_frc_osu(self):
        max_foot_frc = 350
        normed_left_frc = self.avg_feet_frc[:,0].clip(max=max_foot_frc)/max_foot_frc
        normed_right_frc = self.avg_feet_frc[:,1].clip(max=max_foot_frc)/max_foot_frc
        left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],self.cfg.commands.clock_transition_time)[0]
        right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],self.cfg.commands.clock_transition_time)[0]
        # left_frc_score = left_frc_clock*(1.0-torch.exp(- 10*torch.square(normed_left_frc)))
        # right_frc_score = right_frc_clock*(1.0-torch.exp(- 10*torch.square(normed_right_frc)))
        left_frc_score = left_frc_clock*(torch.exp(- 10*torch.square(self.avg_feet_frc[:,0])))
        right_frc_score = right_frc_clock*(torch.exp(- 10*torch.square(self.avg_feet_frc[:,1])))
        # left_frc_score = left_frc_clock*(1.0 - torch.exp(- 10.0*torch.abs(self.avg_feet_frc[:,0])))
        # right_frc_score = right_frc_clock*(1.0 - torch.exp(- 10.0*torch.abs(self.avg_feet_frc[:,1])))

        return left_frc_score + right_frc_score
    def _reward_feet_frc_osu_positive(self):
        left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],self.cfg.commands.clock_transition_time)[1]
        right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],self.cfg.commands.clock_transition_time)[1]
        # left_frc_score = left_frc_clock*(1.0-torch.exp(- 10*torch.square(normed_left_frc)))
        # right_frc_score = right_frc_clock*(1.0-torch.exp(- 10*torch.square(normed_right_frc)))
        # left_frc_score = left_frc_clock*(torch.exp(- 10*torch.square(self.avg_feet_frc[:,0])))
        # right_frc_score = right_frc_clock*(torch.exp(- 10*torch.square(self.avg_feet_frc[:,1])))
        # left_frc_score = left_frc_clock*(torch.exp(- 10.0*(torch.abs(self.avg_feet_frc[:,0] - 550.0))/100.0))
        # right_frc_score = right_frc_clock*(torch.exp(- 10.0*(torch.abs(self.avg_feet_frc[:,1]- 550.0))/100.0))
        # left_frc_score = left_frc_clock*(torch.exp(- 20.0*(-((self.avg_feet_frc[:,0] - 300.0).clip(max=0)))))
        # right_frc_score = right_frc_clock*(torch.exp(- 20.0*(-((self.avg_feet_frc[:,1] - 300.0).clip(max=0)))))
        left_frc_score = left_frc_clock*(torch.exp(- 5.0*(-((self.avg_feet_frc[:,0] - 10.0).clip(max=0)))))
        right_frc_score = right_frc_clock*(torch.exp(- 5.0*(-((self.avg_feet_frc[:,1] - 10.0).clip(max=0)))))
        return left_frc_score + right_frc_score        
    def _reward_feet_spd_osu(self):
        left_spd_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],self.cfg.commands.clock_transition_time)[1]
        right_spd_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],self.cfg.commands.clock_transition_time)[1]
        # left_spd_score = left_spd_clock*(torch.exp(- 2*torch.square(self.avg_feet_spd[:,0])))
        # right_spd_score = right_spd_clock*(torch.exp(- 2*torch.square(self.avg_feet_spd[:,1])))

        left_spd_score = left_spd_clock*(1.0 - torch.exp(- 10.0*torch.square(self.avg_feet_spd[:,0])))
        right_spd_score = right_spd_clock*(1.0 - torch.exp(- 10.0*torch.square(self.avg_feet_spd[:,1])))
        # print(left_foot_speed)
        # print("left_spd_clock")
        # print(left_spd_score + right_spd_score)
        return left_spd_score + right_spd_score

    def _reward_x_vel_diff_osu(self):
        # print("velerr")
        # print(torch.abs(self.commands[:, 0] - self.base_lin_vel[:, 0]))
        # print(torch.exp(-2*(torch.abs(self.commands[:, 0] - self.avg_x_vel))))
        # return torch.exp(-5*(torch.abs(self.commands[:, 0] - self.base_lin_vel[:, 0])))
        # print("x_vel_diff_osu: ")
        # print(1.0 - torch.exp(-20*(torch.square(self.commands[:, 0] - self.avg_x_vel))))
        return 1.0 - torch.exp(-50*(torch.square(self.commands[:, 0] - self.avg_x_vel)))
    def _reward_x_vel_diff_osu_positive(self):
        # print("velerr")
        # print(torch.abs(self.commands[:, 0] - self.base_lin_vel[:, 0]))
        # print(torch.exp(-2*(torch.abs(self.commands[:, 0] - self.avg_x_vel))))
        # return torch.exp(-5*(torch.abs(self.commands[:, 0] - self.base_lin_vel[:, 0])))
        # print("x_vel_diff_osu_positive: ")
        # print(torch.exp(-5.0*(torch.square(self.commands[:, 0] - self.avg_x_vel))))
        return torch.exp(-10.0*(torch.square(self.commands[:, 0] - self.avg_x_vel)))
    
    def _reward_y_vel_diff_osu(self):
        # print(1.0-torch.exp(-50*(torch.square(self.commands[:, 1] - self.avg_y_vel))))
        # return torch.exp(-5*(torch.abs(self.commands[:, 1] - self.base_lin_vel[:, 1])))
        # return 1.0 - torch.exp(-2*(torch.square(self.commands[:, 1] - self.avg_y_vel)))
        return 1.0 - torch.exp(-50*(torch.square(self.commands[:, 1] - self.avg_y_vel)))

    def _reward_z_vel_diff_osu(self):
        # print(1.0-torch.exp(-1*(torch.abs(self.base_lin_vel[:, 2]))))
        return 1.0 - torch.exp(-2*(torch.square(self.base_lin_vel[:, 2])))
    
    # 惩罚 yaw 速度差异
    def _reward_ang_vel_diff_osu(self):
        # ang_vel_error = torch.sum(torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2]),dim=-1)
        ang_vel_error = torch.abs(self.commands[:, 2] - self.avg_yaw_vel)
        # print(ang_vel_error)
        return 1.0 - torch.exp(-5.0 * ang_vel_error)
    
    # 惩罚 yaw 速度差异
    def _reward_ang_vel_diff_osu_positive(self):
        # ang_vel_error = torch.sum(torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2]),dim=-1)
        ang_vel_error = torch.abs(self.commands[:, 2] - self.avg_yaw_vel)
        # print(ang_vel_error)
        return torch.exp(-1.0 * ang_vel_error)
    
    def _reward_orientation_diff_osu(self):
        # print(self.projected_gravity[:, :2])
        # print(1.0-torch.exp(-10*torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)))
        return 1.0 - torch.exp(-10*torch.sum(torch.square(self.projected_gravity[:, :2]), dim=-1))

    def _reward_action_diff_osu(self):

        # print(1.0-torch.exp(-0.05*(torch.norm(self.last_actions - self.actions, dim=-1))))
        return 1.0 - torch.exp(-5.0*(torch.norm(self.last_actions - self.actions, dim=-1)))

    def _reward_torques_osu(self):
        # Penalize torques
        return 1.0 - torch.exp(-0.05*(torch.norm(self.torques,dim=-1)))
    def _reward_bias(self):
        return 1.0
# addition reward

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
        return 1.0 - torch.exp(-2.0 * torch.sum(out_of_limits, dim=1))
    def _reward_swing_landing_vel(self):
        s0 = self.gait_phase[:,0]/self.phase_ratio[:,0] - 0.5
        s0 = s0.clip(min=0., max=1.)
        s0 = s0*(s0 < 0.6)
        s0 = s0.clip(min=0., max=0.5)
        s1 = self.gait_phase[:,1]/self.phase_ratio[:,1] - 0.5
        s1 = s1.clip(min=0., max=1.)
        s1 = s1*(s1 < 0.6)
        s1 = s1.clip(min=0., max=0.5)

        leftfoot_v = self.rigid_body_states[:,self.feet_indices][:, 0, 9]
        rightfoot_v = self.rigid_body_states[:,self.feet_indices][:, 1, 9]
        # left_score = 1.0 - torch.exp(-2.0*s0*s0*leftfoot_v*leftfoot_v)
        # right_score = 1.0 - torch.exp(-2.0*s1*s1*rightfoot_v*rightfoot_v)
        
        left_score = 1.0 - torch.exp(-20.0*s0*s0*leftfoot_v*leftfoot_v)
        right_score = 1.0 - torch.exp(-20.0*s1*s1*rightfoot_v*rightfoot_v)
        return left_score + right_score
    
    def _reward_upperbody_action_diff_osu(self):
        return 1.0 - torch.exp(-0.05*(torch.norm(self.last_actions[:,12:] - self.actions[:,12:], dim=-1)))
    def _reward_feet_distance(self):
        leftfoot = self.rigid_body_states[:,self.feet_indices][:, 0, 0:3] - self.root_states[:,0:3]
        # leftfoot = quat_apply(quat_conjugate(self.root_states[:,3:7]),leftfoot)
        rightfoot = self.rigid_body_states[:,self.feet_indices][:, 1, 0:3] - self.root_states[:,0:3]
        # rightfoot = quat_apply(quat_conjugate(self.root_states[:,3:7]),rightfoot)
        # feet_distance_x = torch.abs(leftfoot[:,0]-rightfoot[:,0])
        feet_distance_y = torch.abs(leftfoot[:,1]-rightfoot[:,1]-0.2)
        # print(feet_distance_x)
        # return 0.5*torch.exp(-10*feet_distance_x) + 
        return 1.0 - torch.exp(-5.0*feet_distance_y)
    def _reward_arm_pose(self):
        # print(torch.sum(torch.abs(self.actions[:,[16,17,18,20,21,22]]), dim=1))
        # return torch.sum(torch.abs(self.actions[:,[16,17,18,20,21,22]]), dim=1)
        return 1.0 - torch.exp(-20.0*torch.sum(torch.abs(self.actions[:,[13,14,15,17,18,19]]), dim=-1))
    def _reward_arm_pose_shoulder_pitch(self):
        # print(torch.sum(torch.abs(self.actions[:,[16,17,18,20,21,22]]), dim=1))
        return 1.0 - torch.exp(-1.0*torch.sum(torch.abs(self.actions[:,[12,16]]), dim=-1) )
    def _reward_dof_vel_diff_osu(self):
        # Penalize dof velocities
        # print(1.0-torch.exp(-0.0001*torch.sum(torch.square(self.dof_vel), dim=1)))
        return 1.0 - torch.exp(-1.0*torch.sum(torch.square(self.dof_vel), dim=-1))
    def _reward_dof_acc_diff_osu(self):
        # Penalize dof accelerations
        # print(1.0-torch.exp(-0.0000001*torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1)))
        return 1.0 - torch.exp(-0.1*torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=-1))
    def _reward_feet_orien_diff_osu(self):
        left_foot_height = torch.mean(self.rigid_body_states[:,self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(self.rigid_body_states[:,self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        # print(1.0-torch.exp(-5.0*(torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1)*torch.abs(0.2-left_foot_height)+torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1)*torch.abs(0.2-right_foot_height))))
        return 1.0 - torch.exp(-5.0*(torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1)*torch.abs(0.2-left_foot_height)+torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1)*torch.abs(0.2-right_foot_height)))
    def _reward_ankleaction_diff_osu(self):
        # return 1.0 - torch.exp(-0.05*(torch.norm(self.last_actions[:,[4,5,10,11]] - self.actions[:,[4,5,10,11]], dim=-1))) 
        return 1.0 - torch.exp(-1.0*(torch.norm(self.last_actions[:,[4,5,10,11]] - self.actions[:,[4,5,10,11]], dim=-1)))     
    def _reward_hipyaw_osu(self):            
        left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],0.02)[0]
        right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],0.02)[0]
        left_yaw_stance = (self.gait_phase[:,0]-0.5)
        left_yaw_stance = left_yaw_stance.clip(min=0., max=1.)*self.cfg.commands.gait_cycle
        right_yaw_stance = (self.gait_phase[:,1]-0.5)
        right_yaw_stance = right_yaw_stance.clip(min=0., max=1.)*self.cfg.commands.gait_cycle
        left_yaw_score = left_frc_clock*(1.0 - torch.exp(-3.0*torch.abs(self.actions[:,1] - self.default_dof_pos[:,1]))) + (1-left_frc_clock)*(1.0 - torch.exp(-3.0*torch.abs(self.dof_pos[:,1] + left_yaw_stance*self.commands[:, 2])))
        right_yaw_score = right_frc_clock*(1.0 - torch.exp(-3.0*torch.abs(self.actions[:,7] - self.default_dof_pos[:,7]))) + (1-right_frc_clock)*(1.0 - torch.exp(-3.0*torch.abs(self.dof_pos[:,7] + right_yaw_stance*self.commands[:, 2])))
        return left_yaw_score + right_yaw_score
    def _reward_torso_orientation_diff_osu(self):
        # print(self.rigid_body_states[:,self.torso_indices])
        
        torso_projected_gravity = quat_rotate_inverse(self.rigid_body_states[:,self.torso_indices][:,0,3:7], self.gravity_vec)
        
        return 1.0 - torch.exp(-20.0*torch.sum(torch.square(torso_projected_gravity[:, :2]), dim=-1))
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
        # return 0.5 * torch.exp(-10 * left_distance) + 0.5 * torch.exp(-10 * right_distance)
        return 0.5 * torch.exp(-20 * left_distance) + 0.5 * torch.exp(-20 * right_distance)
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
    def _reward_torso_yaw(self):
        # return torch.abs(self.actions[:, 12])
        return 1.0 - torch.exp(-10.0*torch.abs(self.actions[:, 12]))
    def _reward_torso_ang_vel_xy_osu(self):
        torso_ang_vel_error = torch.sum(torch.square(self.rigid_body_states[:, self.torso_indices][:, 0, 10:12]), dim=-1)
        # print('anvel')
        # print(self.rigid_body_states[:,self.torso_indices][:,0,10:13])
        # print(torso_ang_vel_error)
        # return torso_ang_vel_error
        return 1.0 - torch.exp(-5.0*torso_ang_vel_error)
    def _reward_double_air(self):
        contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
        no_contact = torch.sum(1.*contacts, dim=1) == 0
        return 10.*no_contact
    
    # 惩罚 knee 扭矩
    def _reward_knee_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        # return torch.sum(torch.square(self.torques[:,[3,9]]), dim=1)
        return torch.sum(torch.square(self.torques[:, [3, 9]]), dim=1) \
            * torch.max(torch.exp(-15 * torch.abs(self.avg_x_vel)), torch.abs(self.avg_x_vel) < 0.2)
    def _reward_arm_pitch_keepposture(self):
        # print("arm_roll_keepposture:")
        # print(self.dof_pos[:,13])
        # print(self.dof_pos[:,17])
        # print(torch.exp(-5.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos))))
        # return torch.exp(-5.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos)))
        # return 1.0 - torch.exp(-20.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos)))
        # print("arm_roll_keepposture: ")
        # print(torch.square(torch.abs(self.dof_pos[:,13] - target_pos)+torch.abs(self.dof_pos[:,17] + target_pos)))
        return torch.square(torch.abs(self.dof_pos[:,12])+torch.abs(self.dof_pos[:,16]))
    def _reward_arm_roll_keepposture(self):
        target_pos = 0.2
        # print("arm_roll_keepposture:")
        # print(self.dof_pos[:,13])
        # print(self.dof_pos[:,17])
        # print(torch.exp(-5.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos))))
        # return torch.exp(-5.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos)))
        # return 1.0 - torch.exp(-20.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos)))
        # print("arm_roll_keepposture: ")
        # print(torch.square(torch.abs(self.dof_pos[:,13] - target_pos)+torch.abs(self.dof_pos[:,17] + target_pos)))
        return torch.square(torch.abs(self.dof_pos[:,13] - target_pos)+torch.abs(self.dof_pos[:,17] + target_pos))
    def _reward_lowarm_keepposture(self):
        target_pos = 0.0
        # print("arm_roll_keepposture:")
        # print(self.dof_pos[:,13])
        # print(self.dof_pos[:,17])
        # print(torch.exp(-5.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos))))
        # return torch.exp(-5.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos)))
        # return 1.0 - torch.exp(-20.0*(torch.abs(self.dof_pos[:,16] - target_pos) + torch.abs(self.dof_pos[:,20] + target_pos)))
        # print("arm_roll_keepposture: ")
        # print(torch.square(torch.abs(self.dof_pos[:,13] - target_pos)+torch.abs(self.dof_pos[:,17] + target_pos)))
        return torch.sum(torch.square(torch.abs(self.dof_pos[:,[14,15,18,19]] - target_pos)+torch.abs(self.dof_pos[:,[14,15,18,19]] + target_pos)), dim=1)
    def _reward_ankle_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        return torch.sum(torch.square(self.torques[:, [4, 10]]), dim=1)
    def _reward_arm_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        return torch.sum(torch.square(self.torques[:, 12:]), dim=1)
    def _reward_dof_vel_upperbody(self):
        # Penalize dof velocities
        # print(torch.sum(torch.square(self.dof_vel), dim=1))
        # print(torch.sum(torch.square(self.dof_vel[:,[0,6]]), dim=1))
        return torch.sum(torch.square(self.dof_vel[:,12:]), dim=1)

    def _reward_dof_acc_upperbody(self):
        # Penalize dof accelerations
        # print(torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1))
        return torch.sum(torch.square((self.last_dof_vel[:,12:] - self.dof_vel[:,12:]) / self.dt), dim=1)
    
    def _reward_ankle_torque_limits_penalty(self):
        # print("_reward_ankle_torque_limits_penalty: ")
        # # print(self.torque_limits.shape)
        # print(1.0 - torch.exp(-20.0 * torch.sum(
        #     (torch.abs(self.torques[:,[4,10]]) 
        #      - self.torque_limits[4] 
        #      * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1)))
        return 1.0 - torch.exp(-15.0 * torch.sum(
            (torch.abs(self.torques[:,[4,10]]) - self.torque_limits[4] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))
    def _reward_ankle_torque_limits_positive(self):
        # print("_reward_ankle_torque_limits_positive: ")
        # print(torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,[4,10]]) - self.torque_limits[4] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1)))
        return torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,[4,10]]) - self.torque_limits[4] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1))
    
    def _reward_ankle_action_diff_osu_penalty(self):
        return 1.0 - torch.exp(
            -0.5 * (torch.norm(self.last_actions[:, [4, 5, 10, 11]] - self.actions[:, [4, 5, 10, 11]], dim=-1)))
    def _reward_ankle_action_diff_osu_positive(self):
        return torch.exp(
            -0.1 * (torch.norm(self.last_actions[:, [4, 5, 10, 11]] - self.actions[:, [4, 5, 10, 11]], dim=-1)))
    
    def _reward_arm_torque_limits_penalty(self):
        # print("_reward_ankle_torque_limits_penalty: ")
        # # # print(self.torque_limits.shape)
        # print((torch.abs(self.torques[:,12:]) - 18.0* self.cfg.rewards.soft_torque_limit).clip(min=0.))
        return 1.0 - torch.exp(-20.0 * torch.sum(
            (torch.abs(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))
    def _reward_arm_torque_limits_positive(self):
        # print("_reward_arm_torque_limits_positive: ")
        # print(torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1)))
        # print("arm torque: ")
        # print((torch.abs(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.))
        return torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1))
    def _reward_ankle_roll_actions_penalty(self):
        # print("_reward_arm_torque_limits_positive: ")
        # print(torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1)))
        # print("arm torque: ")
        # print((torch.abs(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.))
        # print(torch.sum(torch.square(self.actions[:, [5, 11]]), dim=-1))
        return torch.sum(torch.square(self.actions[:, [5, 11]]), dim=-1)
    def _reward_ankle_roll_actions_positive(self):
        # print("_reward_arm_torque_limits_positive: ")
        # print(torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1)))
        # print("arm torque: ")
        # print((torch.abs(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.))
        return torch.exp(-1.0 * torch.sum(torch.square(self.actions[:, [5, 11]]), dim=-1))
    def _reward_ankle_roll_torque_limits_penalty(self):
        # print("_reward_ankle_torque_limits_penalty: ")
        # # print(self.torque_limits.shape)
        # print(1.0 - torch.exp(-20.0 * torch.sum(
        #     (torch.abs(self.torques[:,[4,10]]) 
        #      - self.torque_limits[4] 
        #      * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1)))
        return 1.0 - torch.exp(-50.0 * torch.sum(
            (torch.abs(self.torques[:,[5,11]]) - self.torque_limits[5] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))
    def _reward_ankle_roll_torque_limits_positive(self):
        # print("_reward_ankle_torque_limits_positive: ")
        # print(torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,[4,10]]) - self.torque_limits[4] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1)))
        return torch.exp(-10.0 * torch.sum((torch.abs(self.torques[:,[5,11]]) - self.torque_limits[5] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1))
    def _reward_ankle_roll_torques(self):
        # Penalize torques
        # print(torch.sum(torch.square(self.torques), dim=1))
        return torch.sum(torch.square(self.torques[:, [5, 11]]), dim=1)
