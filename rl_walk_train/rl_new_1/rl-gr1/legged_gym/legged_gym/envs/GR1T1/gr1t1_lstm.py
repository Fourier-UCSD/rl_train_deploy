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

class GR1T1(LeggedRobot):      

# standard osu reward
#############################################################

    def _reward_feet_frc_osu(self):
        max_foot_frc = 350
        left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],self.cfg.commands.clock_transition_time)[0]
        right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],self.cfg.commands.clock_transition_time)[0]
        left_frc_score = left_frc_clock * (torch.exp(- 10 * torch.square(self.avg_feet_frc[:, 0])))
        right_frc_score = right_frc_clock * (torch.exp(- 10 * torch.square(self.avg_feet_frc[:, 1])))
        return left_frc_score + right_frc_score
    
    def _reward_feet_frc_stand_osu(self):
        max_foot_frc = 350

        left_frc_score = (torch.exp(- 10 * torch.square(self.avg_feet_frc[:, 0] -350)))
        right_frc_score = (torch.exp(- 10 * torch.square(self.avg_feet_frc[:, 1] -350)))
        return left_frc_score + right_frc_score

    def _reward_feet_frc_stand_left_osu(self):
        max_foot_frc = 350
        # print("self.avg_feet_frc[:,0]: ", self.avg_feet_frc[:,0])
        left_frc_score = 1- (torch.exp(- 10 * torch.square(self.avg_feet_frc[:,0] )))
        return left_frc_score 
    
    
    def _reward_feet_frc_stand_right_osu(self):
        max_foot_frc = 350

        right_frc_score = 1- (torch.exp(- 10 * torch.square(self.avg_feet_frc[:,1] )))
        return right_frc_score 
    
    
    
    def _reward_feet_spd_stand_left_osu(self):
        max_foot_frc = 350

        left_frc_score =  (torch.exp(- 4 * torch.square(self.avg_feet_spd[:, 0])))
        return left_frc_score 
    
    
    def _reward_feet_spd_stand_right_osu(self):
        max_foot_frc = 350

        right_frc_score =  (torch.exp(- 4 * torch.square(self.avg_feet_spd[:, 1])))
        return right_frc_score 
    
    def _reward_feet_spd_stand_osu(self):

        left_spd_score =  (torch.exp(- 4 * torch.square(self.avg_feet_spd[:, 0])))
        right_spd_score =  (torch.exp(- 4 * torch.square(self.avg_feet_spd[:, 1])))
        return left_spd_score + right_spd_score

    def _reward_feet_spd_osu(self):
        left_spd_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],self.cfg.commands.clock_transition_time)[1]
        right_spd_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],self.cfg.commands.clock_transition_time)[1]
        left_spd_score = left_spd_clock * (torch.exp(- 4 * torch.square(self.avg_feet_spd[:, 0])))
        right_spd_score = right_spd_clock * (torch.exp(- 4 * torch.square(self.avg_feet_spd[:, 1])))
        return left_spd_score + right_spd_score

    def _reward_feet_frc_osu_positive(self):
        left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],self.cfg.commands.clock_transition_time)[1]
        right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],self.cfg.commands.clock_transition_time)[1]
        left_frc_score = left_frc_clock*(torch.exp(- 5.0*(-((self.avg_feet_frc[:,0] - 10.0).clip(max=0)))))
        right_frc_score = right_frc_clock*(torch.exp(- 5.0*(-((self.avg_feet_frc[:,1] - 10.0).clip(max=0)))))
        return left_frc_score + right_frc_score    
# base
    def _reward_x_vel_diff_osu(self):
        # return 1.0 - torch.exp(-100*(torch.square(self.commands[:, 0] - self.base_lin_vel[:,0])))   
        return torch.sum(torch.square(self.commands[:, 0] - self.avg_x_vel))
    def _reward_x_vel_diff_osu_positive(self):
        # return torch.exp(-5.0*(torch.square(self.commands[:, 0] - self.avg_x_vel)))
        # return torch.exp(-10.0*(torch.square(self.commands[:, 0] - self.avg_x_vel)))
        # return torch.exp(-20.0*(torch.square(self.commands[:, 0] - self.avg_x_vel)))
        return torch.exp(-20.0*(torch.square(self.commands[:, 0] - self.avg_x_vel)))
    
    def _reward_y_vel_diff_osu(self):
        return torch.sum(torch.square(self.commands[:, 1] - self.base_lin_vel[:,1]))
        # return 1.0 - torch.exp(-50*(torch.square(self.commands[:, 1] - self.base_lin_vel[:,1])))
    def _reward_y_vel_diff_osu_positive(self):
        # return torch.exp(-5.0*(torch.square(self.commands[:, 1] - self.avg_y_vel)))
        return torch.exp(-15.0*(torch.square(self.commands[:, 1] - self.avg_y_vel)))
    
    def _reward_z_vel_diff_osu(self):
        return 1.0 - torch.exp(-15*(torch.square(self.base_lin_vel[:, 2])))
    def _reward_z_vel_diff_osu_positive(self):
        # return torch.exp(-2.0*(torch.square(self.base_lin_vel[:, 2])))   
        return torch.exp(-20.0*(torch.square(self.base_lin_vel[:, 2])))  
     
    def _reward_ang_vel_diff_osu(self):
        ang_vel_error = torch.abs(self.commands[:, 2] - self.avg_yaw_vel)
        return 1.0 - torch.exp(-5.0 * ang_vel_error)
    def _reward_ang_vel_diff_osu_positive(self):
        ang_vel_error = torch.abs(self.commands[:, 2] - self.avg_yaw_vel)
        return torch.exp(-1.0 * ang_vel_error)

    def _reward_ang_vel_xy_osu_positive(self):
        ang_vel_error = torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=-1)
        # return torch.exp(-1.0 * ang_vel_error)
        return torch.exp(-5.0 * ang_vel_error)

    def _reward_torso_orientation_diff_osu(self):        
        torso_projected_gravity = quat_rotate_inverse(self.rigid_body_states[:,self.torso_indices][:,0,3:7], self.gravity_vec)        
        return 1.0 - torch.exp(-20.0*torch.sum(torch.square(torso_projected_gravity[:, :2]), dim=-1))
    def _reward_torso_orientation_diff_osu_positive(self):        
        torso_projected_gravity = quat_rotate_inverse(self.rigid_body_states[:,self.torso_indices][:,0,3:7], self.gravity_vec)        
        return torch.exp(-20.0*torch.sum(torch.square(torso_projected_gravity[:, :2]), dim=-1))
    def _reward_orientation_diff_osu(self):
        # return 1.0 - torch.exp(-20.0*(torch.square(self.projected_gravity[:, 1]))-10.0*(torch.square(self.projected_gravity[:, 0])))
        return 1.0 - torch.exp(-20.0*(torch.square(self.projected_gravity[:, 1]))-10.0*(torch.square(self.projected_gravity[:, 0])))
    def _reward_orientation_diff_osu_positive(self):
        # return torch.exp(-10 * torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1))
        # return torch.exp(-20.0 * torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1))
        return torch.exp(-10.0*(torch.square(self.projected_gravity[:, 1]))-20.0*(torch.square(self.projected_gravity[:, 0])))
    
    def _reward_torso_yaw(self):
        return 1.0 - torch.exp(-10.0*torch.abs(self.actions[:, 12]))
    def _reward_torso_yaw_positive(self):
        return torch.exp(-10.0*torch.abs(self.actions[:, 12]))
    def _reward_torso_ang_vel_xy_osu(self):
        torso_ang_vel_error = torch.sum(torch.square(self.rigid_body_states[:, self.torso_indices][:, 0, 10:12]), dim=-1)
        return 1.0 - torch.exp(-5.0*torso_ang_vel_error)
    def _reward_torso_ang_vel_xy_osu_positive(self):
        torso_ang_vel_error = torch.sum(torch.square(self.rigid_body_states[:, self.torso_indices][:, 0, 10:12]), dim=-1)
        return torch.exp(-5.0*torso_ang_vel_error)
# joint torque
    def _reward_torques_osu(self):
        return 1.0 - torch.exp(-0.05*(torch.norm(self.torques,dim=-1)))
    
    def _reward_torques_diff_osu(self):
        return 1.0 - torch.exp(-5.0*(torch.sum(torch.square(self.last_torques - self.torques),dim=-1)))
    def _reward_torques_diff_osu_strong(self):
        return torch.sum(torch.square(self.last_torques - self.torques),dim=-1)
    
    def _reward_ankle_torques_osu_positive(self):
        # return torch.exp(-0.5*(torch.norm(self.torques[:,[4,5,10,11]],dim=-1)))
        return torch.exp(-0.1*(torch.norm(self.torques[:,[4,5,10,11]],dim=-1)))
    
    def _reward_arm_torques_osu(self):
        return 1.0 - torch.exp(-0.5*(torch.norm(self.torques[:,[12, 13, 14, 15, 16, 17, 18, 19]],dim=-1)))
    
    def _reward_torque_limits_osu(self):     
        return 1.0 - torch.exp(-5.0*torch.sum(
            (torch.abs(self.torques) - self.torque_limits * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))
    def _reward_torque_limits_osu_positive(self):
        return torch.exp(-0.1*torch.sum(
            (torch.abs(self.torques) - self.torque_limits * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))
    
    def _reward_knee_torques(self):
        return torch.sum(torch.square(self.torques[:, [3, 9]]), dim=1) \
            * torch.max(torch.exp(-15 * torch.abs(self.avg_x_vel)), torch.abs(self.avg_x_vel) < 0.2)
    def _reward_knee_torques_shape(self):
        left_spd_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],self.cfg.commands.clock_transition_time)[1]
        right_spd_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],self.cfg.commands.clock_transition_time)[1]
        return (torch.square(self.torques[:, 3]*left_spd_clock) + torch.square(self.torques[:, 9]*right_spd_clock)) \
            * torch.max(torch.exp(-15 * torch.abs(self.avg_x_vel)), torch.abs(self.avg_x_vel) < 0.5)   
    def _reward_knee_pos(self):
        return torch.sum(torch.square(self.dof_pos[:, [3,9]] -0.2), dim=1)
    
    def _reward_ankle_torques(self):
        return torch.sum(torch.square(self.torques[:, [4, 10, 5, 11]]), dim=1)
    def _reward_ankle_roll_torque_limits_penalty(self):
        return 1.0 - torch.exp(-15.0 * torch.sum(
            (torch.abs(self.torques[:,[5,11]]) - self.torque_limits[5] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))
    def _reward_ankle_roll_torque_limits_positive(self):
        return torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,[5,11]]) - self.torque_limits[5] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1))
    def _reward_ankle_torque_limits_penalty(self):
        return 1.0 - torch.exp(-15.0 * torch.sum(
            (torch.abs(self.torques[:,[4,10]]) - self.torque_limits[4] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))
    def _reward_ankle_torque_limits_positive(self):
        return torch.exp(-5.0 * torch.sum((torch.abs(self.torques[:,[4,10]]) - self.torque_limits[4] * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1))
    def _reward_ankle_roll_torques(self):
        return torch.sum(torch.square(self.torques[:, [5, 11]]), dim=1)
    
    def _reward_arm_torques(self):
        return torch.sum(torch.square(self.torques[:, 12:]), dim=1)
    def _reward_arm_torque_limits_penalty(self):
        return 1.0 - torch.exp(-20.0 * torch.sum(
            (torch.square(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1))
    def _reward_arm_torque_limits_positive(self):
        # return torch.exp(-0.01 * torch.sum((torch.square(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1))
        # return torch.exp(-0.005 * torch.sum((torch.square(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1))
        return torch.exp(-0.001 * torch.sum((torch.square(self.torques[:,12:]) - 18.0 * self.cfg.rewards.soft_torque_limit).clip(min=0.), dim=-1))
# bias
    def _reward_bias(self):
        return 1.0

# dof pos vel acc

    def _reward_dof_pos_limits_osu(self):
        out_of_limits = -(self.dof_pos[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 ]] -
                          self.dof_pos_limits[:, 0][[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 ]]).clip(
            max=0.)  # lower limit
        out_of_limits += (self.dof_pos[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 ]] -
                          self.dof_pos_limits[:, 1][[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 ]]).clip(
            min=0.)
        return 1.0 - torch.exp(-20.0 * torch.sum(torch.square(out_of_limits), dim=1))
    def _reward_dof_pos_limits_osu_positive(self):
        out_of_limits = -(self.dof_pos[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 ]] -
                          self.dof_pos_limits[:, 0][[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 ]]).clip(
            max=0.)  # lower limit
        out_of_limits += (self.dof_pos[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 ]] -
                          self.dof_pos_limits[:, 1][[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 ]]).clip(
            min=0.)
        # print("out_of_limits")
        # print(out_of_limits)
        # return torch.exp(-1.0 * torch.sum(out_of_limits, dim=1)) 
        # return torch.exp(-0.5 * torch.sum(out_of_limits, dim=1))  
        return torch.exp(-5.0 * torch.sum(out_of_limits, dim=1))    
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
        return 0.5 * torch.exp(-10 * left_distance) + 0.5 * torch.exp(-10 * right_distance)
    def _reward_arm_keepposture_positive(self):
        return torch.exp(-1.0 * torch.sum((torch.square(self.dof_pos[:,[16,17,18,20,21,22]] - self.default_dof_pos[:,[16,17,18,20,21,22]])), dim=-1))
    def _reward_arm_keepposture_penalty(self):
        return 1.0-torch.exp(-10.0 * torch.sum((torch.square(self.dof_pos[:,[16,17,18,20,21,22]] - self.default_dof_pos[:,[16,17,18,20,21,22]])), dim=-1))
    
    def _reward_dof_vel_diff_osu(self):
        return 1.0 - torch.exp(-0.01*torch.sum(torch.square(self.dof_vel), dim=-1))
    def _reward_dof_acc_diff_osu(self):
        return 1.0 - torch.exp(-0.00001*torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=-1))
    def _reward_dof_vel_diff_osu_positive(self):
        return torch.exp(-0.001*torch.sum(torch.square(self.dof_vel), dim=-1))
    def _reward_dof_acc_diff_osu_positive(self):
        # return torch.exp(-0.000001*torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=-1))
        return torch.exp(-0.0000001*torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=-1))

    def _reward_dof_vel_upperbody(self):
        return torch.sum(torch.square(self.dof_vel[:,12:]), dim=1)
    def _reward_dof_vel_upperbody_osu(self):
        return 1.0 - torch.exp(-0.005*torch.sum(torch.square(self.dof_vel[:,12:]), dim=1))
    def _reward_dof_vel_upperbody_osu_positive(self):
        return torch.exp(-0.001*torch.sum(torch.square(self.dof_vel[:,12:]), dim=1))
    
    def _reward_dof_acc_upperbody(self):
        return torch.sum(torch.square((self.last_dof_vel[:,12:] - self.dof_vel[:,12:]) / self.dt), dim=1)
    def _reward_dof_acc_upperbody_osu(self):
        return 1.0 - torch.exp(-0.00001*torch.sum(torch.square((self.last_dof_vel[:,12:] - self.dof_vel[:,12:]) / self.dt), dim=1))
    def _reward_dof_acc_upperbody_osu_positive(self):
        return torch.exp(-0.0000001*torch.sum(torch.square((self.last_dof_vel[:,12:] - self.dof_vel[:,12:]) / self.dt), dim=1))

    def _reward_dof_vel(self):
        return torch.sum(torch.square(self.dof_vel), dim=1)
    def _reward_dof_acc(self):
        return torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1) 
    def _reward_dof_acc_hippitch(self):
        return torch.sum(torch.square((self.last_dof_vel[:,[2,8]] - self.dof_vel[:,[2,8]]) / self.dt), dim=1)
    
    def _reward_orientation_pitch_osu(self):
        return  torch.exp(-10 * torch.sum(torch.abs(self.projected_gravity[:, 0])))
    
    def _reward_hipyaw_osu(self):            
        left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],0.02)[0]
        right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],0.02)[0]
        left_yaw_stance = 0.5*self.cfg.commands.gait_cycle
        right_yaw_stance = 0.5*self.cfg.commands.gait_cycle
        left_yaw_score = left_frc_clock*(1.0 - torch.exp(-3.0*torch.abs(self.dof_pos[:,1] - self.default_dof_pos[:,1]))) + (1-left_frc_clock)*(1.0 - torch.exp(-3.0*torch.abs(self.dof_pos[:,1] + left_yaw_stance*self.commands[:, 2])))
        right_yaw_score = right_frc_clock*(1.0 - torch.exp(-3.0*torch.abs(self.dof_pos[:,7] - self.default_dof_pos[:,7]))) + (1-right_frc_clock)*(1.0 - torch.exp(-3.0*torch.abs(self.dof_pos[:,7] + right_yaw_stance*self.commands[:, 2])))
        return left_yaw_score + right_yaw_score
    def _reward_hipyaw_osu_positive(self):            
        left_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,0],self.phase_ratio[:,0],0.02)[0]
        right_frc_clock = gait_clock(self.num_envs,self.sim_device,self.gait_phase[:,1],self.phase_ratio[:,1],0.02)[0]
        # left_yaw_stance = 0.5*self.cfg.commands.gait_cycle
        # right_yaw_stance = 0.5*self.cfg.commands.gait_cycle
        left_yaw_stance = (self.gait_phase[:, 0] - 0.5)
        left_yaw_stance = left_yaw_stance.clip(min=0., max=1.) * self.cfg.commands.gait_cycle
        right_yaw_stance = (self.gait_phase[:, 1] - 0.5)
        right_yaw_stance = right_yaw_stance.clip(min=0., max=1.) * self.cfg.commands.gait_cycle
        left_yaw_score = left_frc_clock*(torch.exp(-3.0*torch.abs(self.actions_filter[:,1] - self.default_dof_pos[:,1]))) + (1-left_frc_clock)*(torch.exp(-3.0*torch.abs(self.dof_pos[:,1] + left_yaw_stance*self.commands[:, 2])))
        right_yaw_score = right_frc_clock*(torch.exp(-3.0*torch.abs(self.actions_filter[:,7] - self.default_dof_pos[:,7]))) + (1-right_frc_clock)*(torch.exp(-3.0*torch.abs(self.dof_pos[:,7] + right_yaw_stance*self.commands[:, 2])))
        return left_yaw_score + right_yaw_score  
    
    
    def _reward_hipyaw_osu_stand_positive(self):            
        left_yaw_score = (torch.exp(-3.0*torch.abs(self.dof_pos[:,1] ))) 
        right_yaw_score = (torch.exp(-3.0*torch.abs(self.dof_pos[:,7] ))) 
        return left_yaw_score + right_yaw_score    
    
    def _reward_hiproll_osu_stand_positive(self):            
        left_yaw_score = (torch.exp(-3.0*torch.abs(self.dof_pos[:,0] ))) 
        right_yaw_score = (torch.exp(-3.0*torch.abs(self.dof_pos[:,6] ))) 
        return left_yaw_score + right_yaw_score 
# action 
    def _reward_upperbody_action_diff_osu(self):
        return torch.sum(torch.square(self.last_actions[:,12:] - self.actions[:,12:]), dim=-1)
    def _reward_upperbody_action_diff_osu_positive(self):
        # return torch.exp(-1.0*(torch.sum(torch.square(self.last_actions[:,12:] - self.actions[:,12:]), dim=-1)))
        return torch.exp(-5*(torch.sum(torch.square(self.last_actions[:,12:] - self.actions[:,12:]), dim=-1)))
    def _reward_upperbody_pos_osu_positive(self):
        # return torch.exp(-1.0*(torch.sum(torch.square(self.last_actions[:,12:] - self.actions[:,12:]), dim=-1)))
        # return torch.exp(-5*(torch.sum(torch.square(self.dof_pos[:,[12,13,14,15, 16, 17, 18, 19, 20, 21, 22 ]] - self.default_dof_pos[:,[12,13,14,15, 16, 17, 18, 19, 20, 21, 22 ]]), dim=-1)))
        return torch.exp(-5*(torch.sum(torch.square(self.dof_pos[:,[16]] - self.default_dof_pos[:,[16]]), dim=-1)))
    
    def _reward_upperbody_pos_osu(self):
        # return torch.exp(-1.0*(torch.sum(torch.square(self.last_actions[:,12:] - self.actions[:,12:]), dim=-1)))
        return torch.sum(torch.square(self.dof_pos[:,[12,13,14,15, 16, 17, 18, 19, 20, 21, 22 ]]- self.default_dof_pos[:,[12,13,14,15, 16, 17, 18, 19, 20, 21, 22 ]]), dim=-1)
    
    def _reward_upperbody_actions_osu(self):
        # return torch.exp(-1.0*(torch.sum(torch.square(self.last_actions[:,12:] - self.actions[:,12:]), dim=-1)))
        return torch.sum(torch.square(self.actions[:,[12,13,14]]), dim=-1)
    
    def _reward_action_diff_osu(self):
        return torch.sum(torch.square(self.last_actions - self.actions), dim=-1)
    def _reward_action_diff_osu_positive(self):
        return torch.exp(-0.05 * (torch.norm(self.last_actions - self.actions, dim=-1)))
        
    def _reward_ankleaction_diff_osu(self): 
        return 1.0 - torch.exp(-10.0*(torch.norm(self.last_actions[:,[4,5,10,11]] - self.actions[:,[4,5,10,11]], dim=-1)))  
   
    def _reward_ankle_action_diff_osu_penalty(self):
        return torch.sum(torch.square(self.last_actions[:, [4, 5, 10, 11]] - self.actions[:, [4, 5, 10, 11]]), dim=-1)
    def _reward_ankle_action_diff_osu_positive(self):
        # return torch.exp(
        #     -0.05 * (torch.norm(self.last_actions[:, [4, 5, 10, 11]] - self.actions[:, [4, 5, 10, 11]], dim=-1)))
        return torch.exp(
            -20.0 * (torch.sum(torch.square(self.last_actions[:, [4, 5, 10, 11]] - self.actions[:, [4, 5, 10, 11]]), dim=-1)))
    
    def _reward_ankle_roll_actions_penalty(self):
        return torch.sum(torch.square(self.actions[:, [5, 11]]), dim=-1)
    def _reward_ankle_roll_actions_positive(self):
        return torch.exp(-10.0 * torch.sum(torch.square(self.actions[:, [5, 11]]), dim=-1))
# feet distance and orient    
    def _reward_feet_distance_new(self):
        leftfoot = self.rigid_body_states[:,self.feet_indices][:, 0, 0:3] - self.root_states[:,0:3]
        leftfoot_local = quat_rotate_inverse(self.base_quat, leftfoot)
        rightfoot = self.rigid_body_states[:,self.feet_indices][:, 1, 0:3] - self.root_states[:,0:3]
        rightfoot_local = quat_rotate_inverse(self.base_quat, rightfoot)
        feet_distance_x = torch.abs(leftfoot_local[:,0]-rightfoot_local[:,0])*(torch.abs(self.commands[:,0]) < 0.1)
        feet_distance_y = (0.2 - (leftfoot_local[:,1]-rightfoot_local[:,1])).clip(min= 0.)*(torch.abs(self.commands[:,1]<0.1))
        return 1.0 - torch.exp(-10.0*feet_distance_y - 20.0*feet_distance_x)
    def _reward_feet_distance_new_positive(self):
        leftfoot = self.rigid_body_states[:,self.feet_indices][:, 0, 0:3] - self.root_states[:,0:3]
        leftfoot_local = quat_rotate_inverse(self.base_quat, leftfoot)
        rightfoot = self.rigid_body_states[:,self.feet_indices][:, 1, 0:3] - self.root_states[:,0:3]
        rightfoot_local = quat_rotate_inverse(self.base_quat, rightfoot)
        feet_distance_x = torch.abs(leftfoot_local[:,0]-rightfoot_local[:,0])*(torch.abs(self.commands[:,0])<0.1)
        feet_distance_y = torch.abs(0.22 - (leftfoot_local[:,1]-rightfoot_local[:,1]))
        # print("feet_distance_y")
        # print((leftfoot_local[:,1]-rightfoot_local[:,1]))
        # print("feet_distance_x")
        # print((leftfoot_local[:,0]-rightfoot_local[:,0]))
        # return 0.5 * torch.exp(-10 * feet_distance_x) + 0.5 * torch.exp(-10 * feet_distance_y)
        return torch.exp(-20 * feet_distance_x - 50.0 * feet_distance_y)
    
    def _reward_feet_orien_diff_osu(self):
        left_foot_height = torch.mean(self.rigid_body_states[:,self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(self.rigid_body_states[:,self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        return 1.0 - torch.exp(-5.0*(torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1)*torch.abs((0.2-left_foot_height).clip(min=0.))+torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1)*torch.abs((0.2-right_foot_height).clip(min=0.))))
    def _reward_feet_orien_diff_osu_positive(self):
        left_foot_height = torch.mean(self.rigid_body_states[:,self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights, dim=1)
        right_foot_height = torch.mean(self.rigid_body_states[:,self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights, dim=1)
        return torch.exp(-5.0 * (torch.sum(torch.square(self.left_foot_orien_projected[:, :2]), dim=1) * torch.abs(
                0.2 - left_foot_height)
                + torch.sum(torch.square(self.right_foot_orien_projected[:, :2]), dim=1) * torch.abs(
                0.2 - right_foot_height)))
    def _reward_swing_symmetric(self):
        left_spd_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 0], self.phase_ratio[:, 0], 0.02)[1].unsqueeze(1)
        right_spd_clock = gait_clock(self.num_envs, self.sim_device, self.gait_phase[:, 1], self.phase_ratio[:, 1], 0.02)[1].unsqueeze(1)
        feet_distance = self.rigid_body_states[:, self.feet_indices][:, 0, 0:3] - self.rigid_body_states[:, self.feet_indices][:, 1, 0:3]
        feet_distance = quat_apply(quat_conjugate(self.root_states[:, 3:7]), feet_distance)
        db_flag = (left_spd_clock > 0.5) * (right_spd_clock > 0.5)
        score = torch.abs(feet_distance[:, 0] + self.last_feet_distance[:, 0]).unsqueeze(1)
        self.feet_distance = db_flag * (feet_distance) + (~db_flag) * self.feet_distance
        self.last_feet_distance = (~db_flag) * (self.feet_distance) + db_flag * self.last_feet_distance
        return (3.3 * db_flag * torch.exp(-10 * score)).squeeze(1)
    
    
    # def _reward_swing_height(self):
    #     s0 = self.gait_phase[:, 0] / self.phase_ratio[:, 0]
    #     # s0 = s0.clip(min=0., max=1.)
    #     s0 = s0 * (s0 < 0.5)
    #     s1 = self.gait_phase[:, 1] / self.phase_ratio[:, 1]
    #     # s1 = s1.clip(min=0., max=1.)
    #     s1 = s1 * (s1 < 0.5)
    #     leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 2]
    #     rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 2]
    #     # print(leftfoot-rightfoot)
    #     # print(s0*(leftfoot - rightfoot - 0.1) + s1*(rightfoot - leftfoot - 0.1))
    #     left_score = 2 * s0 * (torch.exp(-25. * torch.abs(leftfoot - rightfoot - 0.04)))
    #     right_score = 2 * s1 * (torch.exp(-25. * torch.abs(rightfoot - leftfoot - 0.04)))
    #     # print(left_score + right_score)
    #     return left_score + right_score
    
    
    def _reward_swing_height_positive(self):
        s0 = torch.where(self.phase_ratio[:, 0] != 0, self.gait_phase[:, 0] / self.phase_ratio[:, 0], 1.0)
        s0 = s0.clip(min=0., max=1.)
        s0 = s0 * (s0 < 0.5)
        s1 = torch.where(self.phase_ratio[:, 1] != 0, self.gait_phase[:, 1] / self.phase_ratio[:, 1], 1.0)
        s1 = s1.clip(min=0., max=1.)
        s1 = s1 * (s1 < 0.5)
        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 2]
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 2]
        left_score = 2 * s0 * (torch.exp(-25. * torch.abs(leftfoot - rightfoot - 0.04)))
        right_score = 2 * s1 * (torch.exp(-25. * torch.abs(rightfoot - leftfoot - 0.04)))
        return left_score +  right_score
    
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

    def _reward_base_height_diff_stand_osu(self):
        # Penalize base height away from target
        # leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 2]
        # rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 2]
        # foot_height = left_spd_clock * leftfoot + right_spd_clock * rightfoot 
        base_height = self.root_states[:, 2] 
        # print(base_height)
        # print(torch.abs(base_height - self.cfg.rewards.base_height_target))
        # print(1.0-torch.exp(-12*torch.norm(base_height - self.cfg.rewards.base_height_target,dim=-1)))
        # return torch.exp(-12*torch.norm(base_height - self.cfg.rewards.base_height_target,dim=-1))
        return torch.exp(-2 * torch.abs(base_height - self.cfg.rewards.base_height_target))
    
    def _reward_swing_height_penalty(self):
        s0 = torch.where(self.phase_ratio[:, 0] != 0, self.gait_phase[:, 0] / self.phase_ratio[:, 0], 1.0)
        s0 = s0.clip(min=0., max=1.)
        s0 = s0 * (s0 < 0.5)
        s1 = torch.where(self.phase_ratio[:, 1] != 0, self.gait_phase[:, 1] / self.phase_ratio[:, 1], 1.0)
        s1 = s1.clip(min=0., max=1.)
        s1 = s1 * (s1 < 0.5)

        leftfoot = self.rigid_body_states[:, self.feet_indices][:, 0, 2]
        rightfoot = self.rigid_body_states[:, self.feet_indices][:, 1, 2]
        target_h = self.cfg.rewards.feetheight_target
        left_height =  torch.square(leftfoot - rightfoot - target_h)/(target_h*target_h)
        right_height = torch.square(rightfoot - leftfoot - target_h)/(target_h*target_h)
        s0 = s0/0.5
        s1 = s1/0.5
        left_score = 1.0 - torch.exp(-25.0*s0*s0*left_height)
        right_score =1.0 - torch.exp(-25.0*s1*s1*right_height)
        return left_score +  right_score
    def _reward_swing_landing_vel(self):
        s0 = self.gait_phase[:,0]/self.phase_ratio[:,0] - 0.7
        s0 = s0.clip(min=0., max=1.)
        s0 = s0*(s0 < 0.6)
        s0 = s0.clip(min=0., max=0.5)
        s1 = self.gait_phase[:,1]/self.phase_ratio[:,1] - 0.7
        s1 = s1.clip(min=0., max=1.)
        s1 = s1*(s1 < 0.6)
        s1 = s1.clip(min=0., max=0.5)
        leftfoot_v = self.rigid_body_states[:,self.feet_indices][:, 0, 9]
        rightfoot_v = self.rigid_body_states[:,self.feet_indices][:, 1, 9]        
        left_score = 1.0 - torch.exp(-100.0*s0*s0*s0*leftfoot_v*leftfoot_v)
        right_score = 1.0 - torch.exp(-100.0*s1*s1*s1*rightfoot_v*rightfoot_v)
        return left_score + right_score
    
    def _reward_swing_landing_vel_positive(self):
        s0 = self.gait_phase[:,0]/self.phase_ratio[:,0] - 0.7
        s0 = s0.clip(min=0., max=1.)
        s0 = s0*(s0 < 0.6)
        s0 = s0.clip(min=0., max=0.5)
        s1 = self.gait_phase[:,1]/self.phase_ratio[:,1] - 0.7
        s1 = s1.clip(min=0., max=1.)
        s1 = s1*(s1 < 0.6)
        s1 = s1.clip(min=0., max=0.5)
        leftfoot_v = self.rigid_body_states[:,self.feet_indices][:, 0, 9]
        rightfoot_v = self.rigid_body_states[:,self.feet_indices][:, 1, 9]        
        left_score = torch.exp(-10.0*s0*s0*leftfoot_v*leftfoot_v)
        right_score = torch.exp(-10.0*s1*s1*rightfoot_v*rightfoot_v)
        return left_score + right_score
    
    def _reward_contact_force_diff(self):
        contact_force = self.contact_forces[:, self.feet_indices, 2]
        # print("contact_force_diff: ")
        # print(torch.clip(contact_force-550,min=0.0))
        # print(1.0 - torch.exp(-0.1*torch.sum(torch.square(torch.clip(contact_force-550,min=0.0)),dim=-1)) )
        return torch.exp(-0.05*torch.sum(torch.square(torch.clip(contact_force-550.0,min=0.0)),dim=-1))   
    def _reward_feet_speed_close_to_ground_positive(self):
        left_foot_height = torch.mean(
            self.rigid_body_states[:, self.feet_indices][:, 0, 2].unsqueeze(1) - self.measured_heights-0.054, dim=1)
        right_foot_height = torch.mean(
            self.rigid_body_states[:, self.feet_indices][:, 1, 2].unsqueeze(1) - self.measured_heights-0.054, dim=1)

        error_left_foot_speed_close_to_ground = \
            torch.norm(self.avg_feet_spd_xyz[:, 0, :2], dim=1) \
            * torch.abs(left_foot_height - self.cfg.rewards.feetheight_target / 2) \
            * (left_foot_height < self.cfg.rewards.feetheight_target / 2)
        error_right_foot_speed_close_to_ground = \
            torch.norm(self.avg_feet_spd_xyz[:, 1, :2], dim=1) \
            * torch.abs(right_foot_height - self.cfg.rewards.feetheight_target / 2) \
            * (right_foot_height < self.cfg.rewards.feetheight_target / 2)

        error_feet_speed_close_to_ground = error_left_foot_speed_close_to_ground + \
                                           error_right_foot_speed_close_to_ground
        reward_feet_speed_close_to_ground = torch.exp(-100.0 * error_feet_speed_close_to_ground)
        # print("feet_speed_close_to_ground: ")
        # print(reward_feet_speed_close_to_ground)
        # print("left_foot_height: ")
        # print(left_foot_height)
        return reward_feet_speed_close_to_ground

    def _reward_feet_double_fly_penalty(self):
        double_fly = self.avg_feet_frc[:, 0].clip(min=0.0) + self.avg_feet_frc[:, 1].clip(min=0.0) 
        double_fly = double_fly.clip(max = 50.0)
        double_fly = -double_fly + 50.0
        # print(double_fly)
        return torch.exp(-5.0*double_fly)
    