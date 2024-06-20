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

import torch
import torch.nn as nn
import torch.optim as optim

from rsl_rl.modules import ActorCritic
from rsl_rl.storage import RolloutStorage

class PPO_GR1T1:
    actor_critic: ActorCritic
    def __init__(self,
                 actor_critic,
                 num_learning_epochs=1,
                 num_mini_batches=1,
                 clip_param=0.2,
                 gamma=0.998,
                 lam=0.95,
                 value_loss_coef=1.0,
                 entropy_coef=0.0,
                 learning_rate=1e-3,
                 max_grad_norm=1.0,
                 use_clipped_value_loss=True,
                 schedule="fixed",
                 desired_kl=0.01,
                 mirror_coef = 1.0,
                 device='cpu',
                 ):

        self.device = device

        self.desired_kl = desired_kl
        self.schedule = schedule
        self.learning_rate = learning_rate
        self.mirror_coef = mirror_coef
        self.action_coef = mirror_coef
        print("mirror_coef: ", mirror_coef)

        # PPO components
        self.actor_critic = actor_critic
        self.actor_critic.to(self.device)
        self.storage = None # initialized later
        self.optimizer = optim.Adam(self.actor_critic.parameters(), lr=learning_rate)
        self.transition = RolloutStorage.Transition()

        # PPO parameters
        self.clip_param = clip_param
        self.num_learning_epochs = num_learning_epochs
        self.num_mini_batches = num_mini_batches
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.gamma = gamma
        self.lam = lam
        self.max_grad_norm = max_grad_norm
        self.use_clipped_value_loss = use_clipped_value_loss

    def init_storage(self, num_envs, num_transitions_per_env, actor_obs_shape, critic_obs_shape, action_shape):
        self.storage = RolloutStorage(num_envs, num_transitions_per_env, actor_obs_shape, critic_obs_shape, action_shape, self.device)

    def test_mode(self):
        self.actor_critic.test()
    
    def train_mode(self):
        self.actor_critic.train()

    def act(self, obs, critic_obs):
        if self.actor_critic.is_recurrent:
            self.transition.hidden_states = self.actor_critic.get_hidden_states()
        # Compute the actions and values
        self.transition.actions = self.actor_critic.act(obs).detach()
        self.transition.values = self.actor_critic.evaluate(critic_obs).detach()
        self.transition.actions_log_prob = self.actor_critic.get_actions_log_prob(self.transition.actions).detach()
        self.transition.action_mean = self.actor_critic.action_mean.detach()
        self.transition.action_sigma = self.actor_critic.action_std.detach()
        # need to record obs and critic_obs before env.step()
        self.transition.observations = obs
        self.transition.critic_observations = critic_obs
        return self.transition.actions
    
    def process_env_step(self, rewards, dones, infos):
        self.transition.rewards = rewards.clone()
        self.transition.dones = dones
        # Bootstrapping on time outs
        if 'time_outs' in infos:
            self.transition.rewards += self.gamma * torch.squeeze(self.transition.values * infos['time_outs'].unsqueeze(1).to(self.device), 1)

        # Record the transition
        self.storage.add_transitions(self.transition)
        self.transition.clear()
        self.actor_critic.reset(dones)
    
    def compute_returns(self, last_critic_obs):
        last_values= self.actor_critic.evaluate(last_critic_obs).detach()
        self.storage.compute_returns(last_values, self.gamma, self.lam)

    def update(self):
        mean_value_loss = 0
        mean_surrogate_loss = 0
        mean_mirror_loss = 0.0
        mean_action_loss = 0.0
        if self.actor_critic.is_recurrent:
            generator = self.storage.reccurent_mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)
        else:
            generator = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)
        for obs_batch, critic_obs_batch, actions_batch, target_values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch, \
            old_mu_batch, old_sigma_batch, hid_states_batch, masks_batch in generator:

                self.actor_critic.act(obs_batch, masks=masks_batch, hidden_states=hid_states_batch[0])
                actions_log_prob_batch = self.actor_critic.get_actions_log_prob(actions_batch)
                value_batch = self.actor_critic.evaluate(critic_obs_batch, masks=masks_batch, hidden_states=hid_states_batch[1])
                mu_batch = self.actor_critic.action_mean
                sigma_batch = self.actor_critic.action_std
                entropy_batch = self.actor_critic.entropy

                # Mirror loss
                if self.actor_critic.is_recurrent:
                    if self.mirror_coef != 0:
                        new_action_batch = self.actor_critic.act_for_mirror(obs_batch,masks=masks_batch,hidden_states=None)
                        mirror_obs_batch = self._get_mirror_obs_recurrent(obs_batch)
                        mirror_action_batch = self.actor_critic.act_for_mirror(mirror_obs_batch,masks=masks_batch,hidden_states=None)

                        mirror_action_batch_temp = mirror_action_batch.clone()
                        mirror_action_batch[:, :,0:6] = mirror_action_batch_temp[:, :,6:12]
                        mirror_action_batch[:, :,6:12] = mirror_action_batch_temp[:, :,0:6]
                        # get mirror action
                        mirror_action_batch[:, :,0] = - mirror_action_batch[:, :,0]
                        mirror_action_batch[:,:, 6] = - mirror_action_batch[:,:, 6]
                        mirror_action_batch[:,:, 1] = - mirror_action_batch[:, :,1]
                        mirror_action_batch[:,:, 7] = - mirror_action_batch[:,:, 7]
                        mirror_action_batch[:,:, 5] = - mirror_action_batch[:, :,5]
                        mirror_action_batch[:,:, 11] = - mirror_action_batch[:,:, 11]
                        #
                        mirror_action_batch[:, :,12] = - mirror_action_batch[:, :,12]
                        mirror_action_batch[:,:, 14] = - mirror_action_batch[:,:, 14]
                        #
                        mirror_action_batch[:,:, 15:19] = mirror_action_batch_temp[:,:, 19:23]
                        mirror_action_batch[:, :,19:23] = mirror_action_batch_temp[:, :,15:19]
                        #
                        mirror_action_batch[:,:, 16] = - mirror_action_batch[:, :,16]
                        mirror_action_batch[:,:, 17] = - mirror_action_batch[:,:, 17]
                        mirror_action_batch[:, :,20] = - mirror_action_batch[:, :,20]
                        mirror_action_batch[:, :,21] = - mirror_action_batch[:, :,21]

                        # mirror_loss = (mirror_action_batch - actions_batch).pow(2).mean()
                        mse_loss = nn.MSELoss()
                        # mirror_loss = mse_loss(mirror_action_batch[:,:,0:12], new_action_batch[:,:,0:12])
                        mirror_loss = mse_loss(mirror_action_batch, new_action_batch)
                        # mirror_loss = mse_loss(mirror_action_batch[:, :, 0:12], new_action_batch[:, :, 0:12])
                        # print("mirror loss: ")
                        # print(mirror_loss)
                    else:
                        mirror_loss = 0
                else:
                    if self.mirror_coef != 0:
                        new_action_batch = self.actor_critic.act_inference(obs_batch)
                        mirror_obs_batch = self._get_mirror_obs(obs_batch)
                        mirror_action_batch = self.actor_critic.act_inference(mirror_obs_batch)

                        mirror_action_batch_temp = mirror_action_batch.clone()
                        mirror_action_batch[:, 0:6] = mirror_action_batch_temp[:, 6:12]
                        mirror_action_batch[:, 6:12] = mirror_action_batch_temp[:, 0:6]
                        # get mirror action
                        mirror_action_batch[:, 0] = - mirror_action_batch[:, 0]
                        mirror_action_batch[:, 6] = - mirror_action_batch[:, 6]
                        mirror_action_batch[:, 1] = - mirror_action_batch[:, 1]
                        mirror_action_batch[:, 7] = - mirror_action_batch[:, 7]
                        mirror_action_batch[:, 5] = - mirror_action_batch[:, 5]
                        mirror_action_batch[:, 11] = - mirror_action_batch[:, 11]
                        #
                        # mirror_action_batch[:, 12] = - mirror_action_batch[:, 12]
                        # mirror_action_batch[:, 14] = - mirror_action_batch[:, 14]
                        # #
                        # mirror_action_batch[:, 15:19] = mirror_action_batch_temp[:, 19:23]
                        # mirror_action_batch[:, 19:23] = mirror_action_batch_temp[:, 15:19]
                        # #
                        # mirror_action_batch[:, 16] = - mirror_action_batch[:, 16]
                        # mirror_action_batch[:, 17] = - mirror_action_batch[:, 17]
                        # mirror_action_batch[:, 20] = - mirror_action_batch[:, 20]
                        # mirror_action_batch[:, 21] = - mirror_action_batch[:, 21]

                        # mirror_loss = (mirror_action_batch - actions_batch).pow(2).mean()
                        mse_loss = nn.MSELoss()
                        mirror_loss = mse_loss(mirror_action_batch, new_action_batch)
                    else:
                        mirror_loss = 0

                # mirror_action_batch.requires_grad = True
                # new_action_batch.requires_grad = True

                # time frequency analysis loss
                # to do


                # KL
                if self.desired_kl != None and self.schedule == 'adaptive':
                    with torch.inference_mode():
                        kl = torch.sum(
                            torch.log(sigma_batch / old_sigma_batch + 1.e-5) + (torch.square(old_sigma_batch) + torch.square(old_mu_batch - mu_batch)) / (2.0 * torch.square(sigma_batch)) - 0.5, axis=-1)
                        kl_mean = torch.mean(kl)

                        if kl_mean > self.desired_kl * 2.0:
                            self.learning_rate = max(1e-5, self.learning_rate / 1.5)
                        elif kl_mean < self.desired_kl / 2.0 and kl_mean > 0.0:
                            self.learning_rate = min(1e-2, self.learning_rate * 1.5)
                        
                        for param_group in self.optimizer.param_groups:
                            param_group['lr'] = self.learning_rate


                # Surrogate loss
                ratio = torch.exp(actions_log_prob_batch - torch.squeeze(old_actions_log_prob_batch))
                surrogate = -torch.squeeze(advantages_batch) * ratio
                surrogate_clipped = -torch.squeeze(advantages_batch) * torch.clamp(ratio, 1.0 - self.clip_param,
                                                                                1.0 + self.clip_param)
                surrogate_loss = torch.max(surrogate, surrogate_clipped).mean()
                # print("surrogate_loss: ")
                # print(surrogate_loss)
                # Value function loss
                if self.use_clipped_value_loss:
                    value_clipped = target_values_batch + (value_batch - target_values_batch).clamp(-self.clip_param,
                                                                                                    self.clip_param)
                    value_losses = (value_batch - returns_batch).pow(2)
                    value_losses_clipped = (value_clipped - returns_batch).pow(2)
                    value_loss = torch.max(value_losses, value_losses_clipped).mean()
                else:
                    value_loss = (returns_batch - value_batch).pow(2).mean()
                
                if self.mirror_coef != 0:
                    loss = surrogate_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy_batch.mean() + self.mirror_coef * mirror_loss
                else:
                    loss = surrogate_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy_batch.mean()

                # action loss
                # action_mse_loss = nn.MSELoss()
                # action_loss = action_mse_loss(actions_batch[:,:,[5,11,14,15,18,19]])
                # loss += self.action_coef*action_loss
                # print("action_loss: ")
                # print(action_loss)
                # Gradient step
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.actor_critic.parameters(), self.max_grad_norm)
                self.optimizer.step()

                mean_value_loss += value_loss.item()
                mean_surrogate_loss += surrogate_loss.item()
                if self.mirror_coef != 0:
                    mean_mirror_loss += mirror_loss.item()
                else:
                    mean_mirror_loss += mirror_loss

        num_updates = self.num_learning_epochs * self.num_mini_batches
        mean_value_loss /= num_updates
        mean_surrogate_loss /= num_updates
        mean_mirror_loss /= num_updates
        self.storage.clear()

        return mean_value_loss, mean_surrogate_loss, mean_mirror_loss
    
    def _get_mirror_obs(self, obs_batch):
        mirror_obs_batch = obs_batch.clone()

        # mirror base_line_vel
        # neg y vel
        # mirror_obs_batch[:, 0] = obs_batch[:, 87]

        mirror_obs_batch[:, 0] = mirror_obs_batch[:, 0]
        mirror_obs_batch[:, 1] = - mirror_obs_batch[:, 1]
        mirror_obs_batch[:, 2] = mirror_obs_batch[:, 2]
        # mirror base_angle_vel
        # neg roll and yaw
        mirror_obs_batch[:, 3] = - mirror_obs_batch[:, 3]
        mirror_obs_batch[:, 4] = mirror_obs_batch[:, 4]
        mirror_obs_batch[:, 5] = - mirror_obs_batch[:, 5]

        # mirror projected_gravity
        # neg y
        mirror_obs_batch[:, 6] = mirror_obs_batch[:, 6]
        mirror_obs_batch[:, 7] = - mirror_obs_batch[:, 7]
        mirror_obs_batch[:, 8] = mirror_obs_batch[:, 8]

        # mirror commands
        # neg y, yaw
        mirror_obs_batch[:, 9] = mirror_obs_batch[:, 9]
        mirror_obs_batch[:, 10] = - mirror_obs_batch[:, 10]
        mirror_obs_batch[:, 11] = - mirror_obs_batch[:, 11]



        # mirror leg_dof_pos
        mirror_obs_batch[:, 12:18] = obs_batch[:, 18:24]
        mirror_obs_batch[:, 18:24] = obs_batch[:, 12:18]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, 12] = - mirror_obs_batch[:, 12]
        mirror_obs_batch[:, 13] = - mirror_obs_batch[:, 13]
        mirror_obs_batch[:, 17] = - mirror_obs_batch[:, 17]
        mirror_obs_batch[:, 18] = - mirror_obs_batch[:, 18]
        mirror_obs_batch[:, 19] = - mirror_obs_batch[:, 19]
        mirror_obs_batch[:, 23] = - mirror_obs_batch[:, 23]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,24:27] = obs_batch[:, 24:27]
        # neg yaw roll
        mirror_obs_batch[:, 24] = - mirror_obs_batch[:, 24]
        mirror_obs_batch[:, 26] = - mirror_obs_batch[:, 26]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, 27:31] = obs_batch[:, 31:35]
        mirror_obs_batch[:, 31:35] = obs_batch[:, 27:31]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, 28] = - mirror_obs_batch[:, 28]
        mirror_obs_batch[:, 29] = - mirror_obs_batch[:, 29]
        mirror_obs_batch[:, 32] = - mirror_obs_batch[:, 32]
        mirror_obs_batch[:, 33] = - mirror_obs_batch[:, 33]

        # mirror dof_vel
        mirror_obs_batch[:, 35:41] = obs_batch[:, 41:47]
        mirror_obs_batch[:, 41:47] = obs_batch[:, 35:41]

        # neg dof_vel roll yaw
        mirror_obs_batch[:, 35] = - mirror_obs_batch[:, 35]
        mirror_obs_batch[:, 36] = - mirror_obs_batch[:, 36]
        mirror_obs_batch[:, 40] = - mirror_obs_batch[:, 40]

        mirror_obs_batch[:, 41] = - mirror_obs_batch[:, 41]
        mirror_obs_batch[:, 42] = - mirror_obs_batch[:, 42]
        mirror_obs_batch[:, 46] = - mirror_obs_batch[:, 46]

        # mirror waist
        mirror_obs_batch[:, 47:50] = obs_batch[:, 47:50]
        # neg yaw roll
        mirror_obs_batch[:, 47] = - mirror_obs_batch[:, 47]
        mirror_obs_batch[:, 49] = - mirror_obs_batch[:, 49]        

        # mirror arm_dof_vel
        mirror_obs_batch[:, 50:54] = obs_batch[:, 54:58]
        mirror_obs_batch[:, 54:58] = obs_batch[:, 50:54]

        # neg arm_dof_vel roll yaw
        mirror_obs_batch[:, 51] = - mirror_obs_batch[:, 51]
        mirror_obs_batch[:, 52] = - mirror_obs_batch[:, 52]
        mirror_obs_batch[:, 55] = - mirror_obs_batch[:, 55]
        mirror_obs_batch[:, 56] = - mirror_obs_batch[:, 56]

        # mirror previous_actions_leg
        mirror_obs_batch[:, 58:64] = obs_batch[:, 64:70]
        mirror_obs_batch[:, 64:70] = obs_batch[:, 58:64]

        # neg actions_leg roll yaw
        mirror_obs_batch[:, 58] = - mirror_obs_batch[:, 58]
        mirror_obs_batch[:, 59] = - mirror_obs_batch[:, 59]
        mirror_obs_batch[:, 63] = - mirror_obs_batch[:, 63]
        mirror_obs_batch[:, 64] = - mirror_obs_batch[:, 64]
        mirror_obs_batch[:, 65] = - mirror_obs_batch[:, 65]
        mirror_obs_batch[:, 69] = - mirror_obs_batch[:, 69]

        # mirror waist
        mirror_obs_batch[:, 70:73] = obs_batch[:, 70:73]
        # neg yaw roll
        mirror_obs_batch[:, 70] = - mirror_obs_batch[:, 70]
        mirror_obs_batch[:, 72] = - mirror_obs_batch[:, 72]           

        # mirror arm_action
        mirror_obs_batch[:, 73:77] = obs_batch[:, 77:81]
        mirror_obs_batch[:, 77:81] = obs_batch[:, 73:77]

        # neg arm_action roll yaw
        mirror_obs_batch[:, 74] = - mirror_obs_batch[:, 74]
        mirror_obs_batch[:, 75] = - mirror_obs_batch[:, 75]
        mirror_obs_batch[:, 78] = - mirror_obs_batch[:, 78]
        mirror_obs_batch[:, 79] = - mirror_obs_batch[:, 79]

        # mirror sin & cos
        mirror_obs_batch[:, 81] = obs_batch[:, 82]
        mirror_obs_batch[:, 82] = obs_batch[:, 81]
        mirror_obs_batch[:, 83] = obs_batch[:, 84]
        mirror_obs_batch[:, 84] = obs_batch[:, 83]

        # mirror phase ratio
        mirror_obs_batch[:, 85] = obs_batch[:, 86]
        mirror_obs_batch[:, 86] = obs_batch[:, 85]

        # base_height

        # avg_x_vel
        mirror_obs_batch[:, 88] = obs_batch[:, 88]
        # mirror avg_y_vel,avg_yaw_vel
        mirror_obs_batch[:, 89] = - mirror_obs_batch[:, 89]
        mirror_obs_batch[:, 90] = - mirror_obs_batch[:, 90]
        
        
        # start = 91
        # mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        # mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # # neg leg_dof_pos roll yaw
        # mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        # mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        # mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        # mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        # mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        # mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # # mirror waist yaw pitch roll
        # mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # # neg yaw roll
        # mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        # mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # # mirror arm_dof_pos
        # mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        # mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # # neg arm_dof_pos roll yaw
        # mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        # mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        # mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        # mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]

        # # mirror dof_vel
        # mirror_obs_batch[:, start+23:start+29] = obs_batch[:, start+29:start+35]
        # mirror_obs_batch[:, start+29:start+35] = obs_batch[:, start+23:start+29]

        # # neg dof_vel roll yaw
        # mirror_obs_batch[:, start+23] = - mirror_obs_batch[:, start+23]
        # mirror_obs_batch[:, start+24] = - mirror_obs_batch[:, start+24]
        # mirror_obs_batch[:, start+28] = - mirror_obs_batch[:, start+28]

        # mirror_obs_batch[:, start+29] = - mirror_obs_batch[:, start+29]
        # mirror_obs_batch[:, start+30] = - mirror_obs_batch[:, start+30]
        # mirror_obs_batch[:, start+34] = - mirror_obs_batch[:, start+34]

        # # mirror waist
        # mirror_obs_batch[:, start+35:start+38] = obs_batch[:, start+35:start+38]
        # # neg yaw roll
        # mirror_obs_batch[:, start+35] = - mirror_obs_batch[:, start+35]
        # mirror_obs_batch[:, start+37] = - mirror_obs_batch[:, start+37]        

        # # mirror arm_dof_vel
        # mirror_obs_batch[:, start+38:start+42] = obs_batch[:, start+42:start+46]
        # mirror_obs_batch[:, start+42:start+46] = obs_batch[:, start+38:start+42]

        # # neg arm_dof_vel roll yaw
        # mirror_obs_batch[:, start+39] = - mirror_obs_batch[:, start+39]
        # mirror_obs_batch[:, start+40] = - mirror_obs_batch[:, start+40]
        # mirror_obs_batch[:, start+43] = - mirror_obs_batch[:, start+43]
        # mirror_obs_batch[:, start+44] = - mirror_obs_batch[:, start+44]
        
        start = 91
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]
        
        start = 114
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]
        
        start = 137
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]
        
        start = 160
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]

        start = 183
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]


        start = 206
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]
        
        start = 229
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]


        start = 252
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]
        
        start = 275
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]


        start = 298
        mirror_obs_batch[:, start:start+6] = obs_batch[:, start+6:start+12]
        mirror_obs_batch[:, start+6:start+12] = obs_batch[:, start:start+6]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, start] = - mirror_obs_batch[:, start]
        mirror_obs_batch[:, start+1] = - mirror_obs_batch[:, start+1]
        mirror_obs_batch[:, start+5] = - mirror_obs_batch[:, start+5]
        mirror_obs_batch[:, start+6] = - mirror_obs_batch[:, start+6]
        mirror_obs_batch[:, start+7] = - mirror_obs_batch[:, start+7]
        mirror_obs_batch[:, start+11] = - mirror_obs_batch[:, start+11]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:,start+12:start+15] = obs_batch[:, start+12:start+15]
        # neg yaw roll
        mirror_obs_batch[:, start+12] = - mirror_obs_batch[:, start+12]
        mirror_obs_batch[:, start+14] = - mirror_obs_batch[:, start+14]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, start+15:start+19] = obs_batch[:, start+19:start+23]
        mirror_obs_batch[:, start+19:start+23] = obs_batch[:, start+15:start+19]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, start+16] = - mirror_obs_batch[:, start+16]
        mirror_obs_batch[:, start+17] = - mirror_obs_batch[:, start+17]
        mirror_obs_batch[:, start+20] = - mirror_obs_batch[:, start+20]
        mirror_obs_batch[:, start+21] = - mirror_obs_batch[:, start+21]




       
        return mirror_obs_batch

    def _get_mirror_obs_recurrent(self, obs_batch):
        mirror_obs_batch = obs_batch.clone()

        # mirror base_line_vel
        # neg y vel
        mirror_obs_batch[:, :,0] = mirror_obs_batch[:, :,0]
        mirror_obs_batch[:, :,1] = - mirror_obs_batch[:, :,1]
        mirror_obs_batch[:, :,2] = mirror_obs_batch[:, :,2]
        # mirror base_angle_vel
        # neg roll and yaw
        mirror_obs_batch[:, :,3] = - mirror_obs_batch[:, :,3]
        mirror_obs_batch[:, :,4] = mirror_obs_batch[:,:, 4]
        mirror_obs_batch[:, :,5] = - mirror_obs_batch[:,:, 5]

        # mirror projected_gravity
        # neg y
        mirror_obs_batch[:, :,6] = mirror_obs_batch[:, :,6]
        mirror_obs_batch[:, :,7] = - mirror_obs_batch[:,:, 7]
        mirror_obs_batch[:, :,8] = mirror_obs_batch[:,:, 8]

        # mirror commands
        # neg y, yaw
        mirror_obs_batch[:, :,9] = mirror_obs_batch[:, :,9]
        mirror_obs_batch[:, :,10] = - mirror_obs_batch[:, :,10]
        mirror_obs_batch[:, :,11] = - mirror_obs_batch[:,:, 11]



        # mirror leg_dof_pos
        mirror_obs_batch[:, :,12:18] = obs_batch[:,:, 18:24]
        mirror_obs_batch[:, :,18:24] = obs_batch[:,:, 12:18]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, :,12] = - mirror_obs_batch[:,:, 12]
        mirror_obs_batch[:, :,13] = - mirror_obs_batch[:,:, 13]
        mirror_obs_batch[:, :,17] = - mirror_obs_batch[:,:, 17]
        mirror_obs_batch[:, :,18] = - mirror_obs_batch[:,:, 18]
        mirror_obs_batch[:, :,19] = - mirror_obs_batch[:,:, 19]
        mirror_obs_batch[:, :,23] = - mirror_obs_batch[:, :,23]

        # mirror waist yaw pitch roll
        mirror_obs_batch[:, :,24:27] = obs_batch[:,:, 24:27]
        # neg yaw roll
        mirror_obs_batch[:, :,24] = - mirror_obs_batch[:, :,24]
        mirror_obs_batch[:, :, 26] = - mirror_obs_batch[:, :,26]        

        # mirror arm_dof_pos
        mirror_obs_batch[:, :,27:31] = obs_batch[:, :,31:35]
        mirror_obs_batch[:,:, 31:35] = obs_batch[:,:, 27:31]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:,:, 28] = - mirror_obs_batch[:,:, 28]
        mirror_obs_batch[:,:, 29] = - mirror_obs_batch[:,:, 29]
        mirror_obs_batch[:,:, 32] = - mirror_obs_batch[:, :,32]
        mirror_obs_batch[:,:, 33] = - mirror_obs_batch[:,:, 33]

        # mirror dof_vel
        mirror_obs_batch[:,:, 35:41] = obs_batch[:, :,41:47]
        mirror_obs_batch[:,:, 41:47] = obs_batch[:, :,35:41]

        # neg dof_vel roll yaw
        mirror_obs_batch[:,:, 35] = - mirror_obs_batch[:,:, 35]
        mirror_obs_batch[:, :,36] = - mirror_obs_batch[:,:, 36]
        mirror_obs_batch[:,:, 40] = - mirror_obs_batch[:, :,40]

        mirror_obs_batch[:,:, 41] = - mirror_obs_batch[:,:, 41]
        mirror_obs_batch[:, :,42] = - mirror_obs_batch[:,:, 42]
        mirror_obs_batch[:, :,46] = - mirror_obs_batch[:, :,46]

        # mirror waist
        mirror_obs_batch[:,:, 47:50] = obs_batch[:, :,47:50]
        # neg yaw roll
        mirror_obs_batch[:,:, 47] = - mirror_obs_batch[:,:, 47]
        mirror_obs_batch[:,:, 49] = - mirror_obs_batch[:, :,49]        

        # mirror arm_dof_vel
        mirror_obs_batch[:,:, 50:54] = obs_batch[:, :,54:58]
        mirror_obs_batch[:,:, 54:58] = obs_batch[:, :,50:54]

        # neg arm_dof_vel roll yaw
        mirror_obs_batch[:,:, 51] = - mirror_obs_batch[:,:, 51]
        mirror_obs_batch[:,:, 52] = - mirror_obs_batch[:, :,52]
        mirror_obs_batch[:,:, 55] = - mirror_obs_batch[:,:, 55]
        mirror_obs_batch[:,:, 56] = - mirror_obs_batch[:, :,56]

        # mirror previous_actions_leg
        mirror_obs_batch[:,:, 58:64] = obs_batch[:,:, 64:70]
        mirror_obs_batch[:,:, 64:70] = obs_batch[:, :,58:64]

        # neg actions_leg roll yaw
        mirror_obs_batch[:,:, 58] = - mirror_obs_batch[:,:, 58]
        mirror_obs_batch[:,:, 59] = - mirror_obs_batch[:,:, 59]
        mirror_obs_batch[:,:, 63] = - mirror_obs_batch[:, :,63]
        mirror_obs_batch[:,:, 64] = - mirror_obs_batch[:, :,64]
        mirror_obs_batch[:,:, 65] = - mirror_obs_batch[:, :,65]
        mirror_obs_batch[:, :,69] = - mirror_obs_batch[:,:, 69]

        # mirror waist
        mirror_obs_batch[:, :,70:73] = obs_batch[:, :,70:73]
        # neg yaw roll
        mirror_obs_batch[:,:, 70] = - mirror_obs_batch[:,:, 70]
        mirror_obs_batch[:,:, 72] = - mirror_obs_batch[:, :,72]           

        # mirror arm_action
        mirror_obs_batch[:,:, 73:77] = obs_batch[:, :,77:81]
        mirror_obs_batch[:, :,77:81] = obs_batch[:, :,73:77]

        # neg arm_action roll yaw
        mirror_obs_batch[:,:, 74] = - mirror_obs_batch[:,:, 74]
        mirror_obs_batch[:,:, 75] = - mirror_obs_batch[:,:, 75]
        mirror_obs_batch[:, :,78] = - mirror_obs_batch[:, :,78]
        mirror_obs_batch[:,:, 79] = - mirror_obs_batch[:,:, 79]

        # mirror sin & cos
        mirror_obs_batch[:,:, 81] = obs_batch[:,:, 82]
        mirror_obs_batch[:,:, 82] = obs_batch[:,:, 81]
        mirror_obs_batch[:, :,83] = obs_batch[:,:, 84]
        mirror_obs_batch[:, :,84] = obs_batch[:, :,83]

        # mirror phase ratio
        mirror_obs_batch[:,:, 85] = obs_batch[:,:, 86]
        mirror_obs_batch[:,:, 86] = obs_batch[:, :,85]

        # base_height
        mirror_obs_batch[:,:, 87] = obs_batch[:, :,87]

        # avg_x_vel
        mirror_obs_batch[:,:, 88] = obs_batch[:, :,88]
        # mirror avg_y_vel,avg_yaw_vel
        mirror_obs_batch[:, :,89] = - mirror_obs_batch[:,:, 89]
        mirror_obs_batch[:,:, 90] = - mirror_obs_batch[:, :,90]

        return mirror_obs_batch
