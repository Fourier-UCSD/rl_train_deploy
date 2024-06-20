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
from rsl_rl.storage.replay_buffer import ReplayBuffer


class PPOAMP:
    actor_critic: ActorCritic

    def __init__(self,
                 actor_critic,
                 discriminator,
                 amp_data,
                 amp_normalizer,
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
                 device='cpu',
                 amp_replay_buffer_size=100000,
                 min_std=None,
                 ):

        print("----------------------------------")
        print("PPOAMP")

        self.device = device

        self.desired_kl = desired_kl
        self.schedule = schedule
        self.learning_rate = learning_rate
        self.mirror_coef = 4.0
        self.amploss_coef = 1.0
        self.min_std = min_std

        # Discriminator components
        self.discriminator = discriminator
        self.discriminator.to(self.device)
        self.amp_transition = RolloutStorage.Transition()
        self.amp_storage = ReplayBuffer(
            discriminator.input_dim // 2, amp_replay_buffer_size, device)
        self.amp_data = amp_data
        self.amp_normalizer = amp_normalizer

        # PPO components
        self.actor_critic = actor_critic
        self.actor_critic.to(self.device)
        self.storage = None  # initialized later

        # Optimizer for policy and discriminator.
        params = [
            {'params': self.actor_critic.parameters(), 'name': 'actor_critic'},
            {'params': self.discriminator.trunk.parameters(),
             'weight_decay': 10e-4, 'name': 'amp_trunk'},
            {'params': self.discriminator.amp_linear.parameters(),
             'weight_decay': 10e-2, 'name': 'amp_head'}]
        self.optimizer = optim.Adam(params, lr=learning_rate)
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
        self.storage = RolloutStorage(
            num_envs, num_transitions_per_env, actor_obs_shape, critic_obs_shape, action_shape, self.device)

    def test_mode(self):
        self.actor_critic.test()

    def train_mode(self):
        self.actor_critic.train()

    def act(self, obs, critic_obs, amp_obs):
        if self.actor_critic.is_recurrent:
            self.transition.hidden_states = self.actor_critic.get_hidden_states()
        # Compute the actions and values
        aug_obs, aug_critic_obs = obs.detach(), critic_obs.detach()
        self.transition.actions = self.actor_critic.act(aug_obs).detach()
        self.transition.values = self.actor_critic.evaluate(aug_critic_obs).detach()
        self.transition.actions_log_prob = self.actor_critic.get_actions_log_prob(self.transition.actions).detach()
        self.transition.action_mean = self.actor_critic.action_mean.detach()
        self.transition.action_sigma = self.actor_critic.action_std.detach()
        # need to record obs and critic_obs before env.step()
        self.transition.observations = obs
        self.transition.critic_observations = critic_obs
        self.amp_transition.observations = amp_obs
        return self.transition.actions

    def process_env_step(self, rewards, dones, infos, amp_obs):
        self.transition.rewards = rewards.clone()
        self.transition.dones = dones
        # Bootstrapping on time outs
        if 'time_outs' in infos:
            self.transition.rewards += self.gamma * torch.squeeze(self.transition.values * infos['time_outs'].unsqueeze(1).to(self.device), 1)

        not_done_idxs = (dones == False).nonzero().squeeze()
        self.amp_storage.insert(
            self.amp_transition.observations, amp_obs)

        # Record the transition
        self.storage.add_transitions(self.transition)
        self.transition.clear()
        self.amp_transition.clear()
        self.actor_critic.reset(dones)

    def compute_returns(self, last_critic_obs):
        aug_last_critic_obs = last_critic_obs.detach()
        last_values = self.actor_critic.evaluate(aug_last_critic_obs).detach()
        self.storage.compute_returns(last_values, self.gamma, self.lam)

    def update(self):
        mean_value_loss = 0
        mean_surrogate_loss = 0
        mean_mirror_loss = 0
        mean_amp_loss = 0
        mean_grad_pen_loss = 0
        mean_policy_pred = 0
        mean_expert_pred = 0
        if self.actor_critic.is_recurrent:
            generator = self.storage.reccurent_mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)
        else:
            generator = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)

        amp_policy_generator = self.amp_storage.feed_forward_generator(
            self.num_learning_epochs * self.num_mini_batches,
            self.storage.num_envs * self.storage.num_transitions_per_env //
            self.num_mini_batches)
        amp_expert_generator = self.amp_data.feed_forward_generator(
            self.num_learning_epochs * self.num_mini_batches,
            self.storage.num_envs * self.storage.num_transitions_per_env //
            self.num_mini_batches)
        for sample, sample_amp_policy, sample_amp_expert in zip(generator, amp_policy_generator, amp_expert_generator):

            obs_batch, critic_obs_batch, actions_batch, target_values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch, \
                old_mu_batch, old_sigma_batch, hid_states_batch, masks_batch = sample
            aug_obs_batch = obs_batch.detach()
            self.actor_critic.act(aug_obs_batch, masks=masks_batch, hidden_states=hid_states_batch[0])
            actions_log_prob_batch = self.actor_critic.get_actions_log_prob(actions_batch)
            aug_critic_obs_batch = critic_obs_batch.detach()
            value_batch = self.actor_critic.evaluate(aug_critic_obs_batch, masks=masks_batch, hidden_states=hid_states_batch[1])
            mu_batch = self.actor_critic.action_mean
            sigma_batch = self.actor_critic.action_std
            entropy_batch = self.actor_critic.entropy

            # Mirror loss
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
                mirror_action_batch[:, 12] = - mirror_action_batch[:, 12]
                mirror_action_batch[:, 14] = - mirror_action_batch[:, 14]
                #
                mirror_action_batch[:, 15:19] = mirror_action_batch_temp[:, 19:23]
                mirror_action_batch[:, 19:23] = mirror_action_batch_temp[:, 15:19]
                #
                mirror_action_batch[:, 16] = - mirror_action_batch[:, 16]
                mirror_action_batch[:, 18] = - mirror_action_batch[:, 18]
                mirror_action_batch[:, 20] = - mirror_action_batch[:, 20]
                mirror_action_batch[:, 22] = - mirror_action_batch[:, 22]

                # mirror_loss = (mirror_action_batch - actions_batch).pow(2).mean()
                mse_loss = nn.MSELoss()
                mirror_loss = mse_loss(mirror_action_batch, new_action_batch)
            else:
                mirror_loss = 0

            # mirror_action_batch.requires_grad = True
            # new_action_batch.requires_grad = True

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

            # Value function loss
            if self.use_clipped_value_loss:
                value_clipped = target_values_batch + (value_batch - target_values_batch).clamp(-self.clip_param,
                                                                                                self.clip_param)
                value_losses = (value_batch - returns_batch).pow(2)
                value_losses_clipped = (value_clipped - returns_batch).pow(2)
                value_loss = torch.max(value_losses, value_losses_clipped).mean()
            else:
                value_loss = (returns_batch - value_batch).pow(2).mean()

            # Discriminator loss.
            policy_state, policy_next_state = sample_amp_policy
            expert_state, expert_next_state = sample_amp_expert
            if self.amp_normalizer is not None:
                with torch.no_grad():
                    policy_state = self.amp_normalizer.normalize_torch(policy_state, self.device)
                    policy_next_state = self.amp_normalizer.normalize_torch(policy_next_state, self.device)
                    expert_state = self.amp_normalizer.normalize_torch(expert_state, self.device)
                    expert_next_state = self.amp_normalizer.normalize_torch(expert_next_state, self.device)
            policy_d = self.discriminator(torch.cat([policy_state, policy_next_state], dim=-1))
            expert_d = self.discriminator(torch.cat([expert_state, expert_next_state], dim=-1))
            expert_loss = torch.nn.MSELoss()(
                expert_d, torch.ones(expert_d.size(), device=self.device))
            policy_loss = torch.nn.MSELoss()(
                policy_d, -1 * torch.ones(policy_d.size(), device=self.device))
            amp_loss = 0.5 * (expert_loss + policy_loss)
            grad_pen_loss = self.discriminator.compute_grad_pen(
                *sample_amp_expert, lambda_=10)

            # Compute total loss.
            if self.mirror_coef != 0:
                loss = (
                        surrogate_loss +
                        self.value_loss_coef * value_loss -
                        self.entropy_coef * entropy_batch.mean() +
                        self.mirror_coef * mirror_loss +
                        self.amploss_coef * amp_loss + self.amploss_coef * grad_pen_loss)
            else:
                loss = (
                        surrogate_loss +
                        self.value_loss_coef * value_loss -
                        self.entropy_coef * entropy_batch.mean() +
                        self.amploss_coef * amp_loss + self.amploss_coef * grad_pen_loss)
            # Gradient step
            self.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.actor_critic.parameters(), self.max_grad_norm)
            self.optimizer.step()

            if not self.actor_critic.fixed_std and self.min_std is not None:
                self.actor_critic.std.data = self.actor_critic.std.data.clamp(min=self.min_std)

            if self.amp_normalizer is not None:
                self.amp_normalizer.update(policy_state.cpu().numpy())
                self.amp_normalizer.update(expert_state.cpu().numpy())

            mean_value_loss += value_loss.item()
            mean_surrogate_loss += surrogate_loss.item()
            mean_mirror_loss += mirror_loss.item()
            mean_amp_loss += amp_loss.item()
            mean_grad_pen_loss += grad_pen_loss.item()
            mean_policy_pred += policy_d.mean().item()
            mean_expert_pred += expert_d.mean().item()

        num_updates = self.num_learning_epochs * self.num_mini_batches
        mean_value_loss /= num_updates
        mean_surrogate_loss /= num_updates
        mean_mirror_loss /= num_updates
        mean_amp_loss /= num_updates
        mean_grad_pen_loss /= num_updates
        mean_policy_pred /= num_updates
        mean_expert_pred /= num_updates
        self.storage.clear()

        return mean_value_loss, mean_surrogate_loss, mean_amp_loss, mean_grad_pen_loss, mean_policy_pred, mean_expert_pred, mean_mirror_loss

    def _get_mirror_obs(self, obs_batch):
        mirror_obs_batch = obs_batch.clone()

        # mirror base_line_vel 
        # neg y vel
        mirror_obs_batch[:, 1] = - mirror_obs_batch[:, 1]

        # mirror base_angle_vel
        # neg roll and yaw
        mirror_obs_batch[:, 3] = - mirror_obs_batch[:, 3]
        mirror_obs_batch[:, 5] = - mirror_obs_batch[:, 5]

        # mirror projected_gravity
        # neg y
        mirror_obs_batch[:, 7] = - mirror_obs_batch[:, 7]

        # mirror commands
        # neg y, yaw
        mirror_obs_batch[:, 10] = - mirror_obs_batch[:, 10]
        mirror_obs_batch[:, 11] = - mirror_obs_batch[:, 11]

        # mirror leg_dof_pos
        mirror_obs_batch[:, 12:18] = obs_batch[:, 18:24]
        mirror_obs_batch[:, 18:24] = obs_batch[:, 12:18]

        # neg leg_dof_pos roll yaw
        mirror_obs_batch[:, 12] = - mirror_obs_batch[:, 12]
        mirror_obs_batch[:, 18] = - mirror_obs_batch[:, 18]
        mirror_obs_batch[:, 13] = - mirror_obs_batch[:, 13]
        mirror_obs_batch[:, 19] = - mirror_obs_batch[:, 19]
        mirror_obs_batch[:, 17] = - mirror_obs_batch[:, 17]
        mirror_obs_batch[:, 23] = - mirror_obs_batch[:, 23]

        # mirror torso_angle_pos
        # neg roll and yaw
        mirror_obs_batch[:, 24] = - mirror_obs_batch[:, 24]
        mirror_obs_batch[:, 26] = - mirror_obs_batch[:, 26]

        # mirror arm_dof_pos
        mirror_obs_batch[:, 27:31] = obs_batch[:, 31:35]
        mirror_obs_batch[:, 31:35] = obs_batch[:, 27:31]

        # neg arm_dof_pos roll yaw
        mirror_obs_batch[:, 28] = - mirror_obs_batch[:, 28]
        mirror_obs_batch[:, 30] = - mirror_obs_batch[:, 30]
        mirror_obs_batch[:, 32] = - mirror_obs_batch[:, 32]
        mirror_obs_batch[:, 34] = - mirror_obs_batch[:, 34]

        # mirror dof_vel
        mirror_obs_batch[:, 35:41] = obs_batch[:, 41:47]
        mirror_obs_batch[:, 41:47] = obs_batch[:, 35:41]
        # mirror_obs_batch[:, 24:30] = obs_batch[:, 30:36]
        # mirror_obs_batch[:, 30:36] = obs_batch[:, 24:30]

        # neg dof_vel roll yaw
        mirror_obs_batch[:, 35] = - mirror_obs_batch[:, 35]
        mirror_obs_batch[:, 41] = - mirror_obs_batch[:, 41]
        mirror_obs_batch[:, 36] = - mirror_obs_batch[:, 36]
        mirror_obs_batch[:, 42] = - mirror_obs_batch[:, 42]
        mirror_obs_batch[:, 40] = - mirror_obs_batch[:, 40]
        mirror_obs_batch[:, 46] = - mirror_obs_batch[:, 46]

        # mirror torso_angle_vel
        # neg roll and yaw
        mirror_obs_batch[:, 47] = - mirror_obs_batch[:, 47]
        mirror_obs_batch[:, 49] = - mirror_obs_batch[:, 49]

        # mirror arm_dof_vel
        mirror_obs_batch[:, 50:54] = obs_batch[:, 54:58]
        mirror_obs_batch[:, 54:58] = obs_batch[:, 50:54]

        # neg arm_dof_vel roll yaw
        mirror_obs_batch[:, 51] = - mirror_obs_batch[:, 51]
        mirror_obs_batch[:, 53] = - mirror_obs_batch[:, 53]
        mirror_obs_batch[:, 55] = - mirror_obs_batch[:, 55]
        mirror_obs_batch[:, 57] = - mirror_obs_batch[:, 57]

        # mirror previous_actions_leg
        mirror_obs_batch[:, 58:64] = obs_batch[:, 64:70]
        mirror_obs_batch[:, 64:70] = obs_batch[:, 58:64]
        # mirror_obs_batch[:, 36:42] = obs_batch[:, 42:48]
        # mirror_obs_batch[:, 42:48] = obs_batch[:, 36:42]

        # neg actions_leg roll yaw
        mirror_obs_batch[:, 58] = - mirror_obs_batch[:, 58]
        mirror_obs_batch[:, 64] = - mirror_obs_batch[:, 64]
        mirror_obs_batch[:, 59] = - mirror_obs_batch[:, 59]
        mirror_obs_batch[:, 65] = - mirror_obs_batch[:, 65]
        mirror_obs_batch[:, 63] = - mirror_obs_batch[:, 63]
        mirror_obs_batch[:, 69] = - mirror_obs_batch[:, 69]

        # mirror torso_action
        # neg roll and yaw
        mirror_obs_batch[:, 70] = - mirror_obs_batch[:, 70]
        mirror_obs_batch[:, 72] = - mirror_obs_batch[:, 72]

        # mirror arm_action
        mirror_obs_batch[:, 73:77] = obs_batch[:, 77:81]
        mirror_obs_batch[:, 77:81] = obs_batch[:, 73:77]

        # neg arm_action roll yaw
        mirror_obs_batch[:, 74] = - mirror_obs_batch[:, 74]
        mirror_obs_batch[:, 76] = - mirror_obs_batch[:, 76]
        mirror_obs_batch[:, 78] = - mirror_obs_batch[:, 78]
        mirror_obs_batch[:, 80] = - mirror_obs_batch[:, 80]

        # mirror sin & cos
        mirror_obs_batch[:, 81] = obs_batch[:, 82]
        mirror_obs_batch[:, 82] = obs_batch[:, 81]
        mirror_obs_batch[:, 83] = obs_batch[:, 84]
        mirror_obs_batch[:, 84] = obs_batch[:, 83]

        # mirror phase ratio
        mirror_obs_batch[:, 85] = obs_batch[:, 86]
        mirror_obs_batch[:, 86] = obs_batch[:, 85]

        # mirror avg_y_vel
        mirror_obs_batch[:, 89] = - mirror_obs_batch[:, 89]

        # # mirror height
        # height_bit = 54
        # for idx in range(11):
        #     mirror_obs_batch[:, (height_bit + idx*11)] = obs_batch[:, (height_bit + idx*11) + 10]
        #     mirror_obs_batch[:, (height_bit + idx*11) + 1] = obs_batch[:, (height_bit + idx*11) + 9]
        #     mirror_obs_batch[:, (height_bit + idx*11) + 2] = obs_batch[:, (height_bit + idx*11) + 8]
        #     mirror_obs_batch[:, (height_bit + idx*11) + 3] = obs_batch[:, (height_bit + idx*11) + 7]
        #     mirror_obs_batch[:, (height_bit + idx*11) + 4] = obs_batch[:, (height_bit + idx*11) + 6]

        #     mirror_obs_batch[:, (height_bit + idx*11) + 10] = obs_batch[:, (height_bit + idx*11)]
        #     mirror_obs_batch[:, (height_bit + idx*11) + 9] = obs_batch[:, (height_bit + idx*11) + 1]
        #     mirror_obs_batch[:, (height_bit + idx*11) + 8] = obs_batch[:, (height_bit + idx*11) + 2]
        #     mirror_obs_batch[:, (height_bit + idx*11) + 7] = obs_batch[:, (height_bit + idx*11) + 3]
        #     mirror_obs_batch[:, (height_bit + idx*11) + 6] = obs_batch[:, (height_bit + idx*11) + 4]
        #
        # # mirror com y
        # mirror_obs_batch[:, 58] = obs_batch[:, 58]

        # # mirror motor strenth
        # mirror_obs_batch[:, 60:66] = obs_batch[:, 66:72]
        # mirror_obs_batch[:, 66:72] = obs_batch[:, 60:66]
        # # mirror_obs_batch[:, 63:69] = obs_batch[:, 69:75]
        # # mirror_obs_batch[:, 69:75] = obs_batch[:, 63:69]

        return mirror_obs_batch
