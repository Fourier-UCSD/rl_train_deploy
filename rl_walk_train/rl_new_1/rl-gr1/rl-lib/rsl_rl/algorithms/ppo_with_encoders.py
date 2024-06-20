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

from rsl_rl.modules import ActorCritic, Encoder
from rsl_rl.storage import RolloutStorageWithEncoders


class PPOWithEncoders:
    actor_critic: ActorCritic
    encoder_1: Encoder
    encoder_2: Encoder
    encoder_3: Encoder
    encoder_4: Encoder

    def __init__(self,
                 actor_critic,
                 encoder_1,
                 encoder_2,
                 encoder_3,
                 encoder_4,
                 num_learning_epochs=1,
                 num_mini_batches=1,
                 clip_param=0.2,
                 gamma=0.998,
                 lam=0.95,
                 value_loss_coef=1.0,
                 entropy_coef=0.0,
                 learning_rate=1e-3,
                 encoder_learning_rate=1e-4,
                 max_grad_norm=1.0,
                 use_clipped_value_loss=True,
                 schedule="fixed",
                 desired_kl=0.01,
                 device='cpu',
                 ):

        print("----------------------------------")
        print("PPOWithEncoder")

        # encoder_1 components
        self.encoder_1 = encoder_1
        self.encoder_1_network_update_count = 0
        self.encoder_1_network_use_count = 20 * 0
        self.encoder_1_network_use_flag = False
        self.encoder_1_learning_rate_init = encoder_learning_rate
        self.encoder_1_learning_rate = encoder_learning_rate
        self.encoder_1_optimizer = optim.Adam(self.encoder_1.parameters(), lr=encoder_learning_rate)

        # encoder_2 components
        self.encoder_2 = encoder_2
        self.encoder_2_network_update_count = 0
        self.encoder_2_network_use_count = 20 * 0
        self.encoder_2_network_use_flag = False
        self.encoder_2_learning_rate_init = encoder_learning_rate
        self.encoder_2_learning_rate = encoder_learning_rate
        self.encoder_2_optimizer = optim.Adam(self.encoder_2.parameters(), lr=encoder_learning_rate)

        # encoder_3 components
        self.encoder_3 = encoder_3
        self.encoder_3_network_update_count = 0
        self.encoder_3_network_use_count = 20 * 0
        self.encoder_3_network_use_flag = False
        self.encoder_3_learning_rate_init = encoder_learning_rate
        self.encoder_3_learning_rate = encoder_learning_rate
        self.encoder_3_optimizer = optim.Adam(self.encoder_3.parameters(), lr=encoder_learning_rate)

        # encoder_4 components
        self.encoder_4 = encoder_4
        self.encoder_4_network_update_count = 0
        self.encoder_4_network_use_count = 20 * 0
        self.encoder_4_network_use_flag = False
        self.encoder_4_learning_rate_init = encoder_learning_rate
        self.encoder_4_learning_rate = encoder_learning_rate
        self.encoder_4_optimizer = optim.Adam(self.encoder_4.parameters(), lr=encoder_learning_rate)

        # learning components
        self.device = device

        self.desired_kl = desired_kl
        self.schedule = schedule

        # PPO components
        self.actor_critic = actor_critic
        self.actor_critic.to(self.device)
        self.storage = None  # initialized later
        self.learning_rate = learning_rate
        self.optimizer = optim.Adam(self.actor_critic.parameters(), lr=learning_rate)
        self.transition = RolloutStorageWithEncoders.Transition()

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

    def init_storage(self,
                     num_envs,
                     num_transitions_per_env,
                     actor_obs_shape,
                     critic_obs_shape,
                     action_shape,
                     encoder_1_obs_shape,
                     encoder_1_out_shape,
                     encoder_2_obs_shape,
                     encoder_2_out_shape,
                     encoder_3_obs_shape,
                     encoder_3_out_shape,
                     encoder_4_obs_shape,
                     encoder_4_out_shape):
        self.storage = RolloutStorageWithEncoders(num_envs,
                                                  num_transitions_per_env,
                                                  actor_obs_shape,
                                                  critic_obs_shape,
                                                  action_shape,
                                                  encoder_1_obs_shape,
                                                  encoder_1_out_shape,
                                                  encoder_2_obs_shape,
                                                  encoder_2_out_shape,
                                                  encoder_3_obs_shape,
                                                  encoder_3_out_shape,
                                                  encoder_4_obs_shape,
                                                  encoder_4_out_shape,
                                                  self.device)

    def test_mode(self):
        self.actor_critic.test()

    def train_mode(self):
        self.actor_critic.train()

    def act(self,
            obs, critic_obs,
            obs_encoder_1=None, out_encoder_1=None,
            obs_encoder_2=None, out_encoder_2=None,
            obs_encoder_3=None, out_encoder_3=None,
            obs_encoder_4=None, out_encoder_4=None):

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

        # encoder
        self.transition.encoder_1_observations = obs_encoder_1
        self.transition.encoder_1_outputs = out_encoder_1
        self.transition.encoder_2_observations = obs_encoder_2
        self.transition.encoder_2_outputs = out_encoder_2
        self.transition.encoder_3_observations = obs_encoder_3
        self.transition.encoder_3_outputs = out_encoder_3
        self.transition.encoder_4_observations = obs_encoder_4
        self.transition.encoder_4_outputs = out_encoder_4

        return self.transition.actions

    def process_env_step(self, rewards, dones, infos):
        self.transition.rewards = rewards.clone()
        self.transition.dones = dones

        # Bootstrapping on time outs
        if 'time_outs' in infos:
            self.transition.rewards += self.gamma * torch.squeeze(
                self.transition.values * infos['time_outs'].unsqueeze(1).to(self.device), 1)

        # Record the transition
        self.storage.add_transitions(self.transition)
        self.transition.clear()
        self.actor_critic.reset(dones)

    def compute_returns(self, last_critic_obs):
        last_values = self.actor_critic.evaluate(last_critic_obs).detach()
        self.storage.compute_returns(last_values, self.gamma, self.lam)

    def update(self):
        mean_value_loss = 0
        mean_surrogate_loss = 0
        mean_encoder_1_loss = 0
        mean_encoder_2_loss = 0
        mean_encoder_3_loss = 0
        mean_encoder_4_loss = 0

        if self.actor_critic.is_recurrent:
            generator = self.storage.reccurent_mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)
        else:
            generator = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)

        for obs_batch, critic_obs_batch, actions_batch, target_values_batch, \
                advantages_batch, returns_batch, old_actions_log_prob_batch, \
                old_mu_batch, old_sigma_batch, hid_states_batch, masks_batch, \
                obs_encoder_1, out_encoder_1, \
                obs_encoder_2, out_encoder_2, \
                obs_encoder_3, out_encoder_3, \
                obs_encoder_4, out_encoder_4 \
                in generator:

            # encoder_1 update --------------------------------------------------

            self.encoder_1_network_update_count = self.encoder_1_network_update_count + 1

            # 当结果较为稳定时，开始降低学习率 (防止过拟合)
            if self.encoder_1_network_update_count % (20 * 100) == 0:
                self.encoder_1_learning_rate = self.encoder_1_learning_rate / 2.0

                if self.encoder_1_learning_rate < 1e-5:
                    self.encoder_1_learning_rate = 1e-5

                for param_group in self.encoder_1_optimizer.param_groups:
                    param_group['lr'] = self.encoder_1_learning_rate

            encoder_1_obs = obs_encoder_1
            encoder_1_target = out_encoder_1

            self.encoder_1.update_encoder(encoder_1_obs)
            encoder_1_batch = self.encoder_1.coded
            self.encoder_1_optimizer.zero_grad()
            encoder_1_criteon = nn.MSELoss()
            encoder_1_loss = encoder_1_criteon(encoder_1_batch, encoder_1_target)
            encoder_1_loss.backward()  # 注意：这里不要使用retain_graph=True!!!
            self.encoder_1_optimizer.step()

            # encoder_1 update --------------------------------------------------

            # encoder_2 update --------------------------------------------------

            self.encoder_2_network_update_count = self.encoder_2_network_update_count + 1

            # 当结果较为稳定时，开始降低学习率 (防止过拟合)
            if self.encoder_2_network_update_count % (20 * 100) == 0:
                self.encoder_2_learning_rate = self.encoder_2_learning_rate / 2.0

                if self.encoder_2_learning_rate < 1e-5:
                    self.encoder_2_learning_rate = 1e-5

                for param_group in self.encoder_2_optimizer.param_groups:
                    param_group['lr'] = self.encoder_2_learning_rate

            encoder_2_obs = obs_encoder_2
            encoder_2_target = out_encoder_2

            self.encoder_2.update_encoder(encoder_2_obs)
            encoder_2_batch = self.encoder_2.coded
            self.encoder_2_optimizer.zero_grad()
            encoder_2_criteon = nn.MSELoss()
            encoder_2_loss = encoder_2_criteon(encoder_2_batch, encoder_2_target)
            encoder_2_loss.backward()  # 注意：这里不要使用retain_graph=True!!!
            self.encoder_2_optimizer.step()

            # encoder_2 update --------------------------------------------------

            # encoder_3 update --------------------------------------------------

            self.encoder_3_network_update_count = self.encoder_3_network_update_count + 1

            # 当结果较为稳定时，开始降低学习率 (防止过拟合)
            if self.encoder_3_network_update_count % (20 * 100) == 0:
                self.encoder_3_learning_rate = self.encoder_3_learning_rate / 2.0

                if self.encoder_3_learning_rate < 1e-5:
                    self.encoder_3_learning_rate = 1e-5

                for param_group in self.encoder_3_optimizer.param_groups:
                    param_group['lr'] = self.encoder_3_learning_rate

            encoder_3_obs = obs_encoder_3
            encoder_3_target = out_encoder_3

            self.encoder_3.update_encoder(encoder_3_obs)
            encoder_3_batch = self.encoder_3.coded
            self.encoder_3_optimizer.zero_grad()
            encoder_3_criteon = nn.MSELoss()
            encoder_3_loss = encoder_3_criteon(encoder_3_batch, encoder_3_target)
            encoder_3_loss.backward()  # 注意：这里不要使用retain_graph=True!!!
            self.encoder_3_optimizer.step()

            # encoder_3 update --------------------------------------------------

            # encoder_4 update --------------------------------------------------

            self.encoder_4_network_update_count = self.encoder_4_network_update_count + 1

            # 当结果较为稳定时，开始降低学习率 (防止过拟合)
            if self.encoder_4_network_update_count % (20 * 100) == 0:
                self.encoder_4_learning_rate = self.encoder_4_learning_rate / 2.0

                if self.encoder_4_learning_rate < 1e-5:
                    self.encoder_4_learning_rate = 1e-5

                for param_group in self.encoder_4_optimizer.param_groups:
                    param_group['lr'] = self.encoder_4_learning_rate

            encoder_4_obs = obs_encoder_4
            encoder_4_target = out_encoder_4

            self.encoder_4.update_encoder(encoder_4_obs)
            encoder_4_batch = self.encoder_4.coded
            self.encoder_4_optimizer.zero_grad()
            encoder_4_criteon = nn.MSELoss()
            encoder_4_loss = encoder_4_criteon(encoder_4_batch, encoder_4_target)
            encoder_4_loss.backward()  # 注意：这里不要使用retain_graph=True!!!
            self.encoder_4_optimizer.step()

            # encoder_4 update --------------------------------------------------

            # actor - critic update ---------------------------------------------

            # actor action distribution
            self.actor_critic.act(obs_batch,
                                  masks=masks_batch,
                                  hidden_states=hid_states_batch[0])
            actions_log_prob_batch = self.actor_critic.get_actions_log_prob(actions_batch)
            value_batch = self.actor_critic.evaluate(critic_obs_batch,
                                                     masks=masks_batch,
                                                     hidden_states=hid_states_batch[1])
            mu_batch = self.actor_critic.action_mean
            sigma_batch = self.actor_critic.action_std
            entropy_batch = self.actor_critic.entropy

            # KL
            if self.desired_kl != None and self.schedule == 'adaptive':
                with torch.inference_mode():
                    kl = torch.sum(
                        torch.log(sigma_batch / old_sigma_batch + 1.e-5)
                        + (torch.square(old_sigma_batch) + torch.square(old_mu_batch - mu_batch))
                        / (2.0 * torch.square(sigma_batch)) - 0.5, axis=-1)
                    kl_mean = torch.mean(kl)

                    if kl_mean > self.desired_kl * 2.0:
                        self.learning_rate = max(1.0e-5, self.learning_rate / 1.5)
                    elif kl_mean < self.desired_kl / 2.0 and kl_mean > 0.0:
                        self.learning_rate = min(1.0e-2, self.learning_rate * 1.5)
                    else:
                        pass

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

            loss = surrogate_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy_batch.mean()

            # Gradient step
            self.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.actor_critic.parameters(), self.max_grad_norm)
            self.optimizer.step()

            # actor - critic update ---------------------------------------------

            mean_value_loss += value_loss.item()
            mean_surrogate_loss += surrogate_loss.item()
            mean_encoder_1_loss += encoder_1_loss.item()
            mean_encoder_2_loss += encoder_2_loss.item()
            mean_encoder_3_loss += encoder_3_loss.item()
            mean_encoder_4_loss += encoder_4_loss.item()

        num_updates = self.num_learning_epochs * self.num_mini_batches
        mean_value_loss /= num_updates
        mean_surrogate_loss /= num_updates
        mean_encoder_1_loss /= num_updates
        mean_encoder_2_loss /= num_updates
        self.storage.clear()

        return mean_value_loss, mean_surrogate_loss, \
            mean_encoder_1_loss, mean_encoder_2_loss, mean_encoder_3_loss, mean_encoder_4_loss
