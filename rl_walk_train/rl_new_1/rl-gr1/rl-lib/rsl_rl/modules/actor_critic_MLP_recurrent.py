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

import numpy as np

import torch
import torch.nn as nn
from torch.distributions import Normal
from torch.nn.modules import rnn
from .actor_critic_recurrent import ActorCriticRecurrent, get_activation

class ActorCriticMLPRecurrent(ActorCriticRecurrent):
    is_recurrent = True
    def __init__(self,  num_actor_obs,
                        num_critic_obs,
                        num_actions,
                        actor_hidden_dims=[256, 256, 256],
                        critic_hidden_dims=[256, 256, 256],
                        actor_hidden_dims_pre=[256, 256, 256],
                        critic_hidden_dims_pre=[256, 256, 256],
                        activation='elu',
                        rnn_type='lstm',
                        rnn_hidden_size=256,
                        rnn_num_layers=1,
                        init_noise_std=1.0,
                        fixed_std=False,
                        **kwargs):
        if kwargs:
            print("ActorCriticMLPRecurrent.__init__ got unexpected arguments, which will be ignored: " + str(kwargs.keys()),)

        super().__init__(num_actor_obs=actor_hidden_dims_pre[-1],
                         num_critic_obs=critic_hidden_dims_pre[-1],
                         num_actions=num_actions,
                         actor_hidden_dims=actor_hidden_dims,
                         critic_hidden_dims=critic_hidden_dims,
                         activation=activation,
                         rnn_type=rnn_type,
                         rnn_hidden_size=rnn_hidden_size,
                         rnn_num_layers=rnn_num_layers,
                         init_noise_std=init_noise_std,
                         fixed_std=fixed_std)

        activation = get_activation(activation)
        mlp_input_dim_a = num_actor_obs
        mlp_input_dim_c = num_critic_obs
        # Policy
        actor_layers_MLPRecurrent = []
        if len(actor_hidden_dims_pre) >= 1:
            for l in range(len(actor_hidden_dims_pre)):
                if l == 0:
                    actor_layers_MLPRecurrent.append(nn.Linear(mlp_input_dim_a, actor_hidden_dims_pre[0]))
                    actor_layers_MLPRecurrent.append(activation)
                else:
                    actor_layers_MLPRecurrent.append(nn.Linear(actor_hidden_dims_pre[l - 1], actor_hidden_dims_pre[l]))
                    actor_layers_MLPRecurrent.append(activation)
                # actor_layers_MLPRecurrent.append(nn.Linear(actor_hidden_dims_pre[-1], rnn_hidden_size))

        self.actor_MLPRecurrent = nn.Sequential(*actor_layers_MLPRecurrent)

        # Value function
        critic_layers_MLPRecurrent = []
        if len(critic_hidden_dims_pre) >= 1:
            for l in range(len(critic_hidden_dims_pre)):
                if l == 0:
                    critic_layers_MLPRecurrent.append(nn.Linear(mlp_input_dim_c, critic_hidden_dims_pre[0]))
                    critic_layers_MLPRecurrent.append(activation)                
                else:
                    critic_layers_MLPRecurrent.append(nn.Linear(critic_hidden_dims_pre[l - 1], critic_hidden_dims_pre[l]))
                    critic_layers_MLPRecurrent.append(activation)
                # critic_layers_MLPRecurrent.append(nn.Linear(actor_hidden_dims_pre[-1], rnn_hidden_size))

        self.critic_MLPRecurrent = nn.Sequential(*critic_layers_MLPRecurrent)

        print(f"Actor MLP_pre: {self.actor_MLPRecurrent}")
        print(f"Critic MLP_pre: {self.critic_MLPRecurrent}")
        # self.obs_mean = 0.0
        # self.obs_std = 1.0
    # def states_normalization(self, obs):
    #     obs_norm = (obs - self.obs_mean)/self.obs_std
    #     return obs_norm
    
    def reset(self, dones=None):
        return super().reset()

    def act(self, observations, masks=None, hidden_states=None):
        # observations_norm = self.states_normalization(observations)
        observations_norm = observations
        mean_MLPRecurrent = self.actor_MLPRecurrent(observations_norm)
        return super().act(mean_MLPRecurrent, masks=masks, hidden_states=hidden_states)
    def act_for_mirror(self, observations, masks=None, hidden_states=None):
        # observations_norm = self.states_normalization(observations)
        observations_norm = observations
        mean_MLPRecurrent = self.actor_MLPRecurrent(observations_norm)
        return super().act_for_mirror(mean_MLPRecurrent, masks=masks, hidden_states=hidden_states)

    def act_inference(self, observations):
        # print("observations MLprecurrent.shape: ")
        # print(observations.shape)
        # observations_norm = self.states_normalization(observations)
        observations_norm = observations
        mean_MLPRecurrent = self.actor_MLPRecurrent(observations_norm)
        return super().act_inference(mean_MLPRecurrent)

    def evaluate(self, critic_observations, masks=None, hidden_states=None):
        # critic_observations_norm = self.states_normalization(critic_observations)
        critic_observations_norm = critic_observations
        critic_mean_MLPRecurrent = self.critic_MLPRecurrent(critic_observations_norm)
        return super().evaluate(critic_mean_MLPRecurrent, masks=masks, hidden_states=hidden_states)
