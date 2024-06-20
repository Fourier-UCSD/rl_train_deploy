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

from .mlp import MLP
from .moe import MoE


class ActorCriticMoE(nn.Module):
    is_recurrent = False
    subpolicy_nns: nn.ModuleList

    def __init__(self,
                 num_actor_obs,
                 num_critic_obs,
                 num_actions,
                 actor_hidden_dims=[256, 256, 256],
                 critic_hidden_dims=[256, 256, 256],
                 activation='elu',
                 init_noise_std=1.0,
                 fixed_std=False,
                 actor_output_activation=None,
                 critic_output_activation=None,
                 num_options=1,
                 subpolicy_paths=None,
                 subpolicy_use_pretrained_model=False,
                 **kwargs):

        print("----------------------------------")
        print("ActorCritic")

        if kwargs:
            print("ActorCritic.__init__ got unexpected arguments, which will be ignored: " + str(
                [key for key in kwargs.keys()]))

        super(ActorCriticMoE, self).__init__()

        # Policy
        self.actor_subpolicy_nns: nn.ModuleList = nn.ModuleList()
        subpolicy_num_input = num_actor_obs  # - num_options  # subpolicy not need to observe the option
        subpolicy_num_output = num_actions
        subpolicy_hidden_dims = actor_hidden_dims
        subpolicy_activation = activation

        for i in range(num_options):
            self.actor_subpolicy_nns.append(MLP(subpolicy_num_input,
                                                subpolicy_num_output,
                                                subpolicy_hidden_dims,
                                                subpolicy_activation,
                                                norm="none"))

        # copy subpolicy weights and biases to subpolicy_nns
        if subpolicy_paths is not None and subpolicy_use_pretrained_model is True:
            self.copy_subpolicy_weights_and_biases(self.actor_subpolicy_nns, num_options, subpolicy_paths)

        # MoE
        moe_num_input = num_actor_obs
        moe_num_output = subpolicy_num_output

        self.actor = MoE(self.actor_subpolicy_nns, moe_num_input, moe_num_output, noisy_gating=False, k=num_options)

        print(f"Actor MLP: {self.actor}")

        # actor_layers = []
        # if len(actor_hidden_dims) >= 1:
        #     actor_layers.append(nn.Linear(mlp_input_dim_a, actor_hidden_dims[0]))
        #     actor_layers.append(activation)
        #     for l in range(len(actor_hidden_dims)):
        #         if l == len(actor_hidden_dims) - 1:
        #             actor_layers.append(nn.Linear(actor_hidden_dims[l], num_actions))
        #         else:
        #             actor_layers.append(nn.Linear(actor_hidden_dims[l], actor_hidden_dims[l + 1]))
        #             actor_layers.append(activation)
        # else:
        #     actor_layers.append(nn.Linear(mlp_input_dim_a, num_actions))
        #     actor_layers.append(activation)
        #
        # # Jason 2023-09-11:
        # # The actor_output_activation can be customized.
        # if actor_output_activation is not None:
        #     actor_layers.append(get_activation(actor_output_activation))
        #
        # self.actor = nn.Sequential(*actor_layers)

        critic_num_input = num_critic_obs
        critic_num_output = 1
        critic_activation = activation

        # Value function
        self.critic = MLP(critic_num_input,
                          critic_num_output,
                          critic_hidden_dims,
                          critic_activation,
                          norm="none")

        print(f"Critic MLP: {self.critic}")

        # Action noise
        self.fixed_std = fixed_std
        std = init_noise_std * torch.ones(num_actions)
        self.std = torch.tensor(std) if fixed_std else nn.Parameter(std)
        self.distribution = None

        # disable args validation for speedup
        Normal.set_default_validate_args = False

        # seems that we get better performance without init
        # self.init_memory_weights(self.memory_a, 0.001, 0.)
        # self.init_memory_weights(self.memory_c, 0.001, 0.)

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]

    def copy_subpolicy_weights_and_biases(self, subpolicy_nns, num_options, subpolicy_paths):

        for i in range(num_options):
            subpolicy_file = torch.load(subpolicy_paths[i])

            # actor.0.weight -> submodule.0.weight
            # actor.0.bias -> submodule.0.bias
            # actor.2.weight -> submodule.2.weight
            # actor.2.bias -> submodule.2.bias
            # actor.4.weight -> submodule.4.weight
            # actor.4.bias -> submodule.4.bias
            # actor.6.weight -> submodule.6.weight
            # actor.6.bias -> submodule.6.bias
            for key in list(subpolicy_file['model_state_dict'].keys()):
                # if key.startswith('actor'):
                #     subpolicy_file['model_state_dict'][key.replace('actor', 'submodule')] = (
                #         subpolicy_file['model_state_dict'].pop(key))

                if key.startswith('actor'):
                    subpolicy_file['model_state_dict'][key.replace('actor.', '')] = (
                        subpolicy_file['model_state_dict'].pop(key))

            # delete critic weights
            for key in list(subpolicy_file['model_state_dict'].keys()):
                if key.startswith('critic'):
                    subpolicy_file['model_state_dict'].pop(key)

            # delete std
            for key in list(subpolicy_file['model_state_dict'].keys()):
                if key.startswith('std'):
                    subpolicy_file['model_state_dict'].pop(key)

            # print model_dict
            model_state_dict = subpolicy_file['model_state_dict']
            # print("model_state_dict: \n", model_state_dict)

            # print model keys
            print(subpolicy_paths[i], "\n", subpolicy_file['model_state_dict'].keys())

            # print actor.subpolicy_nn keys
            print("subpolicy_nns[", i, "]: \n", subpolicy_nns[i].state_dict().keys())

            # load to actor.subpolicy_nn
            subpolicy_nns[i].load_state_dict(model_state_dict)

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    @property
    def action_mean(self):
        return self.distribution.mean

    @property
    def action_std(self):
        return self.distribution.stddev

    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, observations):
        mean = self.actor(observations)
        std = self.std.to(mean.device)
        self.distribution = Normal(mean, mean * 0. + std)

    def act(self, observations, **kwargs):
        self.update_distribution(observations)
        return self.distribution.sample()

    def get_actions_log_prob(self, actions):
        actions_log_prob = self.distribution.log_prob(actions).sum(dim=-1)
        return actions_log_prob

    def act_inference(self, observations):
        actions_mean = self.actor(observations)
        return actions_mean

    def evaluate(self, critic_observations, **kwargs):
        value = self.critic(critic_observations)
        return value


def get_activation(act_name):
    if act_name == "elu":
        return nn.ELU()
    elif act_name == "selu":
        return nn.SELU()
    elif act_name == "relu":
        return nn.ReLU()
    elif act_name == "crelu":
        return nn.ReLU()
    elif act_name == "lrelu":
        return nn.LeakyReLU()
    elif act_name == "tanh":
        return nn.Tanh()
    elif act_name == "sigmoid":
        return nn.Sigmoid()
    else:
        print("invalid activation function!")
        return None
