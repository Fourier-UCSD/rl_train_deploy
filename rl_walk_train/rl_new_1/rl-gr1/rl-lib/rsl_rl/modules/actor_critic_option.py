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


class ActorOption(nn.Module):
    def __init__(self, subpolicy_nns, gnn=None, moe=None):
        print("----------------------------------")
        print("ActorOption")

        super(ActorOption, self).__init__()

        # deploy subpolicy_nns to GPU
        # for subpolicy_nn in self.subpolicy_nns:
        #     subpolicy_nn.to(torch.device("cuda:0"))

        self.num_operations = len(subpolicy_nns)

        # ----------------------------

        self.subpolicy_nns: nn.ModuleList = subpolicy_nns

        # subpolicy nn not need to backprop
        # for subpolicy_nn in self.subpolicy_nns:
        #     for param in subpolicy_nn.parameters():
        #         param.requires_grad = False

        self.subpolicy_nns_model_state_dict = []

        # ----------------------------

        self.gnn: nn.Module = gnn
        self.gnn.to(torch.device("cuda:0"))

        # ----------------------------

        self.subpolicy_moe: nn.Module = moe
        self.subpolicy_moe.to(torch.device("cuda:0"))

    def forward(self, observations):
        # subpolicy output
        subpolicy_actions = []
        for i in range(len(self.subpolicy_nns)):
            subpolicy_nn = self.subpolicy_nns[i]
            subpolicy_action = subpolicy_nn.forward(observations[:, :-self.num_operations])
            subpolicy_actions.append(subpolicy_action)
        subpolicy_outputs = torch.cat(subpolicy_actions, dim=1)

        # gnn output
        gnn_input = torch.cat((observations, subpolicy_outputs), dim=1)
        gnn_output = self.gnn.forward(gnn_input)

        # gnn output is the weight of subpolicy output
        moe_output = gnn_output

        return moe_output


class ActorCriticOption(nn.Module):
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

        super(ActorCriticOption, self).__init__()

        # Policy
        self.actor_subpolicy_nns: nn.ModuleList = nn.ModuleList()
        subpolicy_num_input = num_actor_obs - num_options  # subpolicy not need to observe the option
        subpolicy_num_output = num_actions
        subpolicy_activation = get_activation(activation)

        for i in range(num_options):
            subpolicy_layers = []
            subpolicy_layers.append(nn.Linear(subpolicy_num_input, actor_hidden_dims[0]))
            subpolicy_layers.append(subpolicy_activation)
            for l in range(len(actor_hidden_dims)):
                if l == len(actor_hidden_dims) - 1:
                    subpolicy_layers.append(nn.Linear(actor_hidden_dims[l], subpolicy_num_output))
                else:
                    subpolicy_layers.append(nn.Linear(actor_hidden_dims[l], actor_hidden_dims[l + 1]))
                    subpolicy_layers.append(subpolicy_activation)

            # Jason 2023-09-11:
            # The actor_output_activation can be customized.
            if actor_output_activation is not None:
                subpolicy_layers.append(get_activation(actor_output_activation))

            # add subpolicy to subpolicy_nns
            self.actor_subpolicy_nns.append(nn.Sequential(*subpolicy_layers))

        # GNN
        self.actor_gnn: nn.Module = nn.Module()
        gnn_num_input = num_actor_obs + num_actions * num_options  # num_actor_obs
        gnn_num_output = num_actions  # num_options
        gnn_hidden_dims = [256, 128]
        gnn_activation = get_activation(activation)

        gnn_layers = []
        gnn_layers.append(nn.Linear(gnn_num_input, gnn_hidden_dims[0]))
        gnn_layers.append(gnn_activation)
        for l in range(len(gnn_hidden_dims)):
            if l == len(gnn_hidden_dims) - 1:
                gnn_layers.append(nn.Linear(gnn_hidden_dims[l], gnn_num_output))
            else:
                gnn_layers.append(nn.Linear(gnn_hidden_dims[l], gnn_hidden_dims[l + 1]))
                gnn_layers.append(gnn_activation)

        self.actor_gnn = nn.Sequential(*gnn_layers)

        # MOE
        self.actor_moe: nn.Module = nn.Module()
        moe_num_input = subpolicy_num_input
        moe_num_output = subpolicy_num_output
        moe_hidden_dims = actor_hidden_dims
        moe_activation = get_activation(activation)

        moe_layers = []
        moe_layers.append(nn.Linear(moe_num_input, moe_hidden_dims[0]))
        moe_layers.append(moe_activation)
        for l in range(len(moe_hidden_dims)):
            if l == len(moe_hidden_dims) - 1:
                moe_layers.append(nn.Linear(moe_hidden_dims[l], moe_num_output))
            else:
                moe_layers.append(nn.Linear(moe_hidden_dims[l], moe_hidden_dims[l + 1]))
                moe_layers.append(moe_activation)

        self.actor_moe = nn.Sequential(*moe_layers)

        self.actor = ActorOption(self.actor_subpolicy_nns, self.actor_gnn, self.actor_moe)

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

        critic_num_obs = num_critic_obs
        critic_activation = get_activation(activation)

        # Value function
        critic_layers = []
        if len(critic_hidden_dims) >= 1:
            critic_layers.append(nn.Linear(critic_num_obs, critic_hidden_dims[0]))
            critic_layers.append(critic_activation)
            for l in range(len(critic_hidden_dims)):
                if l == len(critic_hidden_dims) - 1:
                    critic_layers.append(nn.Linear(critic_hidden_dims[l], 1))
                else:
                    critic_layers.append(nn.Linear(critic_hidden_dims[l], critic_hidden_dims[l + 1]))
                    critic_layers.append(critic_activation)
        else:
            critic_layers.append(nn.Linear(critic_num_obs, 1))
            critic_layers.append(critic_activation)

        # Jason 2023-09-11:
        # The critic_output_activation can be customized.
        if critic_output_activation is not None:
            critic_layers.append(get_activation(critic_output_activation))

        self.critic = nn.Sequential(*critic_layers)

        print(f"Actor MLP: {self.actor}")
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

        # copy subpolicy weights and biases to actor and critic
        if subpolicy_paths is not None and subpolicy_use_pretrained_model is True:
            self.copy_subpolicy_weights_and_biases(num_options, subpolicy_paths)

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]

    def copy_subpolicy_weights_and_biases(self, num_options, subpolicy_paths):

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
            print("self.actor.subpolicy_nns[", i, "]: \n", self.actor.subpolicy_nns[i].state_dict().keys())

            # load to actor.subpolicy_nn
            self.actor.subpolicy_nns[i].load_state_dict(model_state_dict)
            self.actor.subpolicy_nns_model_state_dict.append(model_state_dict)

        # print("self.actor.subpolicy_nns_model_state_dict: \n", self.actor.subpolicy_nns_model_state_dict)

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
