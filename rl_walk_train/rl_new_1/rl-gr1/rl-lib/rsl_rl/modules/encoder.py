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


class Encoder(nn.Module):
    is_recurrent = False

    def __init__(self,
                 num_encoder_obs,
                 num_encoder_out,
                 hidden_dims=[256, 256],
                 activation='elu',
                 **kwargs):

        print("----------------------------------")
        print("Encoder")

        if kwargs:
            print("Encoder.__init__ got unexpected arguments, which will be ignored: "
                  + str([key for key in kwargs.keys()]))
        super(Encoder, self).__init__()

        self.num_encoder_input = num_encoder_obs
        self.num_encoder_output = num_encoder_out

        activation = get_activation(activation)

        mlp_input_dim_e = num_encoder_obs
        mlp_output_dim_e = num_encoder_out

        # Encoder
        encoder_layers = []
        encoder_layers.append(nn.Linear(mlp_input_dim_e, hidden_dims[0]))
        encoder_layers.append(activation)
        for l in range(len(hidden_dims)):
            if l == len(hidden_dims) - 1:
                encoder_layers.append(nn.Linear(hidden_dims[l], mlp_output_dim_e))
            else:
                encoder_layers.append(nn.Linear(hidden_dims[l], hidden_dims[l + 1]))
                encoder_layers.append(activation)
        self.encoder = nn.Sequential(*encoder_layers)

        print(f"Encoder MLP: {self.encoder}")

        # disable args validation for speedup
        Normal.set_default_validate_args = False

        # seems that we get better performance without init
        # self.init_memory_weights(self.memory_a, 0.001, 0.)
        # self.init_memory_weights(self.memory_c, 0.001, 0.)

        # encoder output
        self.coded = None

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    def update_encoder(self, observations):
        coded = self.encoder(observations)
        self.coded = coded

    def encode(self, encoder_observations, **kwargs):
        self.update_encoder(encoder_observations)
        return self.coded


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
