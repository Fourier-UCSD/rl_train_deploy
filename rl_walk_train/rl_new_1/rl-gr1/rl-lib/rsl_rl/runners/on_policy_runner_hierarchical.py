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

import time
import os
from collections import deque
import statistics

from torch.utils.tensorboard import SummaryWriter
import torch
import torch.nn as nn

from rsl_rl.algorithms import PPOHierarchical
from rsl_rl.modules import ActorCritic
from rsl_rl.modules import SubModule
from rsl_rl.env import VecEnv

from .on_policy_runner import OnPolicyRunner


class OnPolicyRunnerHierarchical(OnPolicyRunner):

    def __init__(self, env: VecEnv, train_cfg, log_dir=None, device='cpu'):

        super().__init__(env, train_cfg, log_dir, device)

    def init(self, env, train_cfg, log_dir, device):

        print("----------------------------------")
        print("OnPolicyRunnerHierarchicalWithMirror")

        self.cfg = train_cfg["runner"]
        self.alg_cfg = train_cfg["algorithm"]
        self.policy_cfg = train_cfg["policy"]
        self.submodule_cfg = train_cfg["submodule"]

        print("self.cfg: \n", self.cfg)
        print("self.alg_cfg: \n", self.alg_cfg)
        print("self.policy_cfg: \n", self.policy_cfg)
        print("self.submodule_cfg: \n", self.submodule_cfg)

        self.device = device
        self.env = env

        print("self.device: \n", self.device)
        print("self.env: \n", self.env)

        # Submodules
        submodule_class = eval(self.cfg["submodule_class_name"])
        submodule_num_input = self.submodule_cfg["submodule_num_input"]
        submodule_num_output = self.submodule_cfg["submodule_num_output"]
        submodule_hidden_dims = self.submodule_cfg["submodule_hidden_dims"]
        submodule_activation = self.submodule_cfg["submodule_activation"]
        submodule_paths = self.submodule_cfg["submodule_paths"]

        self.submodules: nn.ModuleList = nn.ModuleList()
        submodule_counts = len(submodule_paths)
        for i in range(submodule_counts):
            self.submodules.append(submodule_class(submodule_num_input,
                                                   submodule_num_output,
                                                   submodule_hidden_dims,
                                                   submodule_activation).to(self.device))

        for i in range(submodule_counts):
            submodule = self.submodules[i]
            model_file = torch.load(submodule_paths[i])

            # print model keys
            print(model_file['model_state_dict'].keys())

            # actor.0.weight -> submodule.0.weight
            # actor.0.bias -> submodule.0.bias
            # actor.2.weight -> submodule.2.weight
            # actor.2.bias -> submodule.2.bias
            # actor.4.weight -> submodule.4.weight
            # actor.4.bias -> submodule.4.bias
            # actor.6.weight -> submodule.6.weight
            # actor.6.bias -> submodule.6.bias
            for key in list(model_file['model_state_dict'].keys()):
                if key.startswith('actor'):
                    model_file['model_state_dict'][key.replace('actor', 'submodule')] = (
                        model_file['model_state_dict'].pop(key))

            # delete critic weights
            for key in list(model_file['model_state_dict'].keys()):
                if key.startswith('critic'):
                    model_file['model_state_dict'].pop(key)

            # delete std
            for key in list(model_file['model_state_dict'].keys()):
                if key.startswith('std'):
                    model_file['model_state_dict'].pop(key)

            # print model keys
            print(submodule_paths[i], "\n", model_file['model_state_dict'].keys())

            model_dict = model_file['model_state_dict']
            submodule.load_state_dict(model_dict)
            submodule.eval()  # sets the module in evaluation mode.

        self.submodules_output_dims = [submodule.num_output for submodule in self.submodules]
        print("Submodules output dims: \n", self.submodules_output_dims)

        # ActorCritic
        actor_num_input = self.env.num_obs

        # Jason: 将 actor 上一次的输出作为下一次的输入，因为 actor 的输出是 submodule counts 维的，所以需要加上 submodule counts
        # add submodule counts to actor obs
        # actor_num_obs += submodule_counts

        if self.env.num_privileged_obs is not None:
            critic_num_input = self.env.num_privileged_obs
        else:
            critic_num_input = self.env.num_obs

        actor_num_output = self.env.num_actions

        print("actor_num_input: \n", actor_num_input)
        print("critic_num_input: \n", critic_num_input)
        print("actor_num_output: \n", actor_num_output)

        # Jason: 将 actor 上一次的输出作为下一次的输入，因为 actor 的输出是 submodule counts 维的，所以需要加上 submodule counts
        # num_critic_obs += submodule_counts  # add submodule counts to critic obs

        actor_critic_class = eval(self.cfg["policy_class_name"])

        if self.alg_cfg["mix_type"] == "submodules_output_multiply_ratio_v1":
            actor_num_output = submodule_counts
            self.actor_critic: ActorCritic = actor_critic_class(actor_num_input,
                                                                critic_num_input,
                                                                actor_num_output,
                                                                **self.policy_cfg).to(self.device)
        elif self.alg_cfg["mix_type"] == "submodules_output_multiply_ratio_v2":
            actor_num_output = submodule_counts
            self.actor_critic: ActorCritic = actor_critic_class(actor_num_input,
                                                                critic_num_input,
                                                                actor_num_output,
                                                                **self.policy_cfg).to(self.device)
        elif self.alg_cfg["mix_type"] == "submodules_output_multiply_ratio_v3":
            actor_num_output = submodule_counts
            self.actor_critic: ActorCritic = actor_critic_class(actor_num_input,
                                                                critic_num_input,
                                                                actor_num_output,
                                                                **self.policy_cfg).to(self.device)
        elif self.alg_cfg["mix_type"] == "submodules_output_to_ratio_v1":
            actor_num_input += submodule_counts * submodule_num_output
            actor_num_output = submodule_counts
            self.actor_critic: ActorCritic = actor_critic_class(actor_num_input,
                                                                critic_num_input,
                                                                actor_num_output,
                                                                **self.policy_cfg).to(self.device)
        elif self.alg_cfg["mix_type"] == "submodules_output_to_nn":
            actor_num_input += submodule_counts * submodule_num_output
            actor_num_output = self.env.num_actions
            self.actor_critic: ActorCritic = actor_critic_class(actor_num_input,
                                                                critic_num_input,
                                                                actor_num_output,
                                                                **self.policy_cfg).to(self.device)
        else:
            self.actor_critic: ActorCritic = actor_critic_class(actor_num_input,
                                                                critic_num_input,
                                                                actor_num_output,
                                                                **self.policy_cfg).to(self.device)

        # PPO
        alg_class = eval(self.cfg["algorithm_class_name"])  # PPOWithMirror
        self.alg: PPOHierarchical = alg_class(self.actor_critic, self.submodules, device=self.device, **self.alg_cfg)

        self.num_steps_per_env = self.cfg["num_steps_per_env"]
        self.save_interval = self.cfg["save_interval"]

        # init storage and model
        self.alg.init_storage(self.env.num_envs,
                              self.num_steps_per_env,
                              [actor_num_input],
                              [critic_num_input],
                              [actor_num_output])

        # Log
        self.log_dir = log_dir
        self.writer = None
        self.tot_timesteps = 0
        self.tot_time = 0
        self.current_learning_iteration = 0

        _, _ = self.env.reset()

    def learn(self, num_learning_iterations, init_at_random_ep_len=False):
        # initialize writer
        if self.log_dir is not None and self.writer is None:
            self.writer = SummaryWriter(log_dir=self.log_dir, flush_secs=10)
        if init_at_random_ep_len:
            self.env.episode_length_buf = torch.randint_like(self.env.episode_length_buf,
                                                             high=int(self.env.max_episode_length))
        env_actor_obs = self.env.get_observations()
        env_privileged_obs = self.env.get_privileged_observations()
        env_critic_obs = env_privileged_obs if env_privileged_obs is not None else env_actor_obs

        # print("env_actor_obs.shape: ", env_actor_obs.shape)
        # print("env_critic_obs.shape: ", env_critic_obs.shape)

        env_actor_output = self.alg.act(env_actor_obs, env_critic_obs)
        env_actor_obs, env_critic_obs = env_actor_obs.to(self.device), env_critic_obs.to(self.device)

        # switch to train mode (for dropout for example)
        self.alg.actor_critic.train()

        ep_infos = []
        rewbuffer = deque(maxlen=100)
        lenbuffer = deque(maxlen=100)
        cur_reward_sum = torch.zeros(self.env.num_envs, dtype=torch.float, device=self.device)
        cur_episode_length = torch.zeros(self.env.num_envs, dtype=torch.float, device=self.device)

        tot_iter = self.current_learning_iteration + num_learning_iterations
        for it in range(self.current_learning_iteration, tot_iter):
            start = time.time()

            # Rollout
            with torch.inference_mode():
                for i in range(self.num_steps_per_env):

                    # print("env_actor_obs.shape: ", env_actor_obs.shape)
                    # print("env_critic_obs.shape: ", env_critic_obs.shape)

                    # calculate ppo act
                    env_actor_output = self.alg.act(env_actor_obs, env_critic_obs)

                    # env do step
                    env_actor_obs, env_privileged_obs, rewards, dones, infos, _, _ = self.env.step(env_actor_output)
                    env_critic_obs = env_privileged_obs if env_privileged_obs is not None else env_actor_obs

                    # get obs
                    env_actor_obs, env_critic_obs, rewards, dones = \
                        env_actor_obs.to(self.device), env_critic_obs.to(self.device), \
                            rewards.to(self.device), dones.to(self.device)

                    # process env step
                    self.alg.process_env_step(rewards, dones, infos)

                    if self.log_dir is not None:
                        # Book keeping
                        if 'episode' in infos:
                            ep_infos.append(infos['episode'])
                        cur_reward_sum += rewards
                        cur_episode_length += 1
                        new_ids = (dones > 0).nonzero(as_tuple=False)
                        rewbuffer.extend(cur_reward_sum[new_ids][:, 0].cpu().numpy().tolist())
                        lenbuffer.extend(cur_episode_length[new_ids][:, 0].cpu().numpy().tolist())
                        cur_reward_sum[new_ids] = 0
                        cur_episode_length[new_ids] = 0

                stop = time.time()
                collection_time = stop - start

                # Learning step
                start = stop
                self.alg.compute_returns(env_critic_obs)

            mean_value_loss, mean_surrogate_loss = self.alg.update()
            stop = time.time()
            learn_time = stop - start
            if self.log_dir is not None:
                self.log(locals())
            if it % self.save_interval == 0:
                self.save(os.path.join(self.log_dir, 'model_{}.pt'.format(it)))
            ep_infos.clear()

        self.current_learning_iteration += num_learning_iterations
        self.save(os.path.join(self.log_dir, 'model_{}.pt'.format(self.current_learning_iteration)))

    def get_inference_policy(self, device=None):
        self.alg.actor_critic.eval()  # switch to evaluation mode (dropout for example)
        if device is not None:
            self.alg.actor_critic.to(device)
        # return self.alg.actor_critic.act_inference
        return self.alg.act_inference
