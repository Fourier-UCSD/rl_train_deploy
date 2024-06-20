import numpy as np
import torch
import torch.nn as nn


class actor_network(nn.Module):
    def __init__(self,
                 num_actor_obs,
                 num_critic_obs,
                 num_actions,
                 actor_hidden_dims=[256, 256, 256],
                 critic_hidden_dims=[256, 256, 256],
                 activation='elu',
                 init_noise_std=1.0,
                 **kwargs):
        if kwargs:
            print("ActorCritic.__init__ got unexpected arguments, which will be ignored: " + str([key for key in kwargs.keys()]))
        super(actor_network, self).__init__()

        activation = get_activation(activation)

        mlp_input_dim_a = num_actor_obs
        mlp_input_dim_c = num_critic_obs

        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(mlp_input_dim_a, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for l in range(len(actor_hidden_dims)):
            if l == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], actor_hidden_dims[l + 1]))
                actor_layers.append(activation)
        self.actor = nn.Sequential(*actor_layers)

        # Value function
        critic_layers = []
        critic_layers.append(nn.Linear(mlp_input_dim_c, critic_hidden_dims[0]))
        critic_layers.append(activation)
        for l in range(len(critic_hidden_dims)):
            if l == len(critic_hidden_dims) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], critic_hidden_dims[l + 1]))
                critic_layers.append(activation)
        self.critic = nn.Sequential(*critic_layers)

        print(f"Actor MLP: {self.actor}")
        print(f"Critic MLP: {self.critic}")
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))

    def act_inference(self, observations):
        actions_mean = self.actor(observations)
        return actions_mean


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


class SonnyPolicy():
    def __init__(self):
        self.num_actor_obs = 169
        self.num_critic_obs = 169
        self.num_actions = 12
        self.actor_hidden_dim = [512, 256, 128]
        self.critic_hidden_dim = [512, 256, 128]
        self.activation = 'elu'
        self.init_noise_std = 1.0
        self.rl_network = actor_network(self.num_actor_obs,
                                        self.num_critic_obs,
                                        self.num_actions,
                                        self.actor_hidden_dim,
                                        self.critic_hidden_dim,
                                        self.activation,
                                        self.init_noise_std)
        path = '/home/hgone/Documents/Python/ETH-ISSAC/legged_gym/logs/rough_sonny/7月25_14-06-09_/model_8000.pt'
        # print(path)
        loaded_dict = torch.load(path, map_location=torch.device('cpu'))
        self.rl_network.load_state_dict(loaded_dict['model_state_dict'])
        self.obs_scales_lin_vel = 2.0;
        self.obs_scales_ang_vel = 0.25;
        self.command_scales = [self.obs_scales_lin_vel, self.obs_scales_lin_vel, self.obs_scales_ang_vel];
        self.obs_scales_dof_pos = 1.0;
        self.obs_scales_dof_vel = 0.05;
        self.default_dof_pos = np.zeros(12)
        self.default_dof_pos = [0.0, 0.0, -0.5236, 1.0472, -0.5236, 0.0,
                                0.0, 0.0, -0.5236, 1.0472, -0.5236, 0.0, ]
        self.action_last = np.zeros(12)
        self.height_scales = 5.0
        # Action noise
        # env_cfg, train_cfg = task_registry.get_cfgs(name="sonny")
        # env_cfg.env.num_envs = min(env_cfg.env.num_envs, 1)
        # env_cfg.env.num_actions = 12
        # env_cfg.env.num_observations = 169
        # env_cfg.noise.add_noise = False

        # env = task_registry.make_sonny_env(name="sonny",env_cfg=env_cfg, physics_engine=gymapi.SIM_PHYSX,sim_device='cpu')

        # train_cfg.runner.resume = True
        # train_cfg.runner.load_run = "7月25_14-06-09_"
        # train_cfg.runner.checkpoint = -1
        # ppo_runner, train_cfg = task_registry.make_alg_runner(env=env,name="sonny",train_cfg=train_cfg)
        # self.policy = ppo_runner.get_inference_policy(device="cpu")
        print("init trained PPO model")

    def forward(self, obs):
        obs_1 = torch.tensor(obs, dtype=torch.float, device='cpu', requires_grad=False)
        obs_2 = torch.clip(obs_1, -100.0, 100.0)
        self.rl_network.eval()  # switch to evaluation mode (dropout for example)
        self.rl_network.to('cpu')
        actions = self.rl_network.act_inference(obs_2.detach())
        return actions.detach().numpy()

    def forward2(self, x, y):
        obs = np.zeros(169)
        obs[0] = x
        obs[1] = y
        action2 = self.forward(obs)
        return action2[0]

    def forward3(self, base_lin_vel_x, base_lin_vel_y, base_lin_vel_z,
                 base_ang_vel_x, base_ang_vel_y, base_ang_vel_z,
                 projected_gravity_x, projected_gravity_y, projected_gravity_z,
                 commands_x, commands_y, commands_z,
                 dof_pos_0, dof_pos_1, dof_pos_2, dof_pos_3, dof_pos_4, dof_pos_5,
                 dof_pos_6, dof_pos_7, dof_pos_8, dof_pos_9, dof_pos_10, dof_pos_11,
                 dof_vel_0, dof_vel_1, dof_vel_2, dof_vel_3, dof_vel_4, dof_vel_5,
                 dof_vel_6, dof_vel_7, dof_vel_8, dof_vel_9, dof_vel_10, dof_vel_11,
                 height
                 ):
        obs = np.zeros(169)
        obs[0] = base_lin_vel_x * self.obs_scales_lin_vel
        obs[1] = base_lin_vel_y * self.obs_scales_lin_vel
        obs[2] = base_lin_vel_z * self.obs_scales_lin_vel

        obs[3] = base_ang_vel_x * self.obs_scales_ang_vel
        obs[4] = base_ang_vel_y * self.obs_scales_ang_vel
        obs[5] = base_ang_vel_z * self.obs_scales_ang_vel

        obs[6] = projected_gravity_x
        obs[7] = projected_gravity_y
        obs[8] = projected_gravity_z

        obs[9] = commands_x * self.command_scales[0]
        obs[10] = commands_y * self.command_scales[1]
        obs[11] = commands_z * self.command_scales[2]

        obs[12] = (dof_pos_0 - self.default_dof_pos[0]) * self.obs_scales_dof_pos
        obs[13] = (dof_pos_1 - self.default_dof_pos[1]) * self.obs_scales_dof_pos
        obs[14] = (dof_pos_2 - self.default_dof_pos[2]) * self.obs_scales_dof_pos
        obs[15] = (dof_pos_3 - self.default_dof_pos[3]) * self.obs_scales_dof_pos
        obs[16] = (dof_pos_4 - self.default_dof_pos[4]) * self.obs_scales_dof_pos
        obs[17] = (dof_pos_5 - self.default_dof_pos[5]) * self.obs_scales_dof_pos
        obs[18] = (dof_pos_6 - self.default_dof_pos[6]) * self.obs_scales_dof_pos
        obs[19] = (dof_pos_7 - self.default_dof_pos[7]) * self.obs_scales_dof_pos
        obs[20] = (dof_pos_8 - self.default_dof_pos[8]) * self.obs_scales_dof_pos
        obs[21] = (dof_pos_9 - self.default_dof_pos[9]) * self.obs_scales_dof_pos
        obs[22] = (dof_pos_10 - self.default_dof_pos[10]) * self.obs_scales_dof_pos
        obs[23] = (dof_pos_11 - self.default_dof_pos[11]) * self.obs_scales_dof_pos

        obs[24] = dof_vel_0 * self.obs_scales_dof_vel
        obs[25] = dof_vel_1 * self.obs_scales_dof_vel
        obs[26] = dof_vel_2 * self.obs_scales_dof_vel
        obs[27] = dof_vel_3 * self.obs_scales_dof_vel
        obs[28] = dof_vel_4 * self.obs_scales_dof_vel
        obs[29] = dof_vel_5 * self.obs_scales_dof_vel
        obs[30] = dof_vel_6 * self.obs_scales_dof_vel
        obs[31] = dof_vel_7 * self.obs_scales_dof_vel
        obs[32] = dof_vel_8 * self.obs_scales_dof_vel
        obs[33] = dof_vel_9 * self.obs_scales_dof_vel
        obs[34] = dof_vel_10 * self.obs_scales_dof_vel
        obs[35] = dof_vel_11 * self.obs_scales_dof_vel

        obs[36:48] = self.action_last

        obs[48:169] = np.clip((height - 0.5), -1, 1) * self.height_scales * np.ones(121)
        self.action_last = self.forward(obs)
        # print("obs:")
        # print(obs)
        # print("action:")
        print(self.action_last)
        return self.action_last


class RL_interface():
    def __init__(self):
        self.obj = SonnyPolicy()
        print("Hello init success!")

    def SetName(self, name="test"):
        self._name = name
        print("set name: " + self._name)

    def PrintName(self):
        print("Print name: " + self._name)

    def Add(self, x, y):
        res = x + y
        print("hello add: " + str(res))
        return res

    def arrayAdd(self, x):
        res = x[0] + x[1]
        print("ArrayAdd: " + str(res[0]))
        return res

    def Interface(self, base_lin_vel_x, base_lin_vel_y, base_lin_vel_z,
                  base_ang_vel_x, base_ang_vel_y, base_ang_vel_z,
                  projected_gravity_x, projected_gravity_y, projected_gravity_z,
                  commands_x, commands_y, commands_z,
                  dof_pos_0, dof_pos_1, dof_pos_2, dof_pos_3, dof_pos_4, dof_pos_5,
                  dof_pos_6, dof_pos_7, dof_pos_8, dof_pos_9, dof_pos_10, dof_pos_11,
                  dof_vel_0, dof_vel_1, dof_vel_2, dof_vel_3, dof_vel_4, dof_vel_5,
                  dof_vel_6, dof_vel_7, dof_vel_8, dof_vel_9, dof_vel_10, dof_vel_11,
                  height):
        # a = np.zeros(169)
        # a[0] = x
        # a[1] = y
        # obs = th.tensor(a, dtype=th.float, device='cpu', requires_grad=False)
        action = self.obj.forward3(base_lin_vel_x, base_lin_vel_y, base_lin_vel_z,
                                   base_ang_vel_x, base_ang_vel_y, base_ang_vel_z,
                                   projected_gravity_x, projected_gravity_y, projected_gravity_z,
                                   commands_x, commands_y, commands_z,
                                   dof_pos_0, dof_pos_1, dof_pos_2, dof_pos_3, dof_pos_4, dof_pos_5,
                                   dof_pos_6, dof_pos_7, dof_pos_8, dof_pos_9, dof_pos_10, dof_pos_11,
                                   dof_vel_0, dof_vel_1, dof_vel_2, dof_vel_3, dof_vel_4, dof_vel_5,
                                   dof_vel_6, dof_vel_7, dof_vel_8, dof_vel_9, dof_vel_10, dof_vel_11,
                                   height)
        # print("Interface: ")
        # print(obs)
        # print(action)

        res = np.array(action, dtype=np.float32)
        # print('res')
        # print(res.dtype)
        return res


def main():
    h = RL_interface()
    res = h.Interface(0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, -0.0, 0.0, -0.0, 0.0,
                      0.0, 0.0, -0.0, 0.0, -0.0, 0.0,
                      0.0, 0.0, -0.0, 0.0, -0.0, 0.0,
                      0.0, 0.0, -0.0, 0.0, -0.0, 0.0,
                      0.0)
    print(res)


if __name__ == '__main__':
    main()
