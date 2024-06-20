import time, os, statistics
from collections import deque
from datetime import datetime

from torch.utils.tensorboard import SummaryWriter
import torch

from legged_gym import LEGGED_GYM_ROOT_DIR, LEGGED_GYM_ENVS_DIR
from legged_gym.utils import class_to_dict

from rsl_rl.env import *
from rsl_rl.modules import *
from rsl_rl.algorithms import *
from rsl_rl.storage import *


class StudentRunner:
    def __init__(self, env: VecEnv,  train_cfg, log_dir=None, device='cuda:0') :
        
        # train_cfg_dict = class_to_dict(train_cfg)
        self.cfg_all = train_cfg  # config of trained policy
        self.cfg = train_cfg["runner"]  # config of trained policy
        # print(self.cfg )
        # self.alg_cfg = train_cfg.algorithm   # train_cfg["algorithm"]
        self.policy_cfg = train_cfg["policy"]
        self.device = device
        self.env = env

        critic_num_input = self.env.num_obs

        self.num_envs = self.env.num_envs
        num_obs = self.env.num_obs
        actor_num_output = self.env.num_actions
        t_steps = 50

        # teacher network
        # print(train_cfg)
        if self.cfg_all["env"]["use_teacher_network"] is True:
            teacher_network_path = self.cfg_all["env"]["teacher_net_file"].format(LEGGED_GYM_ROOT_DIR=LEGGED_GYM_ROOT_DIR)
            print("Loading teacher network from: ", teacher_network_path)

            self.teacher_network = ActorCritic(num_actor_obs=num_obs,
                                                    num_critic_obs=critic_num_input,
                                                    num_actions=actor_num_output,
                                                    actor_hidden_dims=[512, 256, 128],
                                                    critic_hidden_dims=[512, 256, 128],
                                                    set_std=False).to(self.device)

            expert_policy_dict = self.load(teacher_network_path)
            self.teacher_network.load_state_dict(expert_policy_dict['model_state_dict'])
            # teacher_network_model = torch.load(teacher_network_path, map_location=self.device)
            # teacher_network_model_actor_dict = teacher_network_model["model_state_dict"]
            # self.teacher_network.std = teacher_network_model_actor_dict['std']
            
            self.student_mlp = ActorCritic(num_actor_obs=num_obs,
                                                    num_critic_obs=critic_num_input,
                                                    num_actions=actor_num_output,
                                                    actor_hidden_dims=[512, 256, 128],
                                                    critic_hidden_dims=[512, 256, 128], set_std=False).to(self.device)
            # self.teacher_network.load_state_dict(expert_policy_dict)
            self.student_mlp.load_state_dict(expert_policy_dict['model_state_dict'])
            

            self.teacher_obs_buf = torch.zeros(self.num_envs, num_obs, device=self.device, dtype=torch.float)
            self.teacher_actions = torch.zeros(self.num_envs, num_obs, device=self.device, dtype=torch.float)


            prop_latent_encoder: StateHistoryEncoder_MLP = StateHistoryEncoder_MLP(
            input_size= actor_num_output *3 + 3  ,  # * self.t_steps,
            t_steps=t_steps,
            output_size=4,
        ).to(self.device)

        
        agent = DAggerAgent(
            self.teacher_network,
            prop_latent_encoder,
            self.student_mlp,
            num_base_obs=num_obs,
            num_his_obs =actor_num_output *3 +3,
            t_steps=t_steps,
            device=self.device
        )
        self.dagger = DAggerTrainer(agent=agent,
                                    num_base_obs=num_obs,
                                    t_steps=t_steps,
                                    device=device)

        
        self.num_steps_per_env = self.cfg["num_steps_per_env"]
        self.save_interval = self.cfg["save_interval"]

        # init storage and model
        self.dagger.init_storage(
            self.env.num_envs,
            self.num_steps_per_env,
            self.env.num_obs,
            self.env.num_obs,
            self.env.num_actions,
            self.env.num_actions *3 + 3 ,
            4,
            t_steps
        )
        # _, _ = self.env.reset()
        self.env.reset()

        # Log 
        # self.log_dir = log_dir
        self.get_log_dir()
        self.writer = None
        self.tot_timesteps = 0
        self.tot_time = 0
        self.current_learning_iteration = 0
        
    def learn(self, num_learning_iterations, init_at_random_ep_len=False):
        # initialze writer
        if self.log_dir is not None and self.writer is None:
            self.writer = SummaryWriter(log_dir=self.log_dir, flush_secs=10)
        if init_at_random_ep_len:
            self.env.episode_length_buf = torch.randint_like(self.env.episode_length_buf, high=int(self.env.max_episode_length))
        obs = self.env.get_observations()
        obs = obs.to(self.device)
        encoder_input = self.env.get_encoder_info()

        # observations_history= torch.zeros(self.env.num_envs, self.num_base_obs * self.t_steps)
        # obs_history[:, -self.num_base_obs:] = obs[:, :self.num_base_obs]
        # # obs_history.view(self.env.num_envs, -1)
        # self.policy.eval() # switch to evaluation mode (dropout for example)

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
                    # update observations_historywith latest obs
                    
                    actions = self.dagger.act(obs, encoder_input)
                    obs, pri_obs, rewards, dones, infos,_,_= self.env.step(actions.detach())
                    encoder_input = self.env.get_encoder_info()
                    # print("xxxxxxxx: ", encoder_input.shape)

                    self.dagger.process_env_step()

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

            # backward step
            prop_mse_loss, geom_mse_loss = self.dagger.update()

            # will clear storage here!
            self.dagger.clear_storage()

            stop = time.time()
            learn_time = stop - start
            if self.log_dir is not None:
                self.log(locals())  # locals() -> local variables at current domain
            if it % self.save_interval == 0:
                # TODO: add save method 
                self.save(os.path.join(self.log_dir, 'encoder_{}.pt'.format(it)))
                # self.teacher_network.save(os.path.join(self.log_dir, 'teacher_script_{}.pt'.format(it)))
            ep_infos.clear()

        self.current_learning_iteration += num_learning_iterations
        self.save(os.path.join(self.log_dir, 'model_{}.pt'.format(self.current_learning_iteration)))

    def log(self, locs, width=80, pad=35):
        self.tot_timesteps += self.num_steps_per_env * self.env.num_envs
        self.tot_time += locs['collection_time'] + locs['learn_time']
        iteration_time = locs['collection_time'] + locs['learn_time']

        ep_string = f''
        if locs['ep_infos']:
            for key in locs['ep_infos'][0]:
                infotensor = torch.tensor([], device=self.device)
                for ep_info in locs['ep_infos']:
                    # handle scalar and zero dimensional tensor infos
                    if not isinstance(ep_info[key], torch.Tensor):
                        ep_info[key] = torch.Tensor([ep_info[key]])
                    if len(ep_info[key].shape) == 0:
                        ep_info[key] = ep_info[key].unsqueeze(0)
                    infotensor = torch.cat((infotensor, ep_info[key].to(self.device)))
                value = torch.mean(infotensor)
                self.writer.add_scalar('Episode/' + key, value, locs['it'])
                ep_string += f"""{f'Mean episode {key}:':>{pad}} {value:.4f}\n"""
        ## student std is not consistant with epxert's
        # TODO: investigate parameter importing, DONE: std not used
        mean_std = self.dagger.agent.student_mlp.std.mean()
        # expert_mean_std = self.dagger.agent.expert_policy.policy.std.mean()
        fps = int(self.num_steps_per_env * self.env.num_envs / (locs['collection_time'] + locs['learn_time']))

        self.writer.add_scalar('Loss/prop_mse_loss', locs['prop_mse_loss'], locs['it'])
        self.writer.add_scalar('Loss/geom_mse_loss', locs['geom_mse_loss'], locs['it'])

        self.writer.add_scalar('Loss/learning_rate', self.dagger.learning_rate, locs['it'])
        self.writer.add_scalar('Policy/mean_noise_std', mean_std.item(), locs['it'])
        self.writer.add_scalar('Perf/total_fps', fps, locs['it'])
        self.writer.add_scalar('Perf/collection time', locs['collection_time'], locs['it'])
        self.writer.add_scalar('Perf/learning_time', locs['learn_time'], locs['it'])
        if len(locs['rewbuffer']) > 0:
            self.writer.add_scalar('Train/mean_reward', statistics.mean(locs['rewbuffer']), locs['it'])
            self.writer.add_scalar('Train/mean_episode_length', statistics.mean(locs['lenbuffer']), locs['it'])
            self.writer.add_scalar('Train/mean_reward/time', statistics.mean(locs['rewbuffer']), self.tot_time)
            self.writer.add_scalar('Train/mean_episode_length/time', statistics.mean(locs['lenbuffer']), self.tot_time)

        str = f" \033[1m Learning iteration {locs['it']}/{self.current_learning_iteration + locs['num_learning_iterations']} \033[0m "

        if len(locs['rewbuffer']) > 0:
            log_string = (f"""{'#' * width}\n"""
                          f"""{str.center(width, ' ')}\n\n"""
                          f"""{'Computation:':>{pad}} {fps:.0f} steps/s (collection: {locs[
                              'collection_time']:.3f}s, learning {locs['learn_time']:.3f}s)\n"""
                          f"""{'Prop latent MSE loss:':>{pad}} {locs['prop_mse_loss']:.4f}\n"""
                          f"""{'Geom latent MSE loss:':>{pad}} {locs['geom_mse_loss']:.4f}\n"""
                          f"""{'Mean action noise std:':>{pad}} {mean_std.item():.2f}\n"""
                          f"""{'Mean reward:':>{pad}} {statistics.mean(locs['rewbuffer']):.2f}\n"""
                          f"""{'Mean episode length:':>{pad}} {statistics.mean(locs['lenbuffer']):.2f}\n""")
            #   f"""{'Mean reward/step:':>{pad}} {locs['mean_reward']:.2f}\n"""
            #   f"""{'Mean episode length/episode:':>{pad}} {locs['mean_trajectory_length']:.2f}\n""")
        else:
            log_string = (f"""{'#' * width}\n"""
                          f"""{str.center(width, ' ')}\n\n"""
                          f"""{'Computation:':>{pad}} {fps:.0f} steps/s (collection: {locs[
                              'collection_time']:.3f}s, learning {locs['learn_time']:.3f}s)\n"""
                          f"""{'Prop latent MSE loss:':>{pad}} {locs['prop_mse_loss']:.4f}\n"""
                          f"""{'Geom latent MSE loss:':>{pad}} {locs['geom_mse_loss']:.4f}\n"""
                          f"""{'Mean action noise std:':>{pad}} {mean_std.item():.2f}\n""")
            #   f"""{'Mean reward/step:':>{pad}} {locs['mean_reward']:.2f}\n"""
            #   f"""{'Mean episode length/episode:':>{pad}} {locs['mean_trajectory_length']:.2f}\n""")

        log_string += ep_string
        log_string += (f"""{'-' * width}\n"""
                       f"""{'Total timesteps:':>{pad}} {self.tot_timesteps}\n"""
                       f"""{'Iteration time:':>{pad}} {iteration_time:.2f}s\n"""
                       f"""{'Total time:':>{pad}} {self.tot_time:.2f}s\n"""
                       f"""{'ETA:':>{pad}} {self.tot_time / (locs['it'] + 1) * (
                               locs['num_learning_iterations'] - locs['it']):.1f}s\n""")
        print(log_string)

    def save(self, path, infos=None):
        
        script_model = torch.jit.script(self.dagger.agent.prop_latent_encoder)
        script_model.save(path)
        # torch.save({
        #     'model_state_dict': self.dagger.agent.prop_latent_encoder.state_dict(),
        #     'optimizer_state_dict': self.dagger.optimizer.state_dict(),
        #     'iter': self.current_learning_iteration,
        #     'infos': infos,
        # }, path)

    def get_log_dir(self):
        # if self.cfg['log_root'] is not None:
        #     log_root = self.cfg['log_root']
        # else:
        log_root = 'default'

        if log_root == "default":
            log_root = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', "_".join([self.cfg["experiment_name"], 'with_adaptation']))
            log_dir = os.path.join(log_root, datetime.now().strftime('%b%d_%H-%M-%S') + '_' + 'adaptation')
        elif log_root is None:
            log_dir = None
        elif log_root == "synology":
            # get home directory full path /home/<user>
            home = os.path.expanduser("~")
            print(f"Home directory: {home}")
            # set log_root to ~/Sync/logs/<experiment_name>
            log_root = os.path.join(home, 'SynologyDrive', 'logs', "_".join([self.cfg["experiment_name"], 'with_adaptation']))
            # judge whether ~/Sync/logs exists
            if not os.path.exists(log_root):
                os.makedirs(log_root)
            # log to ~/Sync/logs/<experiment_name>
            log_dir = os.path.join(log_root, datetime.now().strftime('%b%d_%H-%M-%S') + '_' + 'adaptation')
        else:
            log_dir = os.path.join(log_root, datetime.now().strftime('%b%d_%H-%M-%S') + '_' + 'adaptation')
        print(f"Logging to : {log_dir}")

        self.log_dir = log_dir


    def get_inference_policy(self, device=None):
            
        self.alg.actor_critic.eval()  # switch to evaluation mode (dropout for example)
        if device is not None:
            self.alg.actor_critic.to(device)
        return self.alg.actor_critic.act_inference    
    
    def load(self, path):
        loaded_dict = torch.load(path)

        return loaded_dict