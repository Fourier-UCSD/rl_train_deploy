import json
import torch

from rsl_rl.modules import  StateHistoryEncoder
from rsl_rl.storage import StudentStorage






class DAggerAgent:
    def __init__(self, expert_policy,
                 prop_latent_encoder,
                 student_mlp,
                 num_base_obs=48, 
                 num_his_obs = 20,
                 t_steps=20, device='cpu') -> None:
        self.device = device

        self.expert_policy = expert_policy  # trained policy with latent encoder
        self.prop_latent_encoder = prop_latent_encoder  # state history encoder
        self.student_mlp = student_mlp  # to copy action encoder in trained policy
        self.t_steps = t_steps  # state history length in time steps
        self.num_base_obs = num_base_obs - 4  # base observations directly consumed by actor
        self.num_his_obs = num_his_obs
        # copy expert weights for student policy
        # self.student_mlp.actor_critic.load_state_dict(self.expert_policy.policy.actor_critic.state_dict())
        # self.student_mlp.actor.load_state_dict(self.expert_policy.policy.actor.state_dict())
        self.history_len = self.num_his_obs * self.t_steps

        for net_i in [self.expert_policy, self.student_mlp]:
            for param in net_i.parameters():
                param.requires_grad = False

    def get_history_encodeing(self, obs_histoty):
        # NOTE: obs_histoty = [o_{t - t_steps}, ..., o_{t}], contains previous 1 + t_steps observations
        # history_len = self.num_base_obs * self.t_steps
        prop_latent = self.prop_latent_encoder(obs_histoty[:, :self.history_len])
        return prop_latent

    def evaluate(self, obs, obs_history):
        prop_latent = self.get_history_encodeing(obs_history)
        # last_obs = obs_history[:, self.history_len:]  # last obs
        x = torch.cat([prop_latent[:,0:3],
            obs[:, 3:87],    
            prop_latent[:, [3]],
            obs[:, 88:]], dim=-1)
        output = self.student_mlp.act_inference(x)  # student actions
        return output

    def get_expert_action(self, obs):
        output = self.expert_policy.act_inference(obs)
        # expert_latent = self.get_expert_latent(obs)
        # # x = torch.cat((obs[:, :self.num_base_obs], expert_latent), dim=-1)
        # output = self.student_mlp.act_inference_after_encoder(obs[:, :self.num_base_obs], expert_latent)  # expert actions
        return output

    def get_student_action(self, obs, obs_history):
        return self.evaluate(obs, obs_history)

    def get_expert_latent(self, obs):
        with torch.no_grad():
            latent = self.expert_policy.get_encoder_latent(obs)
            return latent


class DAggerTrainer:

    def __init__(self,
                 agent: DAggerAgent,
                 num_base_obs,
                 t_steps,
                 num_learning_epochs=4,
                 num_mini_batches=4,
                 learning_rate=5e-4,
                 schedule='fixed',
                 device='cpu',
                 storage_class="StudentStorage",
                 **kwargs) -> None:

        if kwargs:
            print("DAggerTrainer.__init__ got unexpected arguments, which will be ignored: "
                  + str([key for key in kwargs.keys()]))

        self.device = device

        self.agent = agent
        self.num_base_obs = num_base_obs
        self.t_steps = t_steps
        self.optimizer = torch.optim.Adam([*self.agent.prop_latent_encoder.parameters()], lr=learning_rate)
        self.scheduler = torch.optim.lr_scheduler.StepLR(self.optimizer, step_size=200, gamma=0.1)
        self.loss_fn = torch.nn.MSELoss()

        self.learning_rate = learning_rate
        self.schedule = schedule
        self.num_learning_epochs = num_learning_epochs
        self.num_mini_batches = num_mini_batches

        # Storage
        self.storage_class = storage_class
        self.transition = None  #: Transition: transition data of current control step.
        self.storage = None  #: RolloutStorage: storage for batch data.

    def _init_storage_cfg(self):
        storage_cfg = {
            "device": self.device,
        }

        return storage_cfg

    def init_storage(self,
                 num_envs,
                 num_transitions_per_env,
                 obs_shape,
                 pri_obs_shape,
                 actions_shape,
                 base_obs_shape,
                 prop_latent_dim,
                 t_steps,
                     
                     **kwargs):
        """Initialize storage for batch data.

        Args:
            num_envs (int): number envs.
            num_transitions_per_env (int): number of transitions per env per iteration.
        """
        storage_cfg = self._init_storage_cfg()

        print("storage_cfg: \n",
              json.dumps(storage_cfg, indent=4, sort_keys=True))

        storage_class = eval(self.storage_class)

        self.transition = storage_class.Transition()
        self.storage = storage_class(num_envs,
                                    num_transitions_per_env,
                                    obs_shape,
                                    pri_obs_shape,
                                    actions_shape,
                                    base_obs_shape,
                                    prop_latent_dim,
                                    t_steps,
                                     **storage_cfg)

    def act(self, obs, history_obs):
        self.transition.obs = obs
        self.transition.history_obs = history_obs
        # rollover history 
        # self.storage._update_history(self.transition)
        # self.transition.observations_history = self.storage.latest_history.detach().clone()
        self.transition.expert_latent = self.agent.get_expert_latent(obs).detach()
        self.transition.expert_actions = self.agent.get_expert_action(obs).detach()
        self.transition.predicted_latent = self.agent.get_history_encodeing(history_obs).detach()
        self.transition.actions = self.agent.get_student_action(obs, history_obs).detach()

        return self.transition.actions

    def process_env_step(self):
        # Record the transition
        self.storage.add_transitions(self.transition)
        self.transition.clear()

    def update(self):
        prop_mse = 0
        action_mse = 0
        generator = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)

        for obs_batch, history_obs_batch, expert_latent_batch, expert_action_batch in generator:
            # predicted_prop_latent = self.agent.get_history_encodeing(obs_history_batch)
            
            
            # print("obs: ", obs_batch.shape)
            # print("history_obs: ", history_obs_batch.shape)
            student_latent_batch = self.agent.get_history_encodeing(history_obs_batch)
            student_action_batch = self.agent.get_student_action(obs_batch, history_obs_batch)
            # print("student_action_batch: ", student_action_batch.shape)

            loss_prop = self.loss_fn(expert_latent_batch, student_latent_batch)
            loss_action = self.loss_fn(expert_action_batch, student_action_batch)
            
            #             loss_prop = self.loss_fn(student_latent_batch[:, :self.storage.prop_latent_dim],
            #                          expert_latent_batch[:, :self.storage.prop_latent_dim])
            # loss_action = self.loss_fn(student_action_batch[:, self.storage.prop_latent_dim:],
            #                          expert_action_batch[:, self.storage.prop_latent_dim:])
            loss = 0.4*loss_prop + 0.6*loss_action

            # Gradient step
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            prop_mse += loss_prop.item()
            action_mse += loss_action.item()

        num_updates = self.num_learning_epochs * self.num_mini_batches
        avg_prop_loss = prop_mse / num_updates
        avg_action_mse = action_mse / num_updates
        self.scheduler.step()

        return avg_prop_loss, avg_action_mse

    def clear_storage(self):
        self.storage.clear()
