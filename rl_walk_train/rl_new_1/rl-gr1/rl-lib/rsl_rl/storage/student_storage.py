import torch

from .rollout_storage_student import RolloutStorageStudent


class StudentStorage(RolloutStorageStudent):
    """Class stores data of all transitions per iteration.
    """

    class Transistion(RolloutStorageStudent.Transition):
        """Transition data of current control step
        """

        def __init__(self) -> None:
            """ Tansition quantities at time step `t`.
            """
            super().__init__()

            self.observations_history = None
            self.expert_actions = None
            self.expert_latent = None
            self.predicted_latent = None

    def __init__(self,
                 num_envs,
                 num_transitions_per_env,
                 obs_shape,
                 pri_obs_shape,
                 actions_shape,
                 base_obs_shape,
                 prop_latent_dim,
                 t_steps,
                 device='cpu') -> None:
        """Initialize storage for observations
        """
        super().__init__(num_envs,
                         num_transitions_per_env,
                         obs_shape,
                         pri_obs_shape,
                         actions_shape,
                         device)



        # RMA
        self.base_obs_shape = obs_shape
        self.prop_latent_dim = prop_latent_dim
        self.latent_shape = prop_latent_dim 
        self.num_history_obs = base_obs_shape * t_steps
        # record (t_steps + 1) obs
        # self.latest_history = torch.zeros(num_envs, self.num_history_obs + base_obs_shape[0]).to(device)
        
                # Core
        # self.observations_history = torch.zeros(num_transitions_per_env, num_envs, self.num_history_obs, device=device)
        
        print("xxx: ", self.num_history_obs)
        self.history_obs = torch.zeros(num_transitions_per_env, num_envs, self.num_history_obs, device=device)
        self.obs = torch.zeros(num_transitions_per_env, num_envs, self.base_obs_shape, device=device)
        self.expert_actions = torch.zeros(num_transitions_per_env, num_envs, actions_shape, device=self.device)
        self.expert_latent = torch.zeros(num_transitions_per_env, num_envs, self.latent_shape, device=self.device)
        self.predicted_latent = torch.zeros(num_transitions_per_env, num_envs, self.latent_shape, device=self.device)

    def add_transitions(self, transition: Transistion):
        """Add transition data to storage
        
        Args:
            transition (Transistion): transition data of current control step.
        """
        # super()._add_transition(transition)
        # self._add_transitions(transition)
        self.obs[self.step].copy_(transition.obs)  # -> batch expert latent
        self.history_obs[self.step].copy_(transition.history_obs.view(self.num_envs, -1))  # -> batch predicted latent
        self.actions[self.step].copy_(transition.actions)
        self.expert_actions[self.step].copy_(transition.expert_actions)
        self.expert_latent[self.step].copy_(transition.expert_latent)
        self.predicted_latent[self.step].copy_(transition.predicted_latent)

        if self.step >= self.num_transitions_per_env:
            raise AssertionError("Rollout buffer overflow")
        
        self.step += 1

    # def _update_history(self, transition: Transistion):
    #     """Update history of observations
        
    #     Args:
    #         transition (Transistion): transition data of current control step.
    #     """
    #     self.latest_history = torch.cat((
    #         self.latest_history[:, self.base_obs_shape[0]:],
    #         transition.obs[:, :self.base_obs_shape[0]],
    #     ), dim=-1).to(self.device)



    def mini_batch_generator(self, num_mini_batches, num_epochs=8):
        """Generate mini batches for training
        
        Args:
            num_mini_batches (int): number of mini batches
            num_epochs (int, optional): number of epochs. Defaults to 8.

        Yields:
            [torch.Tensor]: [batch data of each entry]
        """
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches
        indices = torch.randperm(num_mini_batches * mini_batch_size, requires_grad=False, device=self.device)

        obs = self.obs.flatten(0, 1)
        history_obs = self.history_obs.flatten(0, 1)

        student_actions = self.actions.flatten(0, 1)
        expert_actions = self.expert_actions.flatten(0, 1)
        expert_latent = self.expert_latent.flatten(0, 1)
        predicted_latent = self.predicted_latent.flatten(0, 1)

        for epoch in range(num_epochs):
            for i in range(num_mini_batches):
                start = i * mini_batch_size
                end = (i + 1) * mini_batch_size
                batch_idx = indices[start:end]


                obs_batch = obs[batch_idx]
                history_obs_batch = history_obs[batch_idx]
                

                expert_latent_batch = expert_latent[batch_idx]
                expert_actions_batch = expert_actions[batch_idx]
                yield obs_batch, history_obs_batch, expert_latent_batch, expert_actions_batch
