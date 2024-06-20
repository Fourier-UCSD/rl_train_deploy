"""Shiwen @ Dec. 2023

RMA modules implementaion.
Paper link: https://arxiv.org/abs/2107.04034.
"""
import numpy as np
import torch
import torch.nn as nn
from torch.nn.utils import weight_norm
import torch.nn.functional as F

from rsl_rl.utils.utils import *


class EncodeActionMLP(nn.Module):
    """A module that encodes observations and applies a multi-layer perceptron (MLP) to generate actions.
    
    Attributes:
        encoder (MLPEncoder): encoder encodes observations not available to partially observable case.
        action_mlp (nn.Module): MLP to generate actions.
    """

    def __init__(self, num_obs, actor_num_output, action_mlp_dims=[256, 256, 256], activation='elu', output_activation=None,
                 prop_dim=1, prop_latent_dim=1, geom_dim=0, geom_latent_dim=1, encoder_mlp_dims=[128, 128], **kwargs) -> None:
        """
        Initialize the EncodeActionMLP class.

        Args:
            num_obs (int): Number of observations.
            actor_num_output (int): Number of actions.
            activation (str): Activation function name for the encoder and MLP layers.
            output_activation (str, optional): Activation function name for the output layer. Defaults to None.
            prop_dim (int, optional): Dimension of the property input. Defaults to 1.
            prop_latent_dim (int, optional): Dimension of the property latent space. Defaults to 1.
            geom_dim (int, optional): Dimension of the geometry input. Defaults to 0.
            geom_latent_dim (int, optional): Dimension of the geometry latent space. Defaults to 1.
            **kwargs: Additional keyword arguments (ignored).

        Returns:
            None
        """

        if kwargs:
            print("EncodeActionMLP.__init__ got unexpected arguments, which will be ignored: " + str(kwargs.keys()), )
        super().__init__()
        self.num_obs = num_obs
        self.actor_num_output = actor_num_output
        self.activation_fn = get_activation_fn(activation)
        if output_activation is not None:
            self.output_activation_fn = get_activation_fn(output_activation)
        else:
            self.output_activation_fn = None
        self.prop_dim = prop_dim
        self.geom_dim = geom_dim
        self.prop_latent_dim = prop_latent_dim
        self.geom_latent_dim = geom_latent_dim
        self.num_base_obs = self.num_obs - self.prop_dim - self.geom_dim
        self.action_mlp_input_size = self.num_base_obs + self.prop_latent_dim + self.geom_latent_dim

        self.prop_encoder, scale_prop = build_mlp(input_size=self.prop_dim,
                                                  output_size=self.prop_latent_dim,
                                                  hidden_dims=action_mlp_dims,
                                                  activation_fn=self.activation_fn,
                                                  output_activation_fn=self.output_activation_fn)
        # self.init_weights(self.prop_encoder, scale_prop)
        if geom_dim > 0:
            self.geom_encoder, scale_geom = build_mlp(input_size=self.geom_dim,
                                                      output_size=self.geom_latent_dim,
                                                      hidden_dims=encoder_mlp_dims,
                                                      activation_fn=self.activation_fn,
                                                      output_activation_fn=self.output_activation_fn)
            # self.init_weights(self.geom_encoder, scale_geom)
        else:
            raise IOError("Not implemented geom_dim")
        # Creating the action encoder.
        self.action_mlp, scale_action_mlp = build_mlp(input_size=self.action_mlp_input_size,
                                                      output_size=self.actor_num_output,
                                                      hidden_dims=encoder_mlp_dims,
                                                      activation_fn=self.activation_fn,
                                                      output_activation_fn=self.output_activation_fn)
        # self.init_weights(self.action_mlp, scale_action_mlp)

        self.input_shape = [num_obs]  #: input dimension of network.
        self.output_shape = [actor_num_output]  #: output dimension of network.

        print(f"Property encoder: {self.prop_encoder}")
        print(f"Geometry encoder: {self.geom_encoder}")
        print(f"Action MLP: {self.action_mlp}")

    @staticmethod
    def init_weights(sequential, scales):
        """Initialize network weights 
        
        Args:
            sequential (nn.Module): sequential model of network
            scale   (list): initial weights of model

        Returns:   
            None
        """
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]

    def forward(self, x: torch.Tensor, **kwargs):
        prop_latent = self.prop_encoder(x[:, self.num_base_obs:self.num_base_obs + self.prop_dim])
        geom_latent = self.geom_encoder(x[:, self.num_base_obs + self.prop_dim:])
        input_a = torch.cat((x[:, :self.num_base_obs], prop_latent, geom_latent), dim=-1)
        return self.action_mlp(input_a)
        # batch_size = obs.shape[0]
        # latent = self.encoder(obs.reshape([-1, self.num_obs])[:, self.num_base_obs:])
        # input_a = torch.cat((obs.reshape([-1, self.num_obs])[:, :self.num_base_obs], latent), dim=-1)
        # return self.action_mlp(torch.squeeze(input_a.reshape([batch_size, -1, self.action_mlp_input_size])))

    def evaluate(self, obs: torch.Tensor):
        raise NotImplementedError

   
    
class StateHistoryEncoder_MLP(nn.Module):
    def __init__(self, input_size,  t_steps=10, output_size = 4, hidden_sizes = [4096, 2048, 1024]):
        super(StateHistoryEncoder_MLP, self).__init__()
        self.hidden_layers = nn.ModuleList()
        self.hidden_sizes = hidden_sizes
        self.input_size = input_size*t_steps
        # 添加隐藏层
        for hidden_size in hidden_sizes:
            self.hidden_layers.append(nn.Linear(self.input_size, hidden_size))
            self.hidden_layers.append(nn.ReLU())
            self.input_size = hidden_size
        
        self.output_layer = nn.Linear(hidden_sizes[-1], output_size)

    def forward(self, x):
        batchsize = x.shape[0]
        x =x.view(batchsize,-1)
        for layer in self.hidden_layers:
            x = layer(x)
        x = self.output_layer(x)
        return x





class StateHistoryEncoder(torch.nn.Module):
    use_state_history = True

    def __init__(self,
                 input_size,
                 t_steps=10,
                 output_size=8,  # latent_dim
                 mlp_hidden_dims=[128, 128],
                 cnn_input_shape=32,
                 #  cnn_hidden_dims=[
                 #      [32, 32, 8, 4],
                 #      [32, 32, 5, 1],
                 #      [32, 32, 5, 1],
                 #  ],
                 mlp_activation='tanh',
                 cnn_activation='lrelu',
                 **kwargs) -> None:
        if kwargs:
            print(
                "StateHistoryEncoder.__init__ got unexpected arguments, which will be ignored: " + str([key for key in kwargs.keys()])
            )
        super(StateHistoryEncoder, self).__init__()

        mlp_activation_fn = get_activation_fn(mlp_activation)
        cnn_activation_fn = get_activation_fn(cnn_activation)
        # Encoder layers
        encoder_layers = []
        encoder_layers.append(
            nn.Linear(input_size, mlp_hidden_dims[0])
        )
        encoder_layers.append(mlp_activation_fn)
        for l in range(len(mlp_hidden_dims)):
            if l == len(mlp_hidden_dims) - 1:
                encoder_layers.append(
                    nn.Linear(mlp_hidden_dims[l], cnn_input_shape)
                )
            else:
                encoder_layers.append(
                    nn.Linear(mlp_hidden_dims[l], mlp_hidden_dims[l + 1])
                )
                encoder_layers.append(mlp_activation_fn)
        self.encoder = nn.Sequential(*encoder_layers)

        # cnn1d layers 
        conv_layers = []
        if t_steps == 50:
            self.conv1d = nn.Sequential(
                nn.Conv1d(in_channels=32, out_channels=32, kernel_size=8, stride=4), cnn_activation_fn,
                nn.Conv1d(in_channels=32, out_channels=32, kernel_size=5, stride=1), cnn_activation_fn,
                nn.Conv1d(in_channels=32, out_channels=32, kernel_size=5, stride=1), cnn_activation_fn,
                nn.Flatten(),
            )
        elif t_steps == 20:
            self.conv1d = nn.Sequential(
                nn.Conv1d(in_channels=32, out_channels=32, kernel_size=6, stride=2), cnn_activation_fn,
                nn.Conv1d(in_channels=32, out_channels=32, kernel_size=4, stride=2), cnn_activation_fn,
                nn.Flatten(),
            )
        elif t_steps == 10:
            self.conv1d = nn.Sequential(
                nn.Conv1d(in_channels=32, out_channels=32, kernel_size=4, stride=2), cnn_activation_fn,
                nn.Conv1d(in_channels=32, out_channels=32, kernel_size=2, stride=1), cnn_activation_fn,
                nn.Flatten()
            )
        else:
            raise NotImplementedError()

        if False:
            conv_layers.append(
                nn.Conv1d(cnn_hidden_dims[0][0],
                          cnn_hidden_dims[0][1],
                          cnn_hidden_dims[0][2],
                          cnn_hidden_dims[0][3])
            )
            conv_layers.append(cnn_activation_fn)
            for l in range(len(cnn_hidden_dims)):
                if l == len(cnn_hidden_dims) - 1:
                    conv_layers.append(
                        nn.Conv1d(cnn_hidden_dims[l][0],
                                  cnn_hidden_dims[l][1],
                                  cnn_hidden_dims[l][2],
                                  cnn_hidden_dims[l][3])
                    )
                else:
                    conv_layers.append(
                        nn.Conv1d(cnn_hidden_dims[l][0],
                                  cnn_hidden_dims[l][1],
                                  cnn_hidden_dims[l][2],
                                  cnn_hidden_dims[l][3])
                    )
                    conv_layers.append(cnn_activation_fn)
            conv_layers.append(nn.Flatten())  # flatten CNN output
            self.conv1d = nn.Sequential(*conv_layers)
        # print(f"State history encoder: {self.conv1d}")

        # project flattened CNN output to desired dimension
        self.linear_output = nn.Sequential(
            # nn.Linear(cnn_hidden_dims[-1][1], output_shape), mlp_activation_fn
            nn.Linear(32 * 3, output_size)
        )
        self.input_shape = [input_size * t_steps]
        self.output_shape = [output_size]
        self.t_steps = t_steps

    def forward(self, x):
        batch_size = x.shape[0]
        T = self.t_steps
        x1 = self.encoder(x.reshape([batch_size * T, -1]))
        x2 = self.conv1d(x1.reshape([batch_size, -1, T]))
        output = self.linear_output(x2)
        return output


class Flatten(nn.Module):
    """A flatten module flats input tensors to shape of [-1, 1]
    """

    def forward(self, x: torch.tensor):
        return x.view(x.size(0), -1)


class Chomp1d(nn.Module):
    def __init__(self, chomp_size):
        super(Chomp1d, self).__init__()
        self.chomp_size = chomp_size

    def forward(self, x):
        return x[:, :, :-self.chomp_size].contiguous()


class TemporalBlock1(nn.Module):
    def __init__(self, n_inputs, n_outputs, kernel_size, stride, dilation, padding, dropout=0.2):
        super(TemporalBlock1, self).__init__()
        self.conv1 = weight_norm(nn.Conv1d(n_inputs, n_outputs, kernel_size,
                                           stride=1, padding=padding, dilation=1))
        self.chomp1 = Chomp1d(padding)
        self.relu1 = nn.ReLU()
        self.dropout1 = nn.Dropout(dropout)

        self.conv2 = weight_norm(nn.Conv1d(n_outputs, n_outputs, kernel_size,
                                           stride=2, padding=padding, dilation=1))
        self.chomp2 = Chomp1d(padding)
        self.relu2 = nn.ReLU()
        self.dropout2 = nn.Dropout(dropout)

        self.net = nn.Sequential(self.conv1, self.chomp1, self.relu1, self.dropout1,
                                 self.conv2, self.chomp2, self.relu2, self.dropout2)
        # self.downsample = nn.Conv1d(n_inputs, n_outputs, 1) if n_inputs != n_outputs else None
        self.downsample =  None
        self.relu = nn.ReLU()
        self.init_weights()
    def init_weights(self):
        self.conv1.weight.data.normal_(0, 0.01)
        self.conv2.weight.data.normal_(0, 0.01)
        if self.downsample is not None:
            self.downsample.weight.data.normal_(0, 0.01)
    def forward(self, x):
        out = self.net(x)
        res = x if self.downsample is None else self.downsample(x)
        return self.relu(out + res)


class TemporalBlock2(nn.Module):
    def __init__(self, n_inputs, n_outputs, kernel_size, stride, dilation, padding, dropout=0.2):
        super(TemporalBlock2, self).__init__()
        self.conv1 = weight_norm(nn.Conv1d(n_inputs, n_outputs, kernel_size,
                                           stride=1, padding=padding, dilation=2))
        self.chomp1 = Chomp1d(padding)
        self.relu1 = nn.ReLU()
        self.dropout1 = nn.Dropout(dropout)

        self.conv2 = weight_norm(nn.Conv1d(n_outputs, n_outputs, kernel_size,
                                           stride=2, padding=padding, dilation=1))
        self.chomp2 = Chomp1d(padding)
        self.relu2 = nn.ReLU()
        self.dropout2 = nn.Dropout(dropout)

        self.net = nn.Sequential(self.conv1, self.chomp1, self.relu1, self.dropout1,
                                 self.conv2, self.chomp2, self.relu2, self.dropout2)
        self.downsample =  None
        # self.downsample = nn.Conv1d(n_inputs, n_outputs, 1) if n_inputs != n_outputs else None
        self.relu = nn.ReLU()
        self.init_weights()
    def init_weights(self):
        self.conv1.weight.data.normal_(0, 0.01)
        self.conv2.weight.data.normal_(0, 0.01)
        if self.downsample is not None:
            self.downsample.weight.data.normal_(0, 0.01) 
    def forward(self, x):
        out = self.net(x)
        res = x if self.downsample is None else self.downsample(x)
        return self.relu(out + res)   
        
class TemporalBlock3(nn.Module):
    def __init__(self, n_inputs, n_outputs, kernel_size, stride, dilation, padding, dropout=0.2):
        super(TemporalBlock3, self).__init__()
        self.conv1 = weight_norm(nn.Conv1d(n_inputs, n_outputs, kernel_size,
                                           stride=1, padding=padding, dilation=4))
        self.chomp1 = Chomp1d(padding)
        self.relu1 = nn.ReLU()
        self.dropout1 = nn.Dropout(dropout)

        self.conv2 = weight_norm(nn.Conv1d(n_outputs, n_outputs, kernel_size,
                                           stride=2, padding=padding, dilation=1))
        self.chomp2 = Chomp1d(padding)
        self.relu2 = nn.ReLU()
        self.dropout2 = nn.Dropout(dropout)

        self.net = nn.Sequential(self.conv1, self.chomp1, self.relu1, self.dropout1,
                                 self.conv2, self.chomp2, self.relu2, self.dropout2)
        self.downsample =  None
        # self.downsample = nn.Conv1d(n_inputs, n_outputs, 1) if n_inputs != n_outputs else None
        self.relu = nn.ReLU()
        self.init_weights()
        
        
    def init_weights(self):
        self.conv1.weight.data.normal_(0, 0.01)
        self.conv2.weight.data.normal_(0, 0.01)
        if self.downsample is not None:
            self.downsample.weight.data.normal_(0, 0.01)

    def forward(self, x):
        out = self.net(x)
        res = x if self.downsample is None else self.downsample(x)
        return self.relu(out + res)


class TemporalConvNet(nn.Module):
    def __init__(self, num_inputs, num_channels, kernel_size, stride_size = [1,1,1], dropout=0.2):
        super(TemporalConvNet, self).__init__()
        layers = []
        num_levels = len(num_channels)
        # for i in range(num_levels):
        #     dilation_size = 2 ** i
        #     in_channels = num_inputs if i == 0 else num_channels[i-1]
        #     out_channels = num_channels[i]
        #     out_stride = kernel_size[i]
        #     layers += [TemporalBlock(in_channels, out_channels, kernel_size, stride=1, dilation=dilation_size,
        #                              padding=(kernel_size-1) * dilation_size, dropout=dropout)]

        layers += [TemporalBlock1(num_inputs, 34, kernel_size, stride=1, dilation=1,
                            padding=(5-1) * 1, dropout=dropout)]
        layers += [TemporalBlock1(34, 34, kernel_size, stride=1, dilation=2,
                    padding=(5-1) * 2, dropout=dropout)]
        layers += [TemporalBlock1(34, 34, kernel_size, stride=1, dilation=4,
            padding=(5-1) * 4, dropout=dropout)]
        
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)

class TCN(nn.Module):
    def __init__(self, input_size, t_steps, output_size, num_channels = [34,34,34], kernel_size = 5, dropout = 0.2):
        super(TCN, self).__init__()
        self.tcn = TemporalConvNet(input_size, num_channels, kernel_size=kernel_size, dropout=dropout)
        self.linear = nn.Linear(num_channels[-1], output_size)
        self.t_steps = t_steps
        self.input_size = input_size

    def forward(self, inputs):
        """Inputs have to have dimension (N, C_in, L_in)"""
        inputs = inputs.transpose(1,2)
        print("inputs: ", inputs.shape)
        y1 = self.tcn(inputs)  # input should have dimension (N, C, L)
        o = self.linear(y1[:, :, -1])
        return F.log_softmax(o, dim=1)
    
    
# input_size = 10  # 输入特征的维度
# output_size = 4  # 输出维度
# num_channels = [34, 34, 34]  # 每个因果卷积层的输出通道数
# kernel_size = 5  # 卷积核大小
# dropout = 0.2  # Dropout 概率

# model = TCN(input_size, output_size, num_channels, kernel_size, dropout)