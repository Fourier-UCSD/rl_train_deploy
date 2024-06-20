import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import math 
import matplotlib.pyplot as plt

class LSTMModel(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(LSTMModel, self).__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers = 2,batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)
        self.hiden_state = None

    def act(self, x, train_mode=True):
        if train_mode==True:
            lstm_out, _= self.lstm(x)
            # 在这里，lstm_out 是 LSTM 模型的输出，通常是一个三维的张量，
            # 其形状为 (batch_size, sequence_length, hidden_size)。
            # [:, -1, :] 中的冒号 : 表示在该维度上选择所有的元素。
            # -1 表示选择该维度的最后一个元素，即选择最后一个时间步的输出。
            # last_lstm_out 保存了最后一个时间步的 LSTM 输出。
            last_lstm_out = lstm_out[:, :, :]
            output = self.fc(last_lstm_out)
        else:
            lstm_out, self.hiden_state= self.lstm(x,self.hiden_state)
            last_lstm_out = lstm_out[:, -1, :]
            output = self.fc(last_lstm_out)
        return output
    def reset(self):
        self.hiden_state = None
        
        
class MotorDelay_80(nn.Module):
    def __init__(self, num_envs, num_actions):
        super(MotorDelay_80, self).__init__()
        self.a = 1.2766
        self.b = 12.13208
        # self.alpha = 1.0
        self.alpha = torch.exp(torch.tensor([-1 / self.b]).to("cuda:0"))
        self.beta = self.a / self.b
        # self.y_pre = 0.0
        self.y_pre = torch.zeros(num_envs, num_actions, dtype = torch.float, device="cuda:0")

        
    def forward(self, x):
        if x.dim() ==1:
            x = x.unsqueeze(1)
        
        # if self.y_pre is None:
        #     self.y_pre = torch.zeros(x.size(0), x.size(1), dtype = x.dtype, device=x.device)

        y = self.alpha * self.y_pre + self.beta * x
        self.y_pre = y
        return y
    
    def reset(self, env_idx):
        self.y_pre[env_idx] = 0
    
    
class MotorDelay_130(nn.Module):
    def __init__(self, num_envs, num_actions):
        super(MotorDelay_130, self).__init__()
        self.a = 0.91
        self.b = 11.28
        # self.alpha = 1.0
        self.alpha = torch.exp(torch.tensor([-1 / self.b]).to("cuda:0"))
        self.beta = self.a / self.b
        # self.y_pre = 0.0
        self.y_pre = torch.zeros(num_envs, num_actions, dtype = torch.float, device="cuda:0")


    def forward(self, x):
        if x.dim() ==1:
            x = x.unsqueeze(1)
        
        # if self.y_pre is None:
        #     self.y_pre = torch.zeros(x.size(0), x.size(1), dtype = x.dtype, device=x.device)

        y = self.alpha * self.y_pre + self.beta * x
        self.y_pre = y
        return y
    
    def reset(self, env_idx):
        self.y_pre[env_idx] = 0
    
class MotorDelay_60(nn.Module):
    def __init__(self, num_envs, num_actions):
        super(MotorDelay_60, self).__init__()
        self.a = 0.2419
        self.b = 10.4578
        # self.alpha = 1.0
        self.alpha = torch.exp(torch.tensor([-1 / self.b]).to("cuda:0"))
        self.beta = self.a / self.b
        # self.y_pre = 0.0
        self.y_pre = torch.zeros(num_envs, num_actions, dtype = torch.float, device="cuda:0")

        
    def forward(self, x):
        if x.dim() ==1:
            x = x.unsqueeze(1)
        
        # if self.y_pre is None:
        #     self.y_pre = torch.zeros(x.size(0), x.size(1), dtype = x.dtype, device=x.device)

        y = self.alpha * self.y_pre + self.beta * x
        self.y_pre = y
        return y
    
    def reset(self, env_idx):
        self.y_pre[env_idx] = 0