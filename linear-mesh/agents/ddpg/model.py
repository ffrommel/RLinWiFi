import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

def hidden_init(layer):
    fan_in = layer.weight.data.size()[0]
    lim = 1. / np.sqrt(fan_in)
    return (-lim, lim)


class Actor(nn.Module):
    """Actor (Policy) Model."""

    def __init__(self, new_state, state_size, action_size, seed, batch_size, fc_units=8, fc2_units=64, fc3_units=32):
        """Initialize parameters and build model.

        Args:
            new_state (bool): Use state = p_col + n (true) or state = p_col (false)
            state_size (int): Dimension of each state
            action_size (int): Dimension of each action
            seed (int): Random seed
            fc_units (int, optional): Defaults to 300. Number of nodes in first hidden layer
            fc2_units (int, optional): Defaults to 200. Number of nodes in second hidden layer
        """

        super(Actor, self).__init__()
        self.fc_units = fc_units
        self.batch_size = batch_size
        self.state_size = int(state_size/3)
        if seed!=-1:
            self.seed = torch.manual_seed(seed)
        self.norm = torch.nn.BatchNorm1d(np.ceil(state_size/(state_size//4)).astype(int))
        self.lstm1 = nn.LSTM(3, fc_units) if new_state else nn.LSTM(2, fc_units)
        self.norm1 = torch.nn.BatchNorm1d(fc_units)
        self.fc2 = nn.Linear(fc_units, fc2_units)
        self.norm2 = torch.nn.BatchNorm1d(fc2_units)
        self.fc3 = nn.Linear(fc_units, fc3_units)
        self.dropout = torch.nn.Dropout(0.5)
        self.fc4 = nn.Linear(fc3_units, action_size)
        self.reset_parameters()

    def reset_parameters(self):
        self.fc2.weight.data.uniform_(*hidden_init(self.fc2))
        self.fc3.weight.data.uniform_(*hidden_init(self.fc3))
        self.fc4.weight.data.uniform_(-3e-3, 3e-3)

    def forward(self, state):
        """Build an actor (policy) network that maps states -> actions."""

        h0 = torch.randn(1, state.shape[1], self.fc_units).to(device)
        c0 = torch.randn(1, state.shape[1], self.fc_units).to(device)

        x, _ = self.lstm1(state, (h0, c0))
        x = F.relu(x[-1])
        x = F.relu(self.fc3(x))

        return self.fc4(x)


class Critic(nn.Module):
    """Critic (Value) Model."""

    def __init__(self, new_state, state_size, action_size, seed, batch_size, fcs1_units=8, fc2_units=64, fc3_units=32):
        """Initialize parameters and build model.
        Params
        ======
            new_state (bool): Use state = p_col + n (true) or state = p_col (false)
            state_size (int): Dimension of each state
            action_size (int): Dimension of each action
            seed (int): Random seed
            fcs1_units (int): Number of nodes in the first hidden layer
            fc2_units (int): Number of nodes in the second hidden layer
        """

        super(Critic, self).__init__()
        self.fc_units = fcs1_units
        self.batch_size = batch_size
        self.state_size = int(state_size/3)

        self.seed = torch.manual_seed(seed)

        self.lstm1 = nn.LSTM(3, fcs1_units) if new_state else nn.LSTM(2, fcs1_units)
        self.norm1 = torch.nn.BatchNorm1d(fcs1_units)
        self.fc2 = nn.Linear(fcs1_units+action_size, fc2_units)
        self.fc3 = nn.Linear(fc2_units, fc3_units)
        self.fc4 = nn.Linear(fc2_units, 1)

        self.reset_parameters()

    def reset_parameters(self):
        self.fc2.weight.data.uniform_(*hidden_init(self.fc2))
        self.fc3.weight.data.uniform_(*hidden_init(self.fc3))
        self.fc4.weight.data.uniform_(-3e-3, 3e-3)

    def forward(self, state, action):
        """Build a critic (value) network that maps (state, action) pairs -> Q-values."""
        h0 = torch.randn(1, state.shape[1], self.fc_units).to(device)
        c0 = torch.randn(1, state.shape[1], self.fc_units).to(device)

        xs, _ = self.lstm1(state, (h0, c0))
        x = F.relu(xs[-1])
        x = torch.cat((x, action), dim=1)
        x = F.relu(self.fc2(x))

        return self.fc4(x)
