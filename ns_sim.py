from ns import ns
import sys
from gymnasium.spaces import Discrete, Box
import gymnasium as gym
from typing import Optional
import numpy as np
import tc
from stable_baselines3.common.noise import NormalActionNoise
import math

import random
from importlib import reload
_count = 0

class SimpleNs(gym.Env):

    def __init__(self, config: Optional[dict] = None):
        config = config or {}
        self.max_steps = 10 
        self.cur_steps = 0
        self.action_space = Box(low=-1, high=1, shape=(3,), dtype=np.float32)
        self.observation_space = Box(low = np.array([10, 10, 0,  500, 3, 10000]), high = np.array([101, 101, 1000000, 9000, 1000, 1000000]), 
                 dtype=np.float32)
        _count = random.randint(1, 255)
        self._moden = tc.ns_sim(_count)
        self._prev_reset = None
        self.noise = NormalActionNoise(mean=np.zeros(self.action_space.shape), sigma=0.1 * np.ones(self.action_space.shape))
        

    def reset(self, *, seed=None, options=None):
        _rand = random.randint(1, 255)
        self.cur_steps = 1.0
        self._moden.reset(_rand)
        _obs = self.np_random.uniform(low=[10, 10, 0, 0, 0, 0], high=[101, 101, 1000000, 0, 0, 0], size=(6,))

        # Return obs and (empty) info dict.
        return np.array(_obs, np.float32), {"env_state": "reset"}

    def step(self, action):
        #assert action in [0, 1], action
        # Move left.
        def scale_action(action, low, high):
            return low + (action + 1) * 0.5 * (high - low)
        truncated = False
        infos = {}

        #action normalize
        action[0] = scale_action(action[0], 500, 9000)
        action[1] = scale_action(action[1], 3, 1000)
        action[2] = scale_action(action[2], 10000, 1000000)

        #limit action space
        if action[0] < 500:
            action[0] = 500
        elif action[0] > 9000:
            action[0] = 9000

        if action[1] < 3:
            action[1] = 3
        elif action[1] > 1000:
            action[1] = 1000

        if action[2] < 10000:
            action[2] = 10000
        elif action[2] > 1000000:
            action[2] = 10000000

        #do action
        obs = self._moden.step(action)


        _loss_ratio = obs[0]
        _throughput = obs[1] 
        _rtt = obs[2]
        _rtt_n1 = obs[3]
        _backlog = obs[4]
        _bottle_rate = obs[5] 
        _base_rtt = obs[6] * 0.001
        _base_rtt_n1 = obs[7] * 0.001
        _client_index = obs[8]
        terminated = self.cur_steps >= self.max_steps
        self.cur_steps += 1
        _obs = [obs[0], obs[1], obs[4], action[0], action[1], action[2]]
        if terminated == True:
            self._prev_reset = _obs


        #_action_range1 = 9000 - 500
        #_action_range2 = 1000 - 3
        #_action_range3 = 10000000 - 10000
        #exploration_bonus = np.abs(action[0] - 500) / _action_range1 + np.abs(action[1] - 3) / _action_range2 + np.abs(action[2] - 10000) / _action_range3
        #if exploration_bonus == 0:
        #    exploration_bonus = -0.1
        #else:
        #    exploration_bonus = exploration_bonus * 0.1
        basertt_reward = 10000.0
        if _base_rtt != _rtt:
            basertt_reward = 1.0 / abs(_base_rtt - _rtt)

        basertt_reward_n1 = 10000.0
        if _base_rtt_n1 != _rtt_n1:
            basertt_reward_n1 = 1.0 / abs(_base_rtt_n1 - _rtt_n1)

        b_rate_reward = 10000.0
        if _bottle_rate != _throughput:
            b_rate_reward = 1.0 / abs(_bottle_rate - _throughput)

        backlog_reward = 10000.0
        if obs[4] != 0.0:
            backlog_reward = 1.0 / (obs[4] / 1000000.0)
            backlog_reward = backlog_reward / 100.0
        reward = backlog_reward * 5.0 + _loss_ratio #+ exploration_bonus #+ b_rate_reward * 5.0 -  _loss_ratio + exploration_bonus
        print(f"""
          Step Info:
            Step: {self.cur_steps:5f} | Terminated: {str(terminated):5s}
          Actions:
            action_1: {action[0]:6.3f} | action_2: {action[1]:6.3f} | action_3: {action[2]:6.3f}
          Network Metrics:
            Throughput: {obs[1]:9.4f} Mbps | Loss Ratio: {obs[0]:7.4f}
            Rtt:        {obs[2]: 9.4f} seconds | Rtt n1:    {obs[3]: 9.4f}  client index: {_client_index} | Base RTT: {_base_rtt:7.4f} seconds | Base RTT N1: {_base_rtt_n1: 7.4f} seconds 
            Bottle rate: {obs[5]:9.4f} Mpbs
            Backlog: {obs[4]: 9.4f} p
          Reward:
            {reward: 7.2f}""")
        return (
            np.array(_obs, np.float32),
            reward,
            terminated,
            truncated,
            infos,
        )



