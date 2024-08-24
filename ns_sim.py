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
        self.observation_space = Box(
            0.0, self.max_steps, shape=(
                6,), dtype=np.float32)
        _count = random.random() % 250
        self._moden = tc.ns_sim(_count)
        self._prev_reset = None
        self.noise = NormalActionNoise(mean=np.zeros(self.action_space.shape), sigma=0.1 * np.ones(self.action_space.shape))
        

    def reset(self, *, seed=None, options=None):
        if seed == None:
            seed = 5
        self.seed = seed
        random.seed(seed)
        self.cur_steps = 1.0
        _obs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._moden.reset(self.seed)
        if self._prev_reset != None:
            _obs = self._prev_reset
            self._prev_reset = None

        _obs[0] = random.random()
        _obs[1] = random.random()
        _obs[2] = random.random()
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
        old_min = -1
        old_max = 1
        range_1 = 9000 - 500
        range_2 = 1000 - 3
        range_3 = 1000000 - 10000
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
        _throughput = obs[1] / 100000000000000
        _rtt = obs[2]
        _bottle_rate = obs[3] * 0.01
        _base_rtt = obs[4] * 0.001
        terminated = self.cur_steps >= self.max_steps
        self.cur_steps += 1
        _obs = [obs[0], obs[1], obs[2], action[0], action[1], action[2]]
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

        b_rate_reward = 10000.0
        if _bottle_rate != _throughput:
            b_rate_reward = 1.0 / abs(_bottle_rate - _throughput)
        reward = basertt_reward * 5.0 - _loss_ratio #+ exploration_bonus #+ b_rate_reward * 5.0 -  _loss_ratio + exploration_bonus
        print(f"""
          Step Info:
            Step: {self.cur_steps:5f} | Terminated: {str(terminated):5s}
          Actions:
            action_1: {action[0]:6.3f} | action_2: {action[1]:6.3f} | action_3: {action[2]:6.3f}
          Network Metrics:
            Throughput: {_throughput:9.2f} | Loss Ratio: {_loss_ratio:7.4f}
            Rtt:        {_rtt: 9.2f} | Base RTT: {_base_rtt:7.2f}
            Bottle rate: {_bottle_rate:9.2f}
          Reward:
            {reward: 7.2f}""")
        return (
            np.array(_obs, np.float32),
            reward,
            terminated,
            truncated,
            infos,
        )



