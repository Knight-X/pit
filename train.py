import gymnasium as gym
import ns_sim

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, SubprocVecEnv
from stable_baselines3.common.utils import set_random_seed

from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from stable_baselines3.common.logger import configure
import time
import numpy as np

class Forced(BaseCallback):
    def __init__(self, check_freq, log_dir, verbose=1):
        super(Forced, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.rewards = []

    def _on_step(self) -> bool:
        if len(self.rewards) == 0:
            self.rewards = [[] for _ in range(self.training_env.num_envs)]

        for i, reward in enumerate(self.locals['rewards']):
            self.rewards[i].append(reward)

        if self.n_calls % self.check_freq == 0:
            mean_rewards = [np.mean(env_rewards) for env_rewards in self.rewards]
            overall_mean_reward = np.mean(mean_rewards)
            self.logger.record("rollout/mean_reward", overall_mean_reward)
            self.logger.dump(self.n_calls)
            self.rewards = [[] for _ in range(self.training_env.num_envs)]
        return True

def make_env(env_id: str, rank: int, seed: int = 0):

    def _init():
        env = ns_sim.SimpleNs()
        time.sleep(2)
        return env
    set_random_seed(seed)
    return _init

if __name__ == "__main__":

    _vec_env = SubprocVecEnv([make_env("ns_sim", i) for i in range(32)])
    _vec_env = VecNormalize(_vec_env, norm_obs=True)
    model = PPO("MlpPolicy",  _vec_env, n_steps=64, verbose =1, tensorboard_log="./sb35/")
    checkpoint_callback = Forced(check_freq=32, log_dir="./sb35/")

    model.learn(total_timesteps=180000, log_interval=1, callback=checkpoint_callback)
    model.save("ppo_stand")


