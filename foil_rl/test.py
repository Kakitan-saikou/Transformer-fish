from env.flow_field_env import fake_env, foil_env
import argparse
import json
from model.online_gpt_model import GPTConfig, GPT
from framework.utils import set_seed, ConfigDict, make_logpath
from framework.logger import LogServer, LogClient
# from framework.buffer import OnlineBuffer
from framework.trainer1 import RSAC, TPPO  # , Trainer, SAC
from framework import utils
from framework.normalization import RewardScaling, Normalization
from model.sin_policy import SinPolicy
from datetime import datetime, timedelta
import wandb
import numpy as np
from tqdm import tqdm
import gym
import pickle
import torch


def prepare_arguments():
    parser = argparse.ArgumentParser()
    # Required_parameter
    parser.add_argument("--config-file", "--cf", default="./config/config.json",
                        help="pointer to the configuration file of the experiment", type=str)
    args, unknown = parser.parse_known_args()
    args.config = json.load(open(args.config_file, 'r', encoding='utf-8'))
    print(args.config)

    ### set seed
    if args.config['seed'] == "none":
        args.config['seed'] = datetime.now().microsecond % 65536
        args.seed = args.config['seed']
    set_seed(args.seed)

    # reconfig some parameter
    args.name = f"[Debug]gym_PPO_transformer_{args.seed}"
    # v4 actionDevide_eta_actionRange

    # wandb remote logger, can mute when debug
    mute = True
    remote_logger = LogServer(args, mute=mute)  # open logging when finish debuging
    remote_logger = LogClient(remote_logger)

    # for the hyperparameter search
    if mute:
        new_args = args
    else:
        new_args = remote_logger.server.logger.config if not mute else args
        new_args = ConfigDict(new_args)
        new_args.washDictChange()
    new_args.remote_logger = remote_logger
    return new_args, remote_logger


### load config AND prepare logger
args, remote_logger = prepare_arguments()
config = args.config
dir = "config/tppo_2.yaml"
config_dict = utils.load_config(dir)
paras = utils.get_paras_from_dict(config_dict)
print("local:", paras)
wandb.init(project="foil", config=paras, name=args.name, mode="disabled")# ("disabled" or "online")
paras = utils.get_paras_from_dict(wandb.config)
print("finetune", paras)
run_dir, log_dir = make_logpath('foil', paras.algo)
### start env
#env = foil_env(paras)
num_envs = 10
env = gym.vector.make('Pendulum-v0', num_envs=num_envs)
#env = gym.vector.make('foil-v0', num_envs=num_envs, config=paras)
env.reset()
paras.action_space, action_dim = env.single_action_space.shape[0], env.single_action_space.shape[0]
paras.obs_space, observation_dim = env.single_observation_space.shape[0], env.single_observation_space.shape[0]
paras.device = "cuda" if torch.cuda.is_available() else "cpu"
paras.env_num = num_envs

### train

agent = TPPO(paras)


# sample batch from buffer, train agent
state_norm_flag = False
state_norm = Normalization((num_envs, paras.obs_space))
reward_norm = RewardScaling(1, 0.99)
ret = []
obs = env.reset()
obs = state_norm(obs) if state_norm_flag else obs
done = [False] * num_envs
epsoide_length = 0
epsoide_num = 0
buffer_new_data = 0
agent.reset_optimizer()  # change the mode from offline to online
for i in tqdm(range(config['epochs'])):
    # rollout in env  -  rollout()
    epsoide_length, Gt = 0, 0
    ct_ls, cp_ls, fx_ls, dt_ls = [], [], [], []
    agent.reset_state()
    while not any(done) and epsoide_length < 800:
        action = agent.choose_action(obs)
        next_obs, ori_reward, done, info = env.step(action)
        # [trick] normalization of reward and observation
        next_obs = state_norm(next_obs) if state_norm_flag else next_obs
        reward = reward_norm(ori_reward)
        # save to buffer
        next_state = next_obs.reshape(next_obs.shape[0], 1, next_obs.shape[1])
        next_state = np.concatenate([agent.state.cpu().detach().numpy(), next_state], axis=1)[:, 1:, :]
        agent.insert_data({'states': agent.state.cpu().detach().numpy(), 'actions': action, 'rewards': reward, 'states_next': next_state, 'dones': done})
        obs = next_obs
        Gt += ori_reward[0]
        epsoide_length += 1
        buffer_new_data += num_envs
        if buffer_new_data > paras.batch_size:
            # it is better to use data for only 1-2 times
            #agent.learn()
            buffer_new_data = 0
            #wandb.log({"reward": reward, "critic_loss": agent.c_loss, "actor_loss": agent.a_loss, "alpha_loss": agent.alpha_loss})
    obs = env.reset()
    obs = state_norm(obs) if state_norm_flag else obs
    agent.learn()
    # clear online buffer
    agent.memory.buffer_dict_clear()
    done = [False] * num_envs
    ret.append(Gt)
    avg_reward = (Gt - reward[0]) / epsoide_length
    eff_avg_reward = avg_reward if epsoide_length > 8000 / paras.action_interval else 0  # set too short episode to zero
    # cal ct, eta, cp, fx from the middle
    wandb.log({"Gt": Gt, "length": epsoide_length, "episode_num": epsoide_num, "avg_reward": avg_reward,
               "effective_avg": eff_avg_reward,"policy_entropy": agent.actor.entropy, "clip_frac": agent.clipfrac, "approxkl": agent.approxkl,
                "time": np.sum(dt_ls), "reward": reward, "critic_loss": agent.critic_loss, "actor_loss": agent.actor_loss})  # avg should remove the final step reward, since it can be too large
    print("epoch:", i, "length:", epsoide_length, "G:", Gt, "actor_loss:", agent.actor_loss, "critic_loss", agent.critic_loss, " sigma:", agent.actor.sigma_param)
    print("action:", action, "; state: ", next_obs, "; reward:", reward)
    epsoide_num += 1
    # update policy   -  update()
    # agent.learn()
    # eval policy performance - rollout(train=False)
    # logger
    # if i % 10 == 0:

    # save model parameter
    if i % 1000 == 0:
        agent.save(run_dir, i)
env.terminate()
