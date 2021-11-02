from ns3gym import ns3env
from comet_ml import Experiment, Optimizer
import tqdm
import subprocess
from collections import deque
import numpy as np

from agents.ddpg.agent import Agent, Config
from agents.ddpg.model import Actor
from agents.teacher import Teacher, EnvWrapper
from preprocessor import Preprocessor

# TRAIN, EVAL OR DRYRUN EXECUTION
runTrain = False
mixedTrain = False        # Enable/Disable training with different configurations (UDP/TCP, direction)
runEval = False
runDry = True

# SCENARIO CONFIGURATION
new_state = True         # True: state = p_col + n; False: state = p_col
udp = True               # True: UDP; False: TCP
direction = 2            # 0: UL, 1: DL, 2: UL+DL
nWifi = 25
scenario = "basic"
cw_dryRun = 184          # CW value for dryRun execution
tracing = False

simTime = 60             # seconds
stepTime = 0.01          # seconds
history_length = 300
stas_window = 10
stas_threshold = 5       # pkts

EPISODE_COUNT = 15
steps_per_ep = int(simTime/stepTime)

sim_args = {
    "newState": new_state,
    "udp": udp,
    "direction": direction,
    "nWifi": nWifi,
    "scenario": scenario,
    "dryRun": runDry,
    "cw_dryRun": cw_dryRun,
    "tracing": tracing,
    "simTime": simTime,
    "envStepTime": stepTime,
    "historyLength": history_length,
    "stasWindow": stas_window,
    "stasThreshold": stas_threshold,
    "agentType": Agent.TYPE,
}

print("Steps per episode:", steps_per_ep)

threads_no = 1
env = EnvWrapper(threads_no, **sim_args)

env.reset()
ob_space = env.observation_space
ac_space = env.action_space

print("Observation space shape:", ob_space)
print("Action space shape:", ac_space)

assert ob_space is not None

teacher = Teacher(env, 1, Preprocessor(False))

lr_actor = 4e-4
lr_critic = 4e-3

config = Config(buffer_size=4*steps_per_ep*threads_no, batch_size=32, gamma=0.7, tau=1e-3, lr_actor=lr_actor, lr_critic=lr_critic, update_every=1)
agent = Agent(new_state, history_length, action_size=1, config=config, actor_layers=[8, 128, 16], critic_layers=[8,128,16])

# Test the model
hyperparams = {**config.__dict__, **sim_args}
tags = ["Rew: normalized speed",
        f"{Agent.NAME}",
        f"Actor: {lr_actor}",
        f"Critic: {lr_critic}",
        f"Instances: {threads_no}",
        *[f"{key}: {sim_args[key]}" for key in list(sim_args)]]

if runTrain:
    logger = teacher.train(agent, EPISODE_COUNT,
                        simTime=simTime,
                        stepTime=stepTime,
                        history_length=history_length,
                        send_logs=True,
                        experimental=True,
                        tags=tags,
                        parameters=hyperparams,
                        mixedTrain=mixedTrain)
    agent.save()

if runEval:
    logger = teacher.eval(True,
                        agent,
                        simTime=simTime,
                        stepTime=stepTime,
                        history_length=history_length,
                        tags=tags,
                        parameters=hyperparams)

if runDry:
    logger = teacher.eval(False,
                        agent,
                        simTime=simTime,
                        stepTime=stepTime,
                        history_length=history_length,
                        tags=tags,
                        parameters=hyperparams)
