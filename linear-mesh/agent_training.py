
#%%
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

import sys, getopt
from distutils import util

#%%
runTrain = False
runEval = True
runDryrun = False

try:
    opts, args = getopt.getopt(sys.argv[1:], "ht:e:d:", ["help", "runTrain=", "runEval=", "runDryrun="])
except getopt.GetoptError:
    print("ERROR! CORRECT USAGE: agent_training.py --runTrain <bool> --runEval <bool> --runDryrun <bool>")
    sys.exit(2)
for opt, arg in opts:
    if opt in ("-h", "--help"):
        print("HELP: agent_training.py --runTrain <bool> --runEval <bool> --runDryrun <bool>")
        sys.exit()
    elif opt in ("-t", "--runTrain"):
        try:
            runTrain = bool(util.strtobool(arg))
        except ValueError:
            print("ERROR! CORRECT USAGE: agent_training.py --runTrain <bool> --runEval <bool> --runDryrun <bool>")
            sys.exit(2)
    elif opt in ("-e", "--runEval"):
        try:
            runEval = bool(util.strtobool(arg))
        except ValueError:
            print("ERROR! CORRECT USAGE: agent_training.py --runTrain <bool> --runEval <bool> --runDryrun <bool>")
            sys.exit(2)
    elif opt in ("-d", "--runDryrun"):
        try:
            runDryrun = bool(util.strtobool(arg))
        except ValueError:
            print("ERROR! CORRECT USAGE: agent_training.py --runTrain <bool> --runEval <bool> --runDryrun <bool>")
            sys.exit(2)
#print("train:", runTrain, ", eval:", runEval, ", dryrun:", runDryrun);

#%%
scenario = "basic"

simTime = 60 # seconds
stepTime = 0.01  # seconds
history_length = 300

EPISODE_COUNT = 15
steps_per_ep = int(simTime/stepTime)

sim_args = {
    "simTime": simTime,
    "envStepTime": stepTime,
    "historyLength": history_length,
    "agentType": Agent.TYPE,
    "scenario": scenario,
    "nWifi": 30,
    "nAx": 25,
    "uplink": True,
    "udp": False,
    "dryRun": runDryrun
}

print("Steps per episode:", steps_per_ep)

threads_no = 1
env = EnvWrapper(threads_no, **sim_args)

#%%
env.reset()
ob_space = env.observation_space
ac_space = env.action_space

print("Observation space shape:", ob_space)
print("Action space shape:", ac_space)

assert ob_space is not None

#%%
teacher = Teacher(env, 1, Preprocessor(False))

lr_actor = 4e-4
lr_critic = 4e-3

config = Config(buffer_size=4*steps_per_ep*threads_no, batch_size=32, gamma=0.7, tau=1e-3, lr_actor=lr_actor, lr_critic=lr_critic, update_every=1)
agent = Agent(history_length, action_size=2, config=config, actor_layers=[8, 128, 16], critic_layers=[8,128,16]) # TODO: Implement paper architecture

# Test the model
hyperparams = {**config.__dict__, **sim_args}
tags = ["Rew: normalized speed",
        f"{Agent.NAME}",
        sim_args['scenario'],
        f"Actor: {lr_actor}",
        f"Critic: {lr_critic}",
        f"Instances: {threads_no}",
        f"Station count: {sim_args['nWifi']}",
        *[f"{key}: {sim_args[key]}" for key in list(sim_args)[:3]]]

if runTrain:
    logger = teacher.train(agent, EPISODE_COUNT,
                        simTime=simTime,
                        stepTime=stepTime,
                        history_length=history_length,
                        send_logs=True,
                        experimental=True,
                        tags=tags,
                        parameters=hyperparams)
    agent.save()

if runEval:
    logger = teacher.eval(agent,
                         simTime=simTime,
                         stepTime=stepTime,
                         history_length=history_length,
                         tags=tags,
                         parameters=hyperparams,
                         dry_run=False)

if runDryrun:
#    logger = teacher.dry_run(agent,
#                         steps_per_ep=steps_per_ep)
    logger = teacher.eval(agent,
                         simTime=simTime,
                         stepTime=stepTime,
                         history_length=history_length,
                         tags=tags,
                         parameters=hyperparams,
                         dry_run=True)
