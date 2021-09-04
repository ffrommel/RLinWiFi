import numpy as np
import tqdm
import subprocess
from comet_ml import Experiment
from ns3gym import ns3env
from ns3gym.start_sim import find_waf_path

import matplotlib.pyplot as plt
from collections import deque
import time
import json
import os
import glob

import math
from scipy.optimize import fsolve
import csv

import sys
from datetime import datetime, timedelta


class Logger:
    def __init__(self, send_logs, tags, parameters, experiment=None):
        self.stations = 5
        self.send_logs = send_logs
        if self.send_logs:
            if experiment is None:
                json_loc = glob.glob("./**/comet_token.json")[0]
                with open(json_loc, "r") as f:
                    kwargs = json.load(f)

                self.experiment = Experiment(**kwargs)
            else:
                self.experiment = experiment
        self.sent_mb = 0
        self.speed_window = deque(maxlen=100)
        self.step_time = None
        self.current_speed = 0
        if self.send_logs:
            if tags is not None:
                self.experiment.add_tags(tags)
            if parameters is not None:
                self.experiment.log_parameters(parameters)

    def begin_logging(self, episode_count, steps_per_ep, sigma, theta, step_time):
        self.step_time = step_time
        if self.send_logs:
            self.experiment.log_parameter("Episode count", episode_count)
            self.experiment.log_parameter("Steps per episode", steps_per_ep)
            self.experiment.log_parameter("theta", theta)
            self.experiment.log_parameter("sigma", sigma)

    def log_round(self, states, reward, cumulative_reward, info, loss, observations, step):

        self.experiment.log_histogram_3d(states, name="Observations", step=step)
        info = [[j for j in i.split("|")] for i in info]
        info = np.mean(np.array(info, dtype=np.float32), axis=0)
        try:
            # round_mb = np.mean([float(i.split("|")[0]) for i in info])
            round_mb = info[0]
        except Exception as e:
            print(info)
            print(reward)
            raise e
        self.speed_window.append(round_mb)
        self.current_speed = np.mean(np.asarray(self.speed_window)/self.step_time)
        self.sent_mb += round_mb
        # CW = np.mean([float(i.split("|")[1]) for i in info])
        CW = info[1]
        # stations = np.mean([float(i.split("|")[2]) for i in info])
        self.stations = info[2]
        fairness = info[3]

        if self.send_logs:
            self.experiment.log_metric("Round reward", np.mean(reward), step=step)
            self.experiment.log_metric("Per-ep reward", np.mean(cumulative_reward), step=step)
            self.experiment.log_metric("Megabytes sent", self.sent_mb, step=step)
            self.experiment.log_metric("Round megabytes sent", round_mb, step=step)
            self.experiment.log_metric("Chosen CW", CW, step=step)
            self.experiment.log_metric("Station count", self.stations, step=step)
            self.experiment.log_metric("Current throughput", self.current_speed, step=step)
            self.experiment.log_metric("Fairness index", fairness, step=step)

            for i, obs in enumerate(observations):
                self.experiment.log_metric(f"Observation {i}", obs, step=step)

            self.experiment.log_metrics(loss, step=step)

    def log_episode(self, cumulative_reward, speed, step):
        if self.send_logs:
            self.experiment.log_metric("Cumulative reward", cumulative_reward, step=step)
            self.experiment.log_metric("Speed", speed, step=step)

        self.sent_mb = 0
        self.last_speed = speed
        self.speed_window = deque(maxlen=100)
        self.current_speed = 0

    def end(self):
        if self.send_logs:
            self.experiment.end()

def solve_eqs(z, *params):
        tau = z[0]
        p = z[1]
        W_min, m, n = params

        F = np.empty((2))
        F[0] = tau - 2 * (1 - 2 * p) / ((1 - 2 * p) * (W_min + 1) + p * W_min * (1 - pow(2 * p, m)))
        F[1] = p - 1 + pow(1 - tau, n - 1)
        return F

class Teacher:
    """Class that handles training of RL model in ns-3 simulator

    Attributes:
        agent: agent which will be trained
        env (ns3-gym env): environment used for learning. NS3 program must be run before creating teacher
        num_agents (int): number of agents present at once
    """

    def __init__(self, env, num_agents, preprocessor):
        self.preprocess = preprocessor.preprocess
        self.env = env
        self.num_agents = num_agents
        self.CW = 16
        self.action = None              # For debug purposes

    def calculate_step(self, w, n):
        phy_h = 48                                                  # PHY header size (b) 
        mac_h = 272                                                 # MAC header size (b)
        E = 988 * 8                                                 # payload size (b)
        A = 112                                                     # ACK size (b)
        phy_rate = 6e6                                              # PHY header and preamble rate (bps)
        data_rate = 143.4e6                                         # data rate (bps) -- MCS 11 - 20 MHz channel - 1 SS (https://en.wikipedia.org/wiki/Wi-Fi_6)
        ack_rate = 6e6                                              # ACK rate (bps)
        aSlotTime = 9e-6                                            # slot time (s) -- ("Performance Evaluation of OFDMA and MU-MIMO in 802.11ax Networks")
        aSIFSTime = 16e-6                                           # SIFS (s) -- ("Performance Evaluation of OFDMA and MU-MIMO in 802.11ax Networks")
        aDIFSTime = aSIFSTime + 2 * aSlotTime                       # DIFS (s) -- (https://en.wikipedia.org/wiki/DCF_Interframe_Space)
        d = 7e-9                                                    # propagation delay (s)
        t_send = (phy_h + mac_h) / phy_rate + E / data_rate         # time it takes to send a complete packet
        t_ack = (phy_h + A) / ack_rate                              # time it takes to transmit an acknowledgement
        aEIFSTime = t_ack + aSIFSTime + aDIFSTime                   # EIFS (s) -- (https://en.wikipedia.org/wiki/Extended_interframe_space)
        T_s = t_send + aSIFSTime + d + t_ack + aDIFSTime + d        # duration of a time-slot containing a successful transmission
        T_c = t_send + aEIFSTime + d                                # duration of a time-slot containing a collision

        # Calculate 'tau' and 'p'
        W_min = w                                                   # minimum contention window
        W_max = w                                                   # maximum contention window
        m = math.log2(W_max / W_min)                                # maximum number of backoff stages
        z_guess = np.array([0.99, 0.99])
        params = (W_min, m, n)
        z = fsolve(solve_eqs, z_guess, args=params)
        tau = z[0]
        p = z[1]

        # Calculate throughput
        S_max = 40e6                                                # throughput upper bound for reward
        P_tr = 1 - pow(1 - tau, n)
        P_s = n * tau * pow(1 - tau, n - 1) / P_tr

        S = P_s * P_tr * E / ((1 - P_tr) * aSlotTime + P_tr * P_s * T_s + P_tr * (1 - P_s) * T_c)

        return p, S, S/S_max

    def dry_run(self, agent, steps_per_ep):
        obs = self.env.reset()
        obs = self.preprocess(np.reshape(obs, (-1, len(self.env.envs), 1)), 25) # 25 stas

        with tqdm.trange(steps_per_ep) as t:
            for step in t:
                self.actions = agent.act()
                next_obs, reward, done, info = self.env.step(self.actions)

                obs = self.preprocess(np.reshape(next_obs, (-1, len(self.env.envs), 1)))

                if(any(done)):
                    break

    def eval(self, loadAgent, agent, simTime, stepTime, history_length, tags=None, parameters=None, experiment=None):
        if loadAgent:
            agent.load()
        steps_per_ep = int(simTime/stepTime + history_length)

        logger = Logger(True, tags, parameters, experiment=experiment)
        try:
            logger.begin_logging(1, steps_per_ep, agent.noise.sigma, agent.noise.theta, stepTime)
        except  AttributeError:
            logger.begin_logging(1, steps_per_ep, None, None, stepTime)
        add_noise = False

        obs_dim = 1
        time_offset = history_length//obs_dim*stepTime

        cumulative_reward = 0
        reward = [0]
        sent_mb = 0
        nWifi = parameters['nWifi']
        cw = 15
        done = [False]

        obs = [[0] * history_length]
        next_obs = [[0] * history_length]
        obs_preproc = self.preprocess(np.reshape(obs, (-1, len(self.env.envs), obs_dim)), nWifi)

        is_convergence = parameters['scenario'] == 'convergence'
        steps_limit = steps_per_ep / (nWifi - 4)
        steps_counter = 0

        if is_convergence:
            if  nWifi > 5:
                nWifi = 5
            else:
                sys.exit("Not enough Wi-Fi stations to support the convergence scenario.")

        f = open('output_eval.csv', 'a')
        writer = csv.writer(f)
        writer.writerow(['cw', 'action', 'pcol', 'thr', 'reward'])

        with tqdm.trange(steps_per_ep) as t:
            for step in t:
                self.debug = obs

                self.actions = agent.act(np.array(obs_preproc, dtype=np.float32), add_noise)

                # update CW
                cw = int(pow(2, self.actions[0][0] + 4))

                min_cw = 16
                max_cw = 1024
                cw = min(max_cw, max(cw, min_cw));

                if is_convergence:
                    if steps_counter >= steps_limit:
                        nWifi = nWifi + 1 if nWifi < parameters['nWifi'] else parameters['nWifi']
                        steps_counter = 0

                pcol, thr, reward[0] = self.calculate_step(cw, nWifi)
                next_obs[0].pop(-1) # remove last element
                next_obs[0].insert(0,pcol) # add last pcol at the beginning
                next_obs_preproc = self.preprocess(np.reshape(next_obs, (-1, len(self.env.envs), obs_dim)), nWifi)
                #print("\n", next_obs)
                #print("\n", next_obs_preproc)

                # create info string (Mbytes_sent | CW | STAs | Jain Index)
                info = ["NaN|" + str(cw) + "|" + str(nWifi) + "|NaN"]

                cumulative_reward += np.mean(reward)

                if step>(history_length/obs_dim):
                    logger.log_round(obs_preproc, reward, cumulative_reward, info, agent.get_loss(), np.mean(obs_preproc, axis=0)[0], step)
                t.set_postfix(mb_sent=f"{logger.sent_mb:.2f} Mb", curr_speed=f"{logger.current_speed:.2f} Mbps")

                obs_preproc = next_obs_preproc

                # write output file
                writer.writerow([cw, self.actions[0][0], pcol, thr, reward[0]])

                if(any(done)):
                    break

                steps_counter += 1

        self.env.close()
        self.env = EnvWrapper(self.env.no_threads, **self.env.params)

        print(f"Sent {logger.sent_mb:.2f} Mb/s.\tMean speed: {logger.sent_mb/(simTime):.2f} Mb/s\tEval finished\n")

        logger.log_episode(cumulative_reward, logger.sent_mb/(simTime), 0)

        logger.end()
        return logger


    def train(self, agent, EPISODE_COUNT, simTime, stepTime, history_length, send_logs=True, experimental=True, tags=None, parameters=None, experiment=None):
        steps_per_ep = int(simTime/stepTime + history_length)

        logger = Logger(send_logs, tags, parameters, experiment=experiment)
        try:
            logger.begin_logging(EPISODE_COUNT, steps_per_ep, agent.noise.sigma, agent.noise.theta, stepTime)
        except  AttributeError:
            logger.begin_logging(EPISODE_COUNT, steps_per_ep, None, None, stepTime)

        add_noise = True

        obs_dim = 1
        time_offset = history_length//obs_dim*stepTime

        for i in range(EPISODE_COUNT):
            print(i)

            if i>=EPISODE_COUNT*4/5:
                add_noise = False
                print("Turning off noise")

            cumulative_reward = 0
            reward = [0]
            sent_mb = 0
            nWifi = parameters['nWifi']
            cw = 15
            done = [False]

            obs = [[0] * history_length]
            next_obs = [[0] * history_length]
            obs_preproc = self.preprocess(np.reshape(obs, (-1, len(self.env.envs), obs_dim)), nWifi)

            is_convergence = parameters['scenario'] == 'convergence'
            steps_limit = steps_per_ep / (nWifi - 4)
            steps_counter = 0

            self.last_actions = None

            if is_convergence:
                if  nWifi > 5:
                    nWifi = 5
                else:
                    sys.exit("Not enough Wi-Fi stations to support the convergence scenario.")

            f = open('output_train.csv', 'a')
            writer = csv.writer(f)
            writer.writerow(['cw', 'action', 'pcol', 'thr', 'reward'])

            with tqdm.trange(steps_per_ep) as t:
                for step in t:
                    self.debug = obs

                    self.actions = agent.act(np.array(obs_preproc, dtype=np.float32), add_noise)

                    # update CW
                    cw = int(pow(2, self.actions[0][0] + 4))

                    min_cw = 16
                    max_cw = 1024
                    cw = min(max_cw, max(cw, min_cw));

                    if is_convergence:
                        if steps_counter >= steps_limit:
                            nWifi = nWifi + 1 if nWifi < parameters['nWifi'] else parameters['nWifi']
                            steps_counter = 0

                    pcol, thr, reward[0] = self.calculate_step(cw, nWifi)
                    next_obs[0].pop(-1) # remove last element
                    next_obs[0].insert(0,pcol) # add last pcol at the beginning
                    next_obs_preproc = self.preprocess(np.reshape(next_obs, (-1, len(self.env.envs), obs_dim)), nWifi)
                    #print("\n", next_obs)
                    #print("\n", next_obs_preproc)

                    # create info string (Mbytes_sent | CW | STAs | Jain Index)
                    info = ["NaN|" + str(cw) + "|" + str(nWifi) + "|NaN"]

                    if self.last_actions is not None and step>(history_length/obs_dim) and i<EPISODE_COUNT-1:
                        agent.step(obs_preproc, self.actions, reward, next_obs_preproc, done, 2)

                    cumulative_reward += np.mean(reward)

                    self.last_actions = self.actions

                    if step>(history_length/obs_dim):
                        logger.log_round(obs_preproc, reward, cumulative_reward, info, agent.get_loss(), np.mean(obs_preproc, axis=0)[0], i*steps_per_ep+step)
                    t.set_postfix(mb_sent=f"{logger.sent_mb:.2f} Mb", curr_speed=f"{logger.current_speed:.2f} Mbps")

                    obs_preproc = next_obs_preproc

                    # write output file
                    writer.writerow([cw, self.actions[0][0], pcol, thr, reward[0]])

                    if(any(done)):
                        break

                    steps_counter += 1

            self.env.close()

            agent.reset()
            print(f"Sent {logger.sent_mb:.2f} Mb/s.\tMean speed: {logger.sent_mb/(simTime):.2f} Mb/s\tEpisode {i+1}/{EPISODE_COUNT} finished\n")

            logger.log_episode(cumulative_reward, logger.sent_mb/(simTime), i)

        logger.end()
        print("Training finished.")
        return logger

class AlreadyRunningException(Exception):
    def __init__(self, *args, **kwargs):
        return super().__init__(*args, **kwargs)

class EnvWrapper:
    def __init__(self, no_threads, **params):
        self.params = params
        self.no_threads = no_threads
        self.ports = [13968+i+np.random.randint(40000) for i in range(no_threads)]
        self.commands = self._craft_commands(params)

        self.SCRIPT_RUNNING = False
        self.envs = []

        self.run()
        for port in self.ports:
            env = ns3env.Ns3Env(port=port, stepTime=params['envStepTime'], startSim=0, simSeed=0, simArgs=params, debug=False)
            self.envs.append(env)

        self.SCRIPT_RUNNING = True

    def run(self):
        if self.SCRIPT_RUNNING:
            raise AlreadyRunningException("Script is already running")

        for cmd, port in zip(self.commands, self.ports):
            subprocess.Popen(['bash', '-c', cmd])
        self.SCRIPT_RUNNING = True

    def _craft_commands(self, params):
        waf_pwd = find_waf_path("./")
        command = f'{waf_pwd} --run "linear-mesh'
        for key, val in params.items():
            command+=f" --{key}={val}"

        commands = []
        for p in self.ports:
            commands.append(command+f' --openGymPort={p}"')

        return commands

    def reset(self):
        obs = []
        for env in self.envs:
            obs.append(env.reset())

        return obs

    def step(self, actions):
        next_obs, reward, done, info = [], [], [], []

        for i, env in enumerate(self.envs):
            no, rew, dn, inf = env.step(actions[i].tolist())
            next_obs.append(no)
            reward.append(rew)
            done.append(dn)
            info.append(inf)

        return np.array(next_obs), np.array(reward), np.array(done), np.array(info)

    @property
    def observation_space(self):
        dim = repr(self.envs[0].observation_space).replace('(', '').replace(',)', '').split(", ")[2]
        return (self.no_threads, int(dim))

    @property
    def action_space(self):
        dim = repr(self.envs[0].action_space).replace('(', '').replace(',)', '').split(", ")[2]
        return (self.no_threads, int(dim))

    def close(self):
        time.sleep(5)
        for env in self.envs:
            env.close()
        # subprocess.Popen(['bash', '-c', "killall linear-mesh"])

        self.SCRIPT_RUNNING = False

    def __getattr__(self, attr):
        for env in self.envs:
            env.attr()
