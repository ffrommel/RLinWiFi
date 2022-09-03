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

# Run simulation in debug mode
debug_mode = False

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
        self.sent_mb_legacy = 0
        self.sent_mb_ax = 0
        self.speed_window = deque(maxlen=100)
        self.speed_window_legacy = deque(maxlen=100)
        self.speed_window_ax = deque(maxlen=100)
        self.step_time = None
        self.current_speed = 0
        self.current_speed_legacy = 0
        self.current_speed_ax = 0
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
            round_mb = info[0]
        except Exception as e:
            print(info)
            print(reward)
            raise e
        self.speed_window.append(round_mb)
        self.current_speed = np.mean(np.asarray(self.speed_window)/self.step_time)
        self.sent_mb += round_mb

        round_mb_legacy = info[1]
        self.speed_window_legacy.append(round_mb_legacy)
        self.current_speed_legacy = np.mean(np.asarray(self.speed_window_legacy)/self.step_time)
        self.sent_mb_legacy += round_mb_legacy

        round_mb_ax = info[2]
        self.speed_window_ax.append(round_mb_ax)
        self.current_speed_ax = np.mean(np.asarray(self.speed_window_ax)/self.step_time)
        self.sent_mb_ax += round_mb_ax

        CW = info[3]
        CW_ax = info[4]
        self.stations = info[5]
        fairness = info[6]

        if self.send_logs:
            self.experiment.log_metric("Round reward", np.mean(reward), step=step)
            self.experiment.log_metric("Per-ep reward", np.mean(cumulative_reward), step=step)
            #self.experiment.log_metric("Total megabytes sent", self.sent_mb, step=step)
            #self.experiment.log_metric("Round total megabytes sent", round_mb, step=step)
            #self.experiment.log_metric("Round legacy megabytes sent", round_mb_legacy, step=step)
            #self.experiment.log_metric("Round ax megabytes sent", round_mb_ax, step=step)
            self.experiment.log_metric("Chosen CW for legacy devices", CW, step=step)
            self.experiment.log_metric("Chosen CW for 802.11ax devices", CW_ax, step=step)
            self.experiment.log_metric("Station count", self.stations, step=step)
            self.experiment.log_metric("Current total throughput", self.current_speed, step=step)
            self.experiment.log_metric("Current legacy throughput", self.current_speed_legacy, step=step)
            self.experiment.log_metric("Current ax throughput", self.current_speed_ax, step=step)
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
        self.CW_ax = 16
        self.action = None              # For debug purposes

    def dry_run(self, agent, steps_per_ep):
        obs = self.env.reset()
        obs = self.preprocess(np.reshape(obs, (-1, len(self.env.envs), 1)))
        add_noise = False

        with tqdm.trange(steps_per_ep) as t:
            for step in t:
                self.actions = agent.act(np.array(obs, dtype=np.float32), add_noise)
                next_obs, reward, done, info = self.env.step(self.actions)

                obs = self.preprocess(np.reshape(next_obs, (-1, len(self.env.envs), 1)))

                if(any(done)):
                    break

    def eval(self, agent, simTime, stepTime, history_length, tags=None, parameters=None, experiment=None, dry_run=False):
        if not dry_run:
            agent.load()
        steps_per_ep = int(simTime/stepTime + history_length)

        logger = Logger(True, tags, parameters, experiment=experiment)
        try:
            logger.begin_logging(1, steps_per_ep, agent.noise.sigma, agent.noise.theta, stepTime)
        except  AttributeError:
            logger.begin_logging(1, steps_per_ep, None, None, stepTime)

        add_noise = False
        new_state = parameters['newState']

        obs_dim = 1
        time_offset = history_length//obs_dim*stepTime

        try:
            self.env.run()
        except AlreadyRunningException as e:
            pass

        cumulative_reward = 0
        reward = 0
        sent_mb = 0
        stas_length = 1

        obs = self.env.reset()
        if (new_state):
            p_col = obs[0][:-stas_length * 2]
            eff_stas_legacy = obs[0][history_length:][0]
            eff_stas_ax = obs[0][history_length + 1:][0]
        else:
            p_col = obs
            eff_stas_legacy = -1
            eff_stas_ax = -1
        obs = self.preprocess(np.reshape(p_col, (-1, len(self.env.envs), obs_dim)), eff_stas_legacy, eff_stas_ax)

        with tqdm.trange(steps_per_ep) as t:
            for step in t:
                self.debug = obs
                self.actions = agent.act(np.array(obs, dtype=np.float32), add_noise)
                next_obs, reward, done, info = self.env.step(self.actions)
                if (new_state):
                    p_col = next_obs[0][:-stas_length * 2]
                    eff_stas_legacy = next_obs[0][history_length:][0]
                    eff_stas_ax = next_obs[0][history_length + 1:][0]
                else:
                    p_col = next_obs
                    eff_stas_legacy = -1
                    eff_stas_ax = -1
                next_obs = self.preprocess(np.reshape(p_col, (-1, len(self.env.envs), obs_dim)), eff_stas_legacy, eff_stas_ax)

                cumulative_reward += np.mean(reward)

                if step>(history_length/obs_dim):
                    logger.log_round(obs, reward, cumulative_reward, info, agent.get_loss(), np.mean(obs, axis=0)[0], step)
                t.set_postfix(mb_sent=f"{logger.sent_mb:.2f} Mb", curr_speed=f"{logger.current_speed:.2f} Mbps")

                obs = next_obs

                if(any(done)):
                    break

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
        new_state = parameters['newState']

        obs_dim = 1
        time_offset = history_length//obs_dim*stepTime

        for i in range(EPISODE_COUNT):
            print(i)
            try:
                self.env.run()
            except AlreadyRunningException as e:
                pass

            if i>=EPISODE_COUNT*4/5:
                add_noise = False
                print("Turning off noise")

            cumulative_reward = 0
            reward = 0
            sent_mb = 0
            stas_length = 1

            obs = self.env.reset()
            if (new_state):
                p_col = obs[0][:-stas_length * 2]
                eff_stas_legacy = obs[0][history_length:][0]
                eff_stas_ax = obs[0][history_length + 1:][0]
            else:
                p_col = obs
                eff_stas_legacy = -1
                eff_stas_ax = -1
            obs = self.preprocess(np.reshape(p_col, (-1, len(self.env.envs), obs_dim)), eff_stas_legacy, eff_stas_ax)

            self.last_actions = None

            with tqdm.trange(steps_per_ep) as t: # tqdm: Instantly make your loops show a smart progress meter
                for step in t:
                    self.debug = obs

                    self.actions = agent.act(np.array(obs, dtype=np.float32), add_noise)
                    next_obs, reward, done, info = self.env.step(self.actions)
                    if (new_state):
                        p_col = next_obs[0][:-stas_length * 2]
                        eff_stas_legacy = next_obs[0][history_length:][0]
                        eff_stas_ax = next_obs[0][history_length + 1:][0]
                    else:
                        p_col = next_obs
                        eff_stas_legacy = -1
                        eff_stas_ax = -1
                    next_obs = self.preprocess(np.reshape(p_col, (-1, len(self.env.envs), obs_dim)), eff_stas_legacy, eff_stas_ax)

                    if self.last_actions is not None and step>(history_length/obs_dim) and i<EPISODE_COUNT-1:
                        agent.step(obs, self.actions, reward, next_obs, done, 2)

                    cumulative_reward += np.mean(reward)

                    self.last_actions = self.actions

                    if step>(history_length/obs_dim):
                        logger.log_round(obs, reward, cumulative_reward, info, agent.get_loss(), np.mean(obs, axis=0)[0], i*steps_per_ep+step)
                    t.set_postfix(mb_sent=f"{logger.sent_mb:.2f} Mb", curr_speed=f"{logger.current_speed:.2f} Mbps")

                    obs = next_obs

                    if(any(done)):
                        break

            self.env.close()
            if experimental:
                self.env = EnvWrapper(self.env.no_threads, **self.env.params)

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
        if debug_mode:
            command = f'{waf_pwd} --run linear-mesh --command-template="gdb --args %s'
        else:
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
            print("#####")
            print("env attr", env.attr())
            print("#####")
