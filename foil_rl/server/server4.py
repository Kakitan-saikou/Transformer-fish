import os
import pickle
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import tensorflow as tf
import numpy as np
from agent_TD3 import Agent
from utils import KalmanFilter, NoneFilter
from datetime import datetime
import argparse
import socket


class Server():
    def __init__(self, environment, filter, explore_noise, one_action,*args , **kwargs):
        self.environment = environment
        self.one_action = one_action
        self.state_dim = 6
        if self.one_action:
            self.action_dim = 1
        else:
            self.action_dim = 2
        self.agent = Agent(self.state_dim, self.action_dim, explore_noise = explore_noise)
        self._stamp("Environment: " + environment)
        self._stamp("Using Exploration Noise: " + explore_noise)
        self.state_record = [] # filtered.
        self.unfiltered_state_record = []
        self.action_record = []
        self.reward=[]
        self.state_mean = np.zeros((1,self.state_dim))
        self.hostname = socket.gethostname()
        self.dns_resolved_addr = socket.gethostbyname(self.hostname)
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.serversocket.bind((self.dns_resolved_addr, 8040))
        self.server = SimpleXMLRPCServer(("0.0.0.0", 8040))
            # self.server = SimpleXMLRPCServer(("localhost", 8080),logRequests=False)
        self._register()
        self.filter_name = filter
        if self.filter_name == "None":
            self.filter = NoneFilter()
            self._stamp("Using No Filter")
        else:
            raise NotImplementedError
        self.save_model_dir = "save4"
        self.save_data_dir = "save_data4"
        self.save_eval_dir = "save_eval4"
        self.save_agent_model = "save"

    def _stamp(self, string):
        time = "UTC " + datetime.utcnow().isoformat(sep=" ", timespec="milliseconds") + " "
        print(time + string, flush = True)

    def _register(self):
        self.server.register_function(self._init, "init")
        self.server.register_function(self._start_episode, "start_episode")
        self.server.register_function(self._request_stochastic_action, "request_stochastic_action")
        self.server.register_function(self._request_deterministic_action, "request_deterministic_action")
        self.server.register_function(self._train, "train")
        self.server.register_function(self._save, "save")
        self.server.register_function(self._save_eval, "save_eval")
        self.server.register_function(self._restore, "restore")
        self.server.register_function(self._save_ANN_weights, "save_ANN_weights")
        self.server.register_function(self._receive_reward, "receive_reward")

    def _reward_func(self, reward):
        reward_raw = reward*10
        # reward = -np.sign(next_state[0,1]) * (next_state[0,1])**2
        # reward = next_state[0,1]*10
        # reward_raw = - next_state[0,1] - np.pi * (1/8) * 0.0097 * (3.66**3) * np.sum((np.abs(action)**3))
        return reward_raw

    def _receive_reward(self, raw_reward):
        self.reward.append(float(raw_reward))
        self._stamp("Reward: {:.4f} ".format(float(raw_reward)))

    def _converter(self, signal):
        """
        convert the signal from sensor to forces and moment.
        Args: 
            signal list: 6*n, 
        Return:
            states (ndarray): (n,2), dimensionless C_lift and C_drag 
        """
        non_dimensional = 0.5 * 1000 * 0.2**2 * 2 * 20 * 0.0254**2
        calmat = np.array([
        [-0.03424, 0.00436,  0.04720,  -1.87392,  0.00438,  1.89084],
        [-0.02353,  2.19709,  -0.02607,  -1.08484,  0.03776,  -1.09707],
        [3.32347,  -0.13564,  3.36623,  -0.09593,  3.35105,  -0.15441],
        [-0.00778,  1.04073,  -3.83509,  -0.40776,  3.81653,  -0.69079],
        [4.36009,  -0.17486,  -2.26568,  0.94542,  -2.18742,  -0.79758],
        [0.03483,  -2.32692,  0.02473,  -2.30260,  0.01156,  -2.33458]])

        signal_array = np.mean(np.array(signal).reshape((6,-1)), axis = 1, keepdims = True)
        calforce = calmat @ signal_array
        lift = calforce[0:1,:]*4.44822 / non_dimensional # C_l
        drag = calforce[1:2,:]*4.44822 / non_dimensional # C_d
        state = np.concatenate([lift, drag], axis = 0) #(state_dim, 1)

        return state.transpose() #(1, state_dim)

    def _init(self, episode_count):
        try:
            self._restore(episode_count)
            self._stamp("Restored from "+str(episode_count)+" episode")
            return True

        except Exception:
            self.agent.reset_agent()
            self._stamp("Initialized!")
            return False

    def _start_episode(self,raw_data):
        # raw_data for calibrating
        self.state_record = []
        self.unfiltered_state_record = []
        self.action_record = []
        self.reward=[]
        self.agent.reset_episode()
        self.filter.reset()
        self._stamp("Episode Start!")
        return True

    def _request_action(self, raw_data, stochastic):
        if self.environment == "CFD":
            calibrated_state = np.asfarray(raw_data.split("_"),float)[None,:]
        else:
            raise NotImplementedError
        filtered_state = self.filter.estimate(calibrated_state)
        action = self.agent.get_action(filtered_state, stochastic = stochastic) #(1, action_dim)
        
        if self.one_action:
            action = np.concatenate((action, - action), axis=1)

        self.unfiltered_state_record.append(calibrated_state)
        self.state_record.append(filtered_state)
        self.action_record.append(action)
        self._stamp("Action: {:.4f} {:.4f} theta: {:.4f} y: {:.4f}".format(
                    action[0,0],
                    action[0,1],
					80*action[0,0],
					80+80*action[0,1]
					))


        if self.environment == "CFD":
            raw_action = str(action[0,0])+"_"+str(action[0,1])
        else:
            raise NotImplementedError
        return raw_action

    def _request_stochastic_action(self, raw_data):
        return self._request_action(raw_data, True)

    def _request_deterministic_action(self, raw_data):
        return self._request_action(raw_data, False)
       
    def _train(self, steps):
        if not os.path.exists(self.save_data_dir):
            os.mkdir(self.save_data_dir)
        np.savez(self.save_data_dir + "/data_{}.npz".format(self.agent.episode_count), 
                    state = np.array(self.state_record), 
                    unfiltered_state = np.array(self.unfiltered_state_record), 
                    action = np.array(self.action_record))
        record_length = len(self.state_record)
        reward_length = len(self.reward)
        action_length = len(self.action_record)
        self._stamp("Length of Record: " + str(record_length))
        self._stamp("Length of Reward: " + str(reward_length))
        self._stamp("Length of Action: " + str(action_length))
        self.agent.saver.restore(self.agent.sess, self.save_agent_model + "/ANN.ckpt")
        pickle_in = open(self.save_agent_model + "/ANN.pickle", "rb")
        self.agent.replay_buffer = pickle.load(pickle_in)
        self._stamp("Restored ANN!")
        for i in range(0, record_length - 1):
            reward = self._reward_func(self.reward[i+1])
            self.agent.replay_buffer.store(self.state_record[i],
                                        self.action_record[i][0,:self.action_dim],
                                        reward,
                                        self.state_record[i+1],
                                        0)
        self._stamp("Training Start!")
        for i in range(steps):
            self.agent.train_iter()
        self._stamp("Training End!")
        return True

    def _save_eval(self, episode_count, rep_count):
        if not os.path.exists(self.save_eval_dir):
            os.mkdir(self.save_eval_dir)
        np.savez(self.save_eval_dir + "/data_{}_{}.npz".format(episode_count, rep_count), 
            state = np.array(self.state_record), 
            unfiltered_state = np.array(self.unfiltered_state_record), 
            action = np.array(self.action_record))
        return True

    def _save(self, dummy = None):
        if not os.path.exists(self.save_model_dir):
            os.mkdir(self.save_model_dir)
        self.agent.saver.save(self.agent.sess, self.save_model_dir + "/{}.ckpt".format(self.agent.episode_count))
        
        pickle_out = open(self.save_model_dir + "/{}.pickle".format(self.agent.episode_count),"wb")
        pickle.dump(self.agent.replay_buffer, pickle_out)
        pickle_out.close()
        self._stamp("Saved Episode {}!".format(self.agent.episode_count))
        return True

    def _save_ANN_weights(self, dummy=None):
        if not os.path.exists(self.save_agent_model):
            os.mkdir(self.save_agent_model)
        self.agent.saver.save(self.agent.sess, self.save_agent_model + "/ANN.ckpt")

        pickle_out = open(self.save_agent_model + "/ANN.pickle", "wb")
        pickle.dump(self.agent.replay_buffer, pickle_out)
        pickle_out.close()
        self._stamp("saved ANN!")
        return True

    def _restore(self,episode_count):
        self.agent.saver.restore(self.agent.sess, self.save_model_dir + "/{}.ckpt".format(episode_count))
        pickle_in = open(self.save_model_dir + "/{}.pickle".format(episode_count),"rb")
        self.agent.replay_buffer = pickle.load(pickle_in)
        self.agent.episode_count = episode_count
        self._stamp("Restored from Episode {}!".format(episode_count))
        return True

    def start_server(self):
        self._stamp("Server Listening...")
        print("action_dimension = "+str(self.action_dim))
        self.server.serve_forever()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="server for experiments and CFD")
    parser.add_argument("-env", "--environment", choices=["CFD", "Experiment"], default="CFD")
    parser.add_argument("-fil", "--filter", choices=["None", "Kalman"], default="None")
    parser.add_argument("-exn", "--explore_noise", choices=["Gaussian", "Process"], default="Gaussian")
    parser.add_argument("-one", "--one_action", action='store_true')
    args = parser.parse_args()
    myserver = Server(**args.__dict__)
    myserver.start_server()
