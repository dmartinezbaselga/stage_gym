from gym.spaces import space
import numpy as np
import gym
from gym import spaces
from numpy.core.fromnumeric import shape
import rospy
import rospkg
from std_srvs.srv import Empty
from typing import Any, Dict, Optional, Union
import os
from collections import namedtuple
from stage_gym.srv import *
import random
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

# from navrep.tools.rings import generate_rings
# from navrep.envs.ianenv import IANEnv
# from navrep.models.rnn import (reset_graph, sample_hps_params, MDNRNN,
#                                rnn_init_state, rnn_next_state, MAX_GOAL_DIST)
# from navrep.models.vae2d import ConvVAE
# from navrep.models.vae1d import Conv1DVAE
# from navrep.models.gpt import GPT, GPTConfig, load_checkpoint
# from navrep.models.gpt1d import GPT1D
# from navrep.models.vae1dlstm import VAE1DLSTM, VAE1DLSTMConfig
# from navrep.models.vaelstm import VAELSTM, VAELSTMConfig
# from navrep.tools.wdataset import scans_to_lidar_obs

ActionVel = namedtuple('ActionVel', ['v', 'w'])
RobotState = namedtuple('RobotState', ['x', 'y', 'theta', 'x_goal', 'y_goal', 'v', 'w', 'scan'])


# PUNISH_SPIN = True

# """ VM backends: VAE_LSTM, W backends: GPT, GPT1D, VAE1DLSTM """
# """ ENCODINGS: V_ONLY, VM, M_ONLY """
# _G = 2  # goal dimensions
# _A = 3  # action dimensions
# _RS = 5  # robot state
# _64 = 64  # ring size
# _L = 1080  # lidar size
# NO_VAE_VAR = True

# BLOCK_SIZE = 32  # sequence length (context)

# class EnvEncoder(object):
#     """ Generic class to encode the observations of an environment,
#     look at EncodedEnv to see how it is typically used """
#     def __init__(self,
#                  backend, encoding,
#                  rnn_model_path=os.path.expanduser("~/navrep/models/M/rnn.json"),
#                  rnn1d_model_path=os.path.expanduser("~/navrep/models/M/rnn1d.json"),
#                  vae_model_path=os.path.expanduser("~/navrep/models/V/vae.json"),
#                  vae1d_model_path=os.path.expanduser("~/navrep/models/V/vae1d.json"),
#                  gpt_model_path=os.path.expanduser("~/navrep/models/W/gpt"),
#                  gpt1d_model_path=os.path.expanduser("~/navrep/models/W/gpt1d"),
#                  vae1dlstm_model_path=os.path.expanduser("~/navrep/models/W/vae1dlstm"),
#                  vaelstm_model_path=os.path.expanduser("~/navrep/models/W/vaelstm"),
#                  gpu=False,
#                  encoder_to_share_model_with=None,  # another EnvEncoder
#                  ):
#         LIDAR_NORM_FACTOR = None
#         if backend == "GPT":
#             from navrep.scripts.train_gpt import _Z, _H
#         elif backend == "GPT1D":
#             from navrep.scripts.train_gpt1d import _Z, _H
#             from navrep.tools.wdataset import LIDAR_NORM_FACTOR
#         elif backend == "VAE1DLSTM":
#             from navrep.scripts.train_vae1dlstm import _Z, _H
#             from navrep.tools.wdataset import LIDAR_NORM_FACTOR
#         elif backend == "VAELSTM":
#             from navrep.scripts.train_vaelstm import _Z, _H
#         elif backend == "VAE_LSTM":
#             from navrep.scripts.train_vae import _Z
#             from navrep.scripts.train_rnn import _H
#         elif backend == "VAE1D_LSTM":
#             from navrep.scripts.train_vae1d import _Z
#             from navrep.scripts.train_rnn import _H
#             from navrep.scripts.train_vae1d import MAX_LIDAR_DIST as LIDAR_NORM_FACTOR
#         self._Z = _Z
#         self._H = _H
#         self.LIDAR_NORM_FACTOR = LIDAR_NORM_FACTOR
#         self.encoding = encoding
#         self.backend = backend
#         if self.encoding == "V_ONLY":
#             self.encoding_dim = _Z + _RS
#         elif self.encoding == "VM":
#             self.encoding_dim = _Z + _H + _RS
#         elif self.encoding == "M_ONLY":
#             self.encoding_dim = _H + _RS
#         else:
#             raise NotImplementedError
#         self.observation_space = spaces.Box(low=-np.inf, high=np.inf,
#                                             shape=(self.encoding_dim,), dtype=np.float32)
#         # V + M Models
#         if encoder_to_share_model_with is not None:
#             self.vae = encoder_to_share_model_with.vae
#             self.rnn = encoder_to_share_model_with.rnn
#         else:
#             # load world model
#             if self.backend == "VAE_LSTM":
#                 reset_graph()
#                 self.vae = ConvVAE(z_size=_Z, batch_size=1, is_training=False)
#                 self.vae.load_json(vae_model_path)
#                 if self.encoding in ["VM", "M_ONLY"]:
#                     hps = sample_hps_params. _replace(seq_width=_Z+_G, action_width=_A, rnn_size=_H)
#                     self.rnn = MDNRNN(hps, gpu_mode=gpu)
#                     self.rnn.load_json(rnn_model_path)
#             elif self.backend == "VAE1D_LSTM":
#                 reset_graph()
#                 self.vae = Conv1DVAE(z_size=_Z, batch_size=1, is_training=False)
#                 self.vae.load_json(vae1d_model_path)
#                 if self.encoding in ["VM", "M_ONLY"]:
#                     hps = sample_hps_params. _replace(seq_width=_Z+_G, action_width=_A, rnn_size=_H)
#                     self.rnn = MDNRNN(hps, gpu_mode=gpu)
#                     self.rnn.load_json(rnn1d_model_path)
#             elif self.backend == "GPT":
#                 mconf = GPTConfig(BLOCK_SIZE, _H)
#                 model = GPT(mconf, gpu=gpu)
#                 load_checkpoint(model, gpt_model_path, gpu=gpu)
#                 self.vae = model
#                 self.rnn = model
#             elif self.backend == "GPT1D":
#                 mconf = GPTConfig(BLOCK_SIZE, _H)
#                 model = GPT1D(mconf, gpu=gpu)
#                 load_checkpoint(model, gpt1d_model_path, gpu=gpu)
#                 self.vae = model
#                 self.rnn = model
#             elif self.backend == "VAELSTM":
#                 mconf = VAELSTMConfig(_Z, _H)
#                 model = VAELSTM(mconf, gpu=gpu)
#                 load_checkpoint(model, vaelstm_model_path, gpu=gpu)
#                 self.vae = model
#                 self.rnn = model
#             elif self.backend == "VAE1DLSTM":
#                 mconf = VAE1DLSTMConfig(_Z, _H)
#                 model = VAE1DLSTM(mconf, gpu=gpu)
#                 load_checkpoint(model, vae1dlstm_model_path, gpu=gpu)
#                 self.vae = model
#                 self.rnn = model
#             else:
#                 raise NotImplementedError
#         # other tools
#         self.rings_def = generate_rings(_64, _64)
#         self.viewer = None
#         # environment state variables
#         self.reset()

#     def reset(self):
#         if self.encoding in ["VM", "M_ONLY"]:
#             if self.backend in ["VAE_LSTM", "VAE1D_LSTM"]:
#                 self.state = rnn_init_state(self.rnn)
#             elif self.backend in ["GPT", "VAELSTM", "VAE1DLSTM", "GPT1D"]:
#                 self.gpt_sequence = []
#         self.lidar_z = np.zeros(self._Z)

#     def close(self):
#         if self.viewer is not None:
#             self.viewer.close()

#     def _get_last_decoded_scan(self):
#         obs_pred = self.vae.decode(self.lidar_z.reshape((1,self._Z)))
#         if self.backend in ["VAE1DLSTM", "GPT1D", "VAE1D_LSTM"]:
#             decoded_scan = (obs_pred * self.LIDAR_NORM_FACTOR).reshape((_L))
#         else:
#             rings_pred = obs_pred * self.rings_def["rings_to_bool"]
#             decoded_scan = self.rings_def["rings_to_lidar"](rings_pred, _L).reshape((_L))
#         return decoded_scan

#     def _encode_obs(self, obs, action):
#         """
#         obs is (lidar, other_obs)
#         where lidar is (time_samples, ray, channel)
#         and other_obs is (5,) - [goal_x, goal_y, vel_x, vel_y, vel_theta] all in robot frame

#         h is (32+2+512), i.e. concat[lidar_z, robotstate, h rnn state]
#         lidar_z is -inf, inf
#         h rnn state is ?
#         other_obs is -inf, inf
#         """
#         # convert lidar scan to obs
#         lidar_scan = obs[0]  # latest scan only obs (buffer, ray, channel)
#         lidar_scan = lidar_scan.reshape(1, _L).astype(np.float32)
#         lidar_mode = "scans" if "1D" in self.backend else "rings"
#         lidar_obs = scans_to_lidar_obs(lidar_scan, lidar_mode, self.rings_def, channel_first=False)
#         self.last_lidar_obs = lidar_obs  # for rendering purposes

#         # obs to z, mu, logvar
#         mu, logvar = self.vae.encode_mu_logvar(lidar_obs)
#         mu = mu[0]
#         logvar = logvar[0]
#         s = logvar.shape
#         if NO_VAE_VAR:
#             lidar_z = mu * 1.
#         else:
#             lidar_z = mu + np.exp(logvar / 2.0) * np.random.randn(*s)

#         # encode obs through V + M
#         self.lidar_z = lidar_z
#         if self.encoding == "V_ONLY":
#             encoded_obs = np.concatenate([self.lidar_z, obs[1]], axis=0)
#         elif self.encoding in ["VM", "M_ONLY"]:
#             # get h
#             if self.backend in ["VAE_LSTM", "VAE1D_LSTM"]:
#                 goal_z = obs[1][:2] / MAX_GOAL_DIST
#                 rnn_z = np.concatenate([lidar_z, goal_z], axis=-1)
#                 self.state = rnn_next_state(self.rnn, rnn_z, action, self.state)
#                 h = self.state.h[0]
#             elif self.backend in ["GPT", "VAELSTM", "VAE1DLSTM", "GPT1D"]:
#                 self.gpt_sequence.append(dict(obs=lidar_obs[0], state=obs[1][:2], action=action))
#                 self.gpt_sequence = self.gpt_sequence[:BLOCK_SIZE]
#                 h = self.rnn.get_h(self.gpt_sequence)
#             # encoded obs
#             if self.encoding == "VM":
#                 encoded_obs = np.concatenate([self.lidar_z, obs[1], h], axis=0)
#             elif self.encoding == "M_ONLY":
#                 encoded_obs = np.concatenate([obs[1], h], axis=0)
#         return encoded_obs

#     def _render_rings_polar(self, close, save_to_file=False):
#         if close:
#             self.viewer.close()
#             return
#         # rendering
#         if self.backend in ["VAE1DLSTM", "GPT1D", "VAE1D_LSTM"]:
#             return False
#         else:
#             last_rings_obs = self.last_lidar_obs.reshape((_64, _64, 1))
#             last_rings_pred = self.vae.decode(self.lidar_z.reshape((1,self._Z))).reshape((_64, _64, 1))
#             import matplotlib.pyplot as plt
#             plt.ion()
#             fig, (ax1, ax2) = plt.subplots(
#                 1, 2, subplot_kw=dict(projection="polar"), num="rings"
#             )
#             ax1.clear()
#             ax2.clear()
#             if self.viewer is None:
#                 self.rendering_iteration = 0
#             self.viewer = fig
#             self.rings_def["visualize_rings"](last_rings_obs, scan=None, fig=fig, ax=ax1)
#             self.rings_def["visualize_rings"](last_rings_pred, scan=None, fig=fig, ax=ax2)
#             ax1.set_ylim([0, 10])
#             ax1.set_title("ground truth")
#             ax2.set_ylim([0, 10])
#             ax2.set_title("lidar reconstruction")
#             # rings box viz
#             fig2, (ax1, ax2) = plt.subplots(1, 2, num="2d")
#             ax1.clear()
#             ax2.clear()
#             ax1.imshow(np.squeeze(last_rings_obs), cmap=plt.cm.Greys)
#             ax2.imshow(np.squeeze(last_rings_pred), cmap=plt.cm.Greys)
#             ax1.set_title("ground truth")
#             ax2.set_title("lidar reconstruction")
#             # update
#             plt.pause(0.01)
#             self.rendering_iteration += 1
#             if save_to_file:
#                 fig.savefig(
#                     "/tmp/encodedenv_polar{:04d}.png".format(self.rendering_iteration))
#                 fig2.savefig(
#                     "/tmp/encodedenv_box{:04d}.png".format(self.rendering_iteration))

#     def _render_rings(self, close, save_to_file=False):
#         if close:
#             self.viewer.close()
#             return
#         # rendering
#         if self.backend in ["VAE1DLSTM", "GPT1D", "VAE1D_LSTM"]:
#             return False
#         else:
#             last_rings_obs = self.last_lidar_obs.reshape((_64, _64))
#             last_rings_pred = self.vae.decode(self.lidar_z.reshape((1,self._Z))).reshape((_64, _64))
#             # Window and viewport size
#             ring_size = _64  # grid cells
#             padding = 4  # grid cells
#             grid_size = 1  # px per grid cell
#             WINDOW_W = (2 * ring_size + 3 * padding) * grid_size
#             WINDOW_H = (1 * ring_size + 2 * padding) * grid_size
#             VP_W = WINDOW_W
#             VP_H = WINDOW_H
#             from gym.envs.classic_control import rendering
#             import pyglet
#             from pyglet import gl
#             # Create viewer
#             if self.viewer is None:
#                 self.viewer = rendering.Viewer(WINDOW_W, WINDOW_H)
#                 self.rendering_iteration = 0
#             # Render in pyglet
#             win = self.viewer.window
#             win.switch_to()
#             win.dispatch_events()
#             win.clear()
#             gl.glViewport(0, 0, VP_W, VP_H)
#             # colors
#             bgcolor = np.array([0.4, 0.8, 0.4])
#             # Green background
#             gl.glBegin(gl.GL_QUADS)
#             gl.glColor4f(bgcolor[0], bgcolor[1], bgcolor[2], 1.0)
#             gl.glVertex3f(0, VP_H, 0)
#             gl.glVertex3f(VP_W, VP_H, 0)
#             gl.glVertex3f(VP_W, 0, 0)
#             gl.glVertex3f(0, 0, 0)
#             gl.glEnd()
#             # rings - observation
#             w_offset = 0
#             for rings in [last_rings_obs, last_rings_pred]:
#                 for i in range(ring_size):
#                     for j in range(ring_size):
#                         cell_color = 1 - rings[i, j]
#                         cell_y = (padding + i) * grid_size  # px
#                         cell_x = (padding + j + w_offset) * grid_size  # px
#                         gl.glBegin(gl.GL_QUADS)
#                         gl.glColor4f(cell_color, cell_color, cell_color, 1.0)
#                         gl.glVertex3f(cell_x+       0,  cell_y+grid_size, 0)  # noqa
#                         gl.glVertex3f(cell_x+grid_size, cell_y+grid_size, 0)  # noqa
#                         gl.glVertex3f(cell_x+grid_size, cell_y+        0, 0)  # noqa
#                         gl.glVertex3f(cell_x+        0, cell_y+        0, 0)  # noqa
#                         gl.glEnd()
#                 w_offset += ring_size + padding
#             if save_to_file:
#                 pyglet.image.get_buffer_manager().get_color_buffer().save(
#                     "/tmp/encodeder_rings{:04d}.png".format(self.rendering_iteration))
#             # actualize
#             win.flip()
#             self.rendering_iteration += 1
#             return self.viewer.isopen

# class StageGymNavrep(gym.GoalEnv):
#   """
#   Custom Environment that follows gym interface.
#   """

#   def __init__(self, backend, encoding,
#                  silent=False, max_episode_length=1000, collect_trajectories=False,
#                  gpu=False, encoder=None):
#     if encoder is None:
#       encoder = EnvEncoder(backend, encoding)
#     self.encoder = encoder
#     self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
#     self.observation_space = self.encoder.observation_space
#     super(StageGymNavrep, self).__init__()
#     print("ROS node init, waiting for service to be advertised")
#     rospy.wait_for_service('/gym_step')
#     print("Service is advertised")

#   def reset(self):
#     """
#     Important: the observation must be a numpy array
#     :return: (np.array) 
#     """
#     rospy.wait_for_service('/gym_reset')
#     try:
#         self.encoder.reset()
#         gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset_lidar)
#         resp = gym_reset(False)        
#         h = self.encoder._encode_obs((np.array(resp.lidar_measurements), np.array(resp.robot_state)), np.array([0,0,0]))
#         return h
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

#   def step(self, action):
#     rospy.wait_for_service('/gym_step')
#     try:
#         gym_step = rospy.ServiceProxy('/gym_step', DQN_gym_lidar)
#         action = np.array([action[0], action[1], 0.])  # no rotation
#         resp = gym_step(action[0], action[1], action[2])
#         h = self.encoder._encode_obs((np.array(resp.lidar_measurements), np.array(resp.robot_state)), action)
#         return h, resp.reward, resp.done, {}
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

#   def render(self, mode='console'):
#     print("RENDER ENV->This should never be called")

#   def close(self):
#     pass
    

class StageGymRandom(gym.Env):
    """
    Custom Environment that follows gym interface.
    """

    def __init__(self, max_v = 0.65, max_w = np.pi, num_v = 6, num_w = 9, robot_state_len = 4, lidar_samples = 360, 
                max_lidar_range=30.0):

        self.velocities_actions = self.get_possible_velocities(max_v, max_w, num_v, num_w)
        self.action_space = spaces.Discrete(len(self.velocities_actions))
        low_list = np.zeros((robot_state_len + lidar_samples,))
        low_list[:robot_state_len] = -np.inf
        high_list = np.ones((robot_state_len + lidar_samples,))*max_lidar_range
        high_list[:robot_state_len] = np.inf
        self.observation_space = spaces.Box(low=-low_list, high=high_list, shape=(robot_state_len + lidar_samples,))
        # super(StageGym, self).__init__()
        # print("ROS node init, waiting for service to be advertised")
        # rospy.wait_for_service('/gym_step')
        # print("Service is advertised")

    def get_possible_velocities(self, max_v, max_w, num_v, num_w):
        vel_list = []
        for v in np.linspace(0.0, max_v, num_v):
            for w in np.linspace(-max_w, max_w, num_w):
                vel_list.append(ActionVel(v,w))
        return vel_list

    def reset(self):
        """
        Important: the observation must be a numpy array
        :return: (np.array) 
        """
        # rospy.wait_for_service('/gym_reset')
        try:
            # gym_reset = rospy.ServiceProxy('/gym_reset', DQN_gym_reset_lidar)
            # resp = gym_reset(False)        
            # h = self.encoder._encode_obs((np.array(resp.lidar_measurements), np.array(resp.robot_state)), np.array([0,0,0]))
            # return h
            return self.observation_space.sample()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def step(self, action):
        # rospy.wait_for_service('/gym_step')
        try:
            # gym_step = rospy.ServiceProxy('/gym_step', DQN_gym_lidar)
            # action = np.array([action[0], action[1], 0.])  # no rotation
            # resp = gym_step(action[0], action[1], action[2])
            # h = self.encoder._encode_obs((np.array(resp.lidar_measurements), np.array(resp.robot_state)), action)
            # return h, resp.reward, resp.done, {}
            final =  np.random.choice([0,1], p=[0.9, 0.1])
            return self.observation_space.sample(), float(final), final, {}
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def render(self, mode='console'):
        print("RENDER ENV->This should never be called")

    def close(self):
        pass

class StageGym(gym.Env):
    """
    Custom Environment that follows gym interface.
    """

    def __init__(self, n_obstacles, max_v = 0.65, max_w = np.pi, num_v = 6, num_w = 9, robot_state_len = 4, lidar_samples = 360, 
                max_lidar_range=30.0):

        self.velocities_actions = self.get_possible_velocities(max_v, max_w, num_v, num_w)
        self.action_space = spaces.Discrete(len(self.velocities_actions))
        low_list = np.zeros((robot_state_len + lidar_samples,))
        low_list[:robot_state_len] = -np.inf
        high_list = np.ones((robot_state_len + lidar_samples,))*max_lidar_range
        high_list[:robot_state_len] = np.inf
        self.observation_space = spaces.Box(low=-low_list, high=high_list, shape=(robot_state_len + lidar_samples,))
        self.n_obstacles = n_obstacles
        self.robot_state = RobotState()
        random.seed(42)
        self.amcl_pub = rospy.Publisher('/robot_0/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.amcl_sub = rospy.Subscriber('/robot_0/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        rospy.init_node('agent_node')
        super(StageGym, self).__init__()
        print("ROS node init, waiting for service to be advertised")
        rospy.wait_for_service('/step_stage')
        print("Service is advertised")
        try:
            stage_stop = rospy.ServiceProxy('/step_stage', step_by_step)
            _ = stage_stop(True)        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def amcl_callback(self, msg):
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        _, _, theta = tf.transformations.euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_state.theta = theta

    def get_possible_velocities(self, max_v, max_w, num_v, num_w):
        vel_list = []
        for v in np.linspace(0.0, max_v, num_v):
            for w in np.linspace(-max_w, max_w, num_w):
                vel_list.append(ActionVel(v,w))
        return vel_list

    def get_t(self):
        t_ini = random.uniform(0, 500)
        t = random.uniform(20, 60)
        return t_ini, t

    def get_velocities(self):
        vx = random.uniform(0.1, 0.5)
        vth = random.uniform(0.1, 0.5)
        if random.random() >= 0.95:
            vx = 0.0
            vth = 0.0
        return vx, vth
    
    def get_position(self, is_active, x_list, y_list):
        done = False
        while not done:
            done = True
            if is_active:
                x = random.uniform(2.0, 3.0)
                y = random.uniform(2.0, 3.0)
            else:
                x = random.uniform(-3.0, 3.0)
                y = random.uniform(-3.0, 3.0)                
            for i_agent in range(len(x_list)):
                if np.sqrt((x_list-x)**2 + (y_list-y)**2) <= 1.2:
                    done = False
        x_list.append(x)
        y_list.append(y)
        theta = random.uniform(-np.pi, np.pi)
        return x, y, theta
    
    def get_goal(self, x, y):
        done = False
        while not done:
            done = True
            xgoal = random.uniform(0.0, -3.0)
            ygoal = random.uniform(0.0, -3.0)
            if np.sqrt((xgoal-x)**2 + (ygoal-y)**2) <= 6.0:
                done = False
        return xgoal, ygoal

    def reset(self):
        """
        Important: the observation must be a numpy array
        :return: (np.array) 
        """
        rospy.wait_for_service('/teleport_stage')
        try:
            stage_teleport = rospy.ServiceProxy('/teleport_stage', teleport)
            teleport_request = teleportRequest()
            x, y, theta = self.get_position(True, [], [])
            teleport_request.x.append(x)
            teleport_request.y.append(y)
            teleport_request.z.append(0.0)
            teleport_request.angle.append(theta)
            amcl_msg = PoseWithCovarianceStamped()
            amcl_msg.header.frame_id = "map"
            amcl_msg.header.stamp = rospy.Time.now()
            amcl_msg.pose.pose.position.x = x
            amcl_msg.pose.pose.position.y = y
            quaternion = tf.transformations.quaternion_from_euler(0,0,theta)
            amcl_msg.pose.pose.orientation = quaternion
            amcl_msg.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            xg, yg = self.get_goal(x, y)
            self.robot_state.x_goal = xg
            self.robot_state.y_goal = yg
            for i_agent in range(self.n_obstacles):
                x, y, theta = self.get_position(False, teleport_request.x, teleport_request.y)
                vx, vth = self.get_velocities()
                t_ini, t = self.get_t
                teleport_request.x.append(x)
                teleport_request.y.append(y)
                teleport_request.z.append(0.0)
                teleport_request.angle.append(theta)
                teleport_request.vx.append(vx)
                teleport_request.vy.append(vth)
                teleport_request.t_ini.append(t_ini)
                teleport_request.t.append(t)
            teleport_request.x.append(xg)
            teleport_request.y.append(yg)
            teleport_request.z.append(0.0)
            teleport_request.angle.append(0.0)
            stage_teleport(teleport_request)
            self.amcl_pub.publish(amcl_msg)
            self.amcl_callback(amcl_msg)
            # Borrar colisiones despues del loop_rate.sleep
            # Subscriber del laser y colisiones

            # resp = gym_reset(False)        
            # h = self.encoder._encode_obs((np.array(resp.lidar_measurements), np.array(resp.robot_state)), np.array([0,0,0]))
            # return h
            return self.observation_space.sample()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def step(self, action):
        # rospy.wait_for_service('/gym_step')
        try:
            # gym_step = rospy.ServiceProxy('/gym_step', DQN_gym_lidar)
            # action = np.array([action[0], action[1], 0.])  # no rotation
            # resp = gym_step(action[0], action[1], action[2])
            # h = self.encoder._encode_obs((np.array(resp.lidar_measurements), np.array(resp.robot_state)), action)
            # return h, resp.reward, resp.done, {}
            final =  np.random.choice([0,1], p=[0.9, 0.1])
            return self.observation_space.sample(), float(final), final, {}
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def render(self, mode='console'):
        print("RENDER ENV->This should never be called")

    def close(self):
        pass