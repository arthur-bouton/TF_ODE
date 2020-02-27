#!/usr/bin/python3
import warnings
warnings.simplefilter( action='ignore', category=FutureWarning )

import tensorflow as tf
import numpy as np
import os
import sys
#from DDPG_vanilla import DDPG
from DDPG_PER import DDPG
import pandas as pd
import shap
import pickle
import matplotlib.pyplot as plt

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'



#data_file = '../training_data/step_06_PER_1_no_sym_06only.dat'
data_file = '../training_data/step_06_PER_1_no_sym.dat'

model_path = '../training_data/step_06_PER_1_no_sym/selected/rover_training_1_0005'

shap_values_file = 'shape_values.pkl'



###################
# Import the data #
###################

state_1 = [ 'Steering angle', 'Roll angle', 'Pitch Angle', 'Boggie angle' ]
state_2 = [ 'Front $f_x$', 'Front $f_y$', 'Front $f_z$', 'Front $\\tau_x$', 'Front $\\tau_y$', 'Front $\\tau_z$',
            'Rear $f_x$', 'Rear $f_y$', 'Rear $f_z$', 'Rear $\\tau_x$', 'Rear $\\tau_y$', 'Rear $\\tau_z$' ]
actions = [ 'Steering rate', 'Boggie torque' ]
columns = [ 'Direction' ] + state_1 + state_2 + actions


df = pd.read_csv( data_file, sep=' ', header=None, names=columns )

# Convert every field to np.float64 and replace strings by np.nan:
df = df.apply( pd.to_numeric, errors='coerce' )

# Remove rows that contain at least one NaN:
df = df.dropna()

# Shuffle the rows:
#df = df.sample( frac=1 )

# Reset the index:
df.reset_index( drop=True, inplace=True )


X_data = df.iloc[:,:-2]



####################
# Import the model #
####################

# Parameters for the training:
EP_MAX = 100000 # Maximal number of episodes for the training
ITER_PER_EP = 200 # Number of training iterations between each episode
S_DIM = 17 # Dimension of the state space
A_DIM = 2 # Dimension of the action space
STATE_SCALE = [ 90, 45, 25, 25, 45 ] + [ 100 ]*3 + [ 30 ]*3 + [ 100 ]*3 + [ 30 ]*3 #+ [ 100 ]*4  # A scalar or a vector to normalize the state
ACTION_SCALE = [ 15, 20 ] # A scalar or a vector to scale the actions
GAMMA = 0.99 # Discount factor of the reward
TAU = 0.001 # Soft target update factor
BUFFER_SIZE = 100000 # Maximal size of the replay buffer
MINIBATCH_SIZE = 64 # Size of each minibatch
#ACTOR_LR = 0.0001 # Learning rate of the actor network
#CRITIC_LR = 0.001 # Learning rate of the critic network
ACTOR_LR = 1e-6 # Learning rate of the actor network
CRITIC_LR = 1e-5 # Learning rate of the critic network
BETA_L2 = 1e-6 # Ridge regularization coefficient
ALPHA_SAMPLING = 1 # Exponent interpolating between uniform sampling (0) and greedy prioritization (1)
BETA_IS = 0 # Exponent of the importance-sampling weights (if 0, no importance sampling)
#SUMMARY_DIR = '/tmp/' + script_name + '/' + data_id # Directory where to save summaries
SUMMARY_DIR = None # No summarie
SEED = None # Random seed for the initialization of all random generators
SINGLE_THREAD = False # Force the execution on a single core in order to have a deterministic behavior


if len( sys.argv ) > 1 and sys.argv[1] == 'load' :
	with open( shap_values_file, 'rb' ) as f :
		shap_values = pickle.load( f )
else :
	with DDPG( S_DIM, A_DIM, STATE_SCALE, ACTION_SCALE, GAMMA, TAU, BUFFER_SIZE, MINIBATCH_SIZE, ACTOR_LR, CRITIC_LR, BETA_L2,
			   alpha_sampling=ALPHA_SAMPLING, beta_IS=BETA_IS,
			   summary_dir=SUMMARY_DIR, seed=SEED, single_thread=SINGLE_THREAD ) as ddpg :

		ddpg.load_model( model_path )



		###########################
		# Compute the SHAP values #
		###########################

		explainer = shap.KernelExplainer( lambda x: ddpg.get_action( x )[np.newaxis,:] if x.shape[0] < 2 else ddpg.get_action( x ), X_data )
		shap_values = explainer.shap_values( X_data, l1_reg='aic' )


		with open( shap_values_file, 'wb' ) as f :
			pickle.dump( shap_values, f )



########################
# Plot the SHAP values #
########################

plt.figure( 'Importances' )
shap.summary_plot( shap_values, X_data, show=False )

plt.figure( 'Steering rate' )
shap.summary_plot( shap_values[0], X_data, show=False )

plt.figure( 'Boggie torque' )
shap.summary_plot( shap_values[1], X_data, show=False )

plt.show()
