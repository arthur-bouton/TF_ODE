#!/usr/bin/env python3
import warnings
warnings.simplefilter( action='ignore', category=FutureWarning )

import tensorflow as tf
import numpy as np
import random
import sys
import os
sys.path.insert( 1, 'MachineLearning' )
from looptools import Loop_handler
from DDPG_vanilla import DDPG
#from DDPG_PER import DDPG

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'
sys.path.append( '../build' )
import rover_training_1_module


sess = tf.Session()

from keras.layers import Dense
from keras import backend as K
K.set_session( sess )


# Actor network:
def actor( states, a_dim ) :

	x = Dense( 400, activation='relu' )( states )
	x = Dense( 400, activation='relu' )( x )
	action = Dense( a_dim, activation='tanh' )( x )

	return action


# Critic network:
def critic( states, actions ) :

	x = Dense( 400, activation='relu' )( tf.concat( [ states, actions ], 1 ) )
	x = Dense( 400, activation='relu' )( x )
	Q_value = Dense( 1, activation='linear' )( x )

	return Q_value


# Identifier name for the training data:
run_id = 'step_05_nobat_nosym_1'

script_name = os.path.splitext( os.path.basename( __file__ ) )[0]

# Name of the file where to store network parameters:
path_to_tf_model = '../training_data/' + run_id + '/' + script_name

# Parameters for the training:
EP_MAX = 100000 # Maximal number of episodes for the training
ITER_PER_EP = 200 # Number of training iterations between each episode
hyper_params = {}
hyper_params['s_dim'] = 17 # Dimension of the state space
hyper_params['a_dim'] = 2 # Dimension of the action space
hyper_params['state_scale'] = [ 90, 45, 25, 25, 45 ] + [ 100 ]*3 + [ 30 ]*3 + [ 100 ]*3 + [ 30 ]*3 # A scalar or a vector to normalize the state
hyper_params['action_scale'] = [ 15, 25 ] # A scalar or a vector to scale the actions
hyper_params['sess'] = sess # The TensorFlow session to use
hyper_params['actor_def'] = actor # The function defining the actor network
hyper_params['critic_def'] = critic # The function defining the critic network
hyper_params['gamma'] = 0.9 # Discount factor of the reward
hyper_params['tau'] = 1e-3 # Soft target update factor
hyper_params['buffer_size'] = 1e5 # Maximal size of the replay buffer
hyper_params['minibatch_size'] = 64 # Size of each minibatch
hyper_params['actor_lr'] = 1e-6 # Learning rate of the actor network
hyper_params['critic_lr'] = 1e-5 # Learning rate of the critic network
hyper_params['beta_L2'] = 0 # Ridge regularization coefficient
#hyper_params['alpha_sampling'] = 1 # Exponent interpolating between a uniform sampling (0) and a greedy prioritization (1) (DDPG_PER only)
#hyper_params['beta_IS'] = 1 # Exponent of the importance-sampling weights (if 0, no importance sampling) (DDPG_PER only)
hyper_params['summary_dir'] = None # No summaries
#hyper_params['summary_dir'] = '/tmp/' + script_name + '/' + data_id # Directory in which to save the summaries
hyper_params['seed'] = None # Seed for the initialization of all random generators
hyper_params['single_thread'] = False # Force the execution on a single core in order to have a deterministic behavior

ddpg = DDPG( **hyper_params )


if len( sys.argv ) == 1 or sys.argv[1] != 'eval' :

	if len( sys.argv ) > 1 and sys.argv[1] == 'load' :
		if len( sys.argv ) > 2 :
			path_to_tf_model = sys.argv[2]
		ddpg.load_model( path_to_tf_model )
		if not ddpg.load_replay_buffer( path_to_tf_model + '_replay_buffer.pkl' ) :
			print( 'Could not find %s: starting with an empty replay buffer.' % ( path_to_tf_model + '_replay_buffer.pkl' ) )
	else :
		ddpg.save_model( path_to_tf_model )


	np.random.seed( hyper_params['seed'] )

	n_ep = 0
	Li = 0

	import time
	start = time.time()

	with Loop_handler() as interruption :

		while not interruption() and n_ep < EP_MAX :


			# Do one trial:
			trial_experience = rover_training_1_module.trial( path_to_tf_model )

			if interruption() :
				break

			#ddpg.replay_buffer.extend( trial_experience )
			for transition in trial_experience :
				ddpg.replay_buffer.append( transition )

			n_ep += 1


			# When there is enough samples, train the networks:
			if len( ddpg.replay_buffer ) >= ddpg.minibatch_size :
				Li = ddpg.train( ITER_PER_EP )

				ddpg.save_model( path_to_tf_model )


			# Evaluate the policy:
			#if ddpg.n_iter % ITER_PER_EP*10 == 0 and ddpg.n_iter > 0 :
				#print( 'It %i | Ep %i | Bs %i | Li %+8.4f => ' % ( ddpg.n_iter, n_ep, len( ddpg.replay_buffer ), Li ), end='', flush=True )
				#rover_training_1_module.eval( path_to_tf_model )
			#else :
			print( 'It %i | Ep %i | Bs %i | Li %+8.4f' % ( ddpg.n_iter, n_ep, len( ddpg.replay_buffer ), Li ), flush=True )


	end = time.time()
	print( 'Elapsed time: %.3f' % ( end - start ), file=sys.stderr )

	answer = input( '\nSave the replay buffer as ' + path_to_tf_model + '_replay_buffer.pkl? (y) ' )
	if answer.strip() == 'y' :
		ddpg.save_replay_buffer( path_to_tf_model + '_replay_buffer.pkl' )
		print( 'Replay buffer saved.' )
	else :
		print( 'Experience discarded.' )

else :

	os.chdir( '../build' )
	import subprocess
	subprocess.call( './rover_training_1_exe display ' + path_to_tf_model )
