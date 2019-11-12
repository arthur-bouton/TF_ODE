#!/usr/bin/python3
import warnings
warnings.simplefilter( action='ignore', category=FutureWarning )

import tensorflow as tf
import numpy as np
import random
import sys
import os
from protect_loop import Protect_loop
from DDPG_PER import DDPG

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'
sys.path.append( '../build' )
import rover_training_1_module


# Identifier name for the data:
#data_id = 'test1'

script_name = os.path.splitext( os.path.basename( __file__ ) )[0]

# Name of the file where to store network parameters:
#path_to_tf_model = '../training_data/' + script_name + '_' + data_id
path_to_tf_model = '../training_data/run_10/rover_training_1'

# Parameters for the training:
EP_MAX = 100000 # Maximal number of episodes for the training
ITER_PER_EP = 200 # Number of training iterations between each episode
S_DIM = 17 # Dimension of the state space
A_DIM = 2 # Dimension of the action space
STATE_SCALE = [ 90, 45, 25, 25, 45 ] + [ 100 ]*12 #+ [ 100 ]*4  # A scalar or a vector to normalize the state
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

with DDPG( S_DIM, A_DIM, STATE_SCALE, ACTION_SCALE, GAMMA, TAU, BUFFER_SIZE, MINIBATCH_SIZE, ACTOR_LR, CRITIC_LR, BETA_L2,
		   alpha_sampling=ALPHA_SAMPLING, beta_IS=BETA_IS,
		   summary_dir=SUMMARY_DIR, seed=SEED, single_thread=SINGLE_THREAD ) as ddpg :


	if len( sys.argv ) == 1 or sys.argv[1] != 'eval' :

		if len( sys.argv ) > 1 and sys.argv[1] == 'load' :
			if len( sys.argv ) > 2 :
				ddpg.load( sys.argv[2] )
			else :
				ddpg.load( path_to_tf_model )
		else :
			ddpg.save( path_to_tf_model )


		np.random.seed( SEED )

		n_ep = 0
		Li = 0

		import time
		start = time.time()

		with Protect_loop() as interruption :

			while not interruption() and n_ep < EP_MAX :


				# Do one trial:
				trial_experience = rover_training_1_module.trial( path_to_tf_model )

				if interruption() :
					break

				ddpg.replay_buffer.add( trial_experience )

				n_ep += 1


				# When there is enough samples, train the networks:
				if len( ddpg.replay_buffer ) >= ddpg.minibatch_size :
					Li = ddpg.train( ITER_PER_EP )

					ddpg.save( path_to_tf_model )


				# Evaluate the policy:
				#if ddpg.n_iter % ITER_PER_EP*10 == 0 and ddpg.n_iter > 0 :
					#print( 'It %i | Ep %i | Bs %i | Li %+8.4f => ' % ( ddpg.n_iter, n_ep, len( ddpg.replay_buffer ), Li ), end='', flush=True )
					#rover_training_1_module.eval( path_to_tf_model )
				#else :
				print( 'It %i | Ep %i | Bs %i | Li %+8.4f' % ( ddpg.n_iter, n_ep, len( ddpg.replay_buffer ), Li ), flush=True )


		end = time.time()
		print( 'Elapsed time: %.3f' % ( end - start ) )

		#answer = input( '\nSave network parameters as ' + path_to_tf_model + '? (y) ' )
		#if answer.strip() == 'y' :
			#ddpg.save( path_to_tf_model )
			#print( 'Parameters saved.' )
		#else :
			#answer = input( 'Where to store network parameters? (leave empty to discard data) ' )
			#if answer.strip() :
				#ddpg.save( answer )
				#print( 'Parameters saved as %s.' % answer )
			#else :
				#print( 'Data discarded.' )

	else :

		os.chdir( '../build' )
		import subprocess
		subprocess.call( './rover_training_1_exe display ' + path_to_tf_model )
