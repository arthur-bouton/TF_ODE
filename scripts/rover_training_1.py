#!/usr/bin/env python3
import sys

if len( sys.argv ) < 2 :
	print( 'Please specify the identification name of the training.', file=sys.stderr )
	exit( -1 )

import numpy as np
import random
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

sys.path.insert( 1, os.environ['TRAINING_SCRIPTS_DIR'] + 'MachineLearning' )
from looptools import Loop_handler
from SAC import SAC

sys.path.insert( 1, os.environ['BUILD_DIR'] )
import rover_training_1_module


from tensorflow import keras
from tensorflow.keras import layers


# Actor network:
def actor( s_dim, a_dim ) :

	states = keras.Input( shape=s_dim )

	x = layers.Dense( 512, activation='relu' )( states )
	x = layers.Dense( 512, activation='relu' )( x )

	mu = layers.Dense( a_dim, activation='linear' )( x )

	x = layers.Dense( 512, activation='relu' )( states )
	x = layers.Dense( 512, activation='relu' )( x )

	sigma = layers.Dense( a_dim, activation='softplus' )( x )

	return keras.Model( states, [ mu, sigma ] )


# Critic network:
def critic( s_dim, a_dim ) :

	states  = keras.Input( shape=s_dim )
	actions = keras.Input( shape=a_dim )

	x = layers.Concatenate()( [ states, actions ] )
	x = layers.Dense( 512, activation='relu' )( x )
	x = layers.Dense( 512, activation='relu' )( x )
	Q_value = layers.Dense( 1, activation='linear' )( x )

	return keras.Model( [ states, actions ], Q_value )



# Parameters for the training:
EP_MAX = 100000 # Maximal number of episodes for the training
ITER_PER_EP = 200 # Number of training iterations between each episode
hyper_params = {}
hyper_params['s_dim'] = 17 # Dimension of the state space
hyper_params['a_dim'] = 2 # Dimension of the action space
hyper_params['state_scale'] = [ 90, 45, 25, 25, 45 ] + ( [ 100 ]*3 + [ 30 ]*3 )*2 # A scalar or a vector to normalize the state
hyper_params['action_scale'] = [ 15, 25 ] # A scalar or a vector to scale the actions
hyper_params['gamma'] = 0.99 # Discount factor applied to the reward
#hyper_params['target_entropy'] = -2 # Desired target entropy of the policy
#hyper_params['tau'] = 5e-3 # Soft target update factor
hyper_params['buffer_size'] = 1e6 # Maximal size of the replay buffer
hyper_params['minibatch_size'] = 256 # Size of each minibatch
hyper_params['learning_rate'] = 1e-4 # Default learning rate used for all the networks
#hyper_params['actor_lr'] = 1e-4 # Learning rate of the actor network
#hyper_params['critic_lr'] = 2e-4 # Learning rate of the critic network
#hyper_params['alpha_lr'] = 1e-3 # Learning rate of the critic network
#hyper_params['alpha0'] = 0.1 # Initial value of the entropy temperature
hyper_params['seed'] = None # Random seed for the initialization of all random generators

sac = SAC( **hyper_params )



# Path to the directory in which to store the training data:
session_dir = sys.argv[1]


if len( sys.argv ) > 2 and sys.argv[2] == 'resume' :
	sac.load( session_dir )
	print( 'Training is resumed where it was left off.' )
	if not sac.load_replay_buffer( session_dir + '/replay_buffer.pkl' ) :
		print( 'Could not find %s: starting with an empty replay buffer.' % ( session_dir + '/replay_buffer.pkl' ) )
	sys.stdout.flush()
else :
	sac.actor.save( session_dir + '/actor' )


np.random.seed( hyper_params['seed'] )

n_ep = 0
LQ = 0

import time
start = time.time()

with Loop_handler() as interruption :

	while not interruption() and n_ep < EP_MAX :


		# Do one trial:
		trial_experience = rover_training_1_module.trial( session_dir + '/actor' )

		if interruption() :
			break

		# Store the experience:
		sac.replay_buffer.extend( trial_experience )

		n_ep += 1


		# Train the networks:
		LQ = sac.train( ITER_PER_EP )

		sac.actor.save( session_dir + '/actor' )

		print( 'It %i | Ep %i | Bs %i | LQ %+7.4f | temp %5.3f' %
			   ( sac.n_iter, n_ep, len( sac.replay_buffer ), LQ, float( sac.alpha ) ),
			   flush=True )


end = time.time()
print( 'Elapsed time: %.3fs  ' % ( end - start ) )

sac.save( session_dir )

answer = input( '\nSave the replay buffer as ' + session_dir + '/replay_buffer.pkl? (y) ' )
if answer.strip() == 'y' :
	sac.save_replay_buffer( session_dir + '/replay_buffer.pkl' )
	print( 'Replay buffer saved.' )
else :
	print( 'Experience discarded.' )
