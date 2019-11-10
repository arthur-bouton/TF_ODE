#!/usr/bin/python3
"""
Distributed Proximal Policy Optimization algorithm (DPPO) [cf. https://arxiv.org/abs/1707.06347 (OpenAI) and https://arxiv.org/abs/1707.02286 (DeepMind)]

Distribute workers on parallel threads to collect data and stop workers' roll-out when the amount of batch steps is reached.
Train networks on all collected data and then restart workers where they were but with the updated policy.

Author: Arthur Bouton [arthur.bouton@gadz.org]
"""

import tensorflow as tf
#import tensorflow_probability as tfp
import threading, queue
from numpy import *
import sys
import os
from pendulum import Pendulum


#sys.stdout = os.fdopen( sys.stdout.fileno(), 'w', 0 )

script_name = os.path.splitext( os.path.basename( __file__ ) )[0]



# DPPO parameters:
EP_MAX = 10000 # Maximum number of episodes for the training
EP_LEN = 100 # Maximum number of steps for a single episode
MIN_BATCH_SIZE = 100 # Minimum number of steps to compose a batch for updates
N_WORKER = 4 # Number of parallel workers
GAMMA = 0.7 # Reward discount factor
#LAMBDA = 0.95 # Return coefficient in the generalized advantage estimation
LAMBDA = 0 # Return coefficient in the generalized advantage estimation
LEARNING_RATE = 0.0001 # Learning rate
UPDATE_STEP = 10 # Number of iteration at each update
EPSILON = 0.2 # Surrogate objective clipping parameter
C1 = 1 # Critic loss coefficient in the objective function
C2 = 0.01 # Entropy coefficient in the objective function
S_DIM = 2 # Dimension of the state space
A_DIM = 1 # Dimension of the action space


class PPO( object ) :

	def __init__( self, Test_env, norm_vec=[1]*S_DIM ) :
		self.eval_env = Test_env()

		self.tf_states = tf.placeholder( tf.float32, [None, S_DIM], 'States' )
		self.tf_actions = tf.placeholder( tf.float32, [None, A_DIM], 'Actions' )
		self.tf_returns = tf.placeholder( tf.float32, [None, 1], 'Returns' )
		self.tf_values = tf.placeholder( tf.float32, [None, 1], 'Values' )

		# Normalization of the inputs:
		norm_w = tf.Variable( norm_vec )
		norm_states = tf.divide( self.tf_states, norm_w )

		# Monitor batch mean and variance:
		self.batch_mean, self.batch_var = tf.nn.moments( norm_states, [0] )

		# Critic:
		with tf.variable_scope( 'Critic' ) :

			n_units_1 = 20
			#w1 = tf.Variable( tf.truncated_normal( [S_DIM, n_units_1], stddev=10 ) )
			#b1 = tf.Variable( tf.constant( 0, tf.float32, [n_units_1] ) )
			wmax = 1
			bmax = 0.2
			w1 = tf.get_variable( 'Wc1', [S_DIM, n_units_1], tf.float32, tf.initializers.random_uniform( -wmax, wmax ) )
			b1 = tf.get_variable( 'Bc1', [n_units_1], tf.float32, tf.initializers.random_uniform( -bmax, bmax ) )
			o1 = tf.add( tf.matmul( norm_states, w1 ), b1 )
			a1 = tf.nn.relu( o1 )
			#a1 = tf.nn.tanh( o1 )

			#wv = tf.Variable( tf.truncated_normal( [n_units_1, 1], stddev=1 ) )
			#bv = tf.Variable( 10, dtype=tf.float32 )
			wmax = 1
			bmax = 10
			wv = tf.get_variable( 'Wv', [n_units_1, 1], tf.float32, tf.initializers.random_uniform( -wmax, wmax ) )
			bv = tf.get_variable( 'Bv', [1], tf.float32, tf.initializers.random_uniform( -bmax, bmax ) )
			self.value = tf.add( tf.matmul( a1, wv ), bv )

		# Actor:
		self.pi_mu, self.pi_sigma, stoch_pi, pi_params = self._build_actor_network( 'Pi', norm_states, trainable=True )
		_, _, oldpi, oldpi_params = self._build_actor_network( 'old_Pi', norm_states, trainable=False )
		self.sample_op = stoch_pi.sample( 1 )
		self.update_oldpi_op = [ oldp.assign( p ) for p, oldp in zip( pi_params, oldpi_params ) ]


		ratios = tf.divide( stoch_pi.prob( self.tf_actions ), oldpi.prob( self.tf_actions ) + 1e-5 )
		clipped_ratios = tf.clip_by_value( ratios, 1. - EPSILON, 1. + EPSILON )
		advantages = self.tf_returns - self.tf_values
		#advantages = self.tf_returns - self.value
		L_clip = tf.minimum( tf.multiply( ratios, advantages ), tf.multiply( clipped_ratios, advantages ) )

		L_vf = tf.squared_difference( self.tf_returns, self.value )
		#L_vf = tf.square( advantages )

		entropy = stoch_pi.entropy()

		self.L = -tf.reduce_mean( L_clip - C1*L_vf + C2*entropy )

		self.train_op = tf.train.AdamOptimizer( learning_rate=LEARNING_RATE ).minimize( self.L )


		self.sess = tf.Session()

		self.sess.run( tf.global_variables_initializer() )
		self.saver = tf.train.Saver()

	def _build_actor_network( self, name, inputs, trainable ) :
		with tf.variable_scope( name ) :

			n_units_1 = 40
			#w1 = tf.Variable( tf.truncated_normal( [S_DIM, n_units_1], stddev=10 ), trainable=trainable )
			#b1 = tf.Variable( tf.constant( 0, tf.float32, [n_units_1] ), trainable=trainable )
			wmax = 1
			bmax = 0.5
			w1 = tf.get_variable( 'Wa1', [S_DIM, n_units_1], tf.float32, tf.initializers.random_uniform( -wmax, wmax ), trainable=trainable )
			b1 = tf.get_variable( 'Ba1', [n_units_1], tf.float32, tf.initializers.random_uniform( -bmax, bmax ), trainable=trainable )
			o1 = tf.add( tf.matmul( inputs, w1 ), b1 )
			a1 = tf.nn.relu( o1 )
			#a1 = tf.nn.tanh( o1 )

			#wm = tf.Variable( tf.truncated_normal( [n_units_1, A_DIM], stddev=0.1 ), trainable=trainable )
			#bm = tf.Variable( tf.constant( 0, tf.float32, [A_DIM] ), trainable=trainable )
			wmax = 0.2
			bmax = 0.1
			wm = tf.get_variable( 'Wm', [n_units_1, A_DIM], tf.float32, tf.initializers.random_uniform( -wmax, wmax ), trainable=trainable )
			bm = tf.get_variable( 'Bm', [A_DIM], tf.float32, tf.initializers.random_uniform( -bmax, bmax ), trainable=trainable )
			om = tf.add( tf.matmul( a1, wm ), bm )


			n_units_1 = 20
			#w1 = tf.Variable( tf.truncated_normal( [S_DIM, n_units_1], stddev=10 ), trainable=trainable )
			#b1 = tf.Variable( tf.constant( 0, tf.float32, [n_units_1] ), trainable=trainable )
			wmax = 0.5
			bmax = 0.2
			ws1 = tf.get_variable( 'Ws1', [S_DIM, n_units_1], tf.float32, tf.initializers.random_uniform( -wmax, wmax ), trainable=trainable )
			bs1 = tf.get_variable( 'Bs1', [n_units_1], tf.float32, tf.initializers.random_uniform( -bmax, bmax ), trainable=trainable )
			os1 = tf.add( tf.matmul( inputs, ws1 ), bs1 )
			as1 = tf.nn.relu( os1 )

			#ws = tf.Variable( tf.truncated_normal( [n_units_1, A_DIM], stddev=0.1 ), trainable=trainable )
			#bs = tf.Variable( tf.constant( 2, tf.float32, [A_DIM] ), trainable=trainable )
			wmax = 0.5
			bc = 0.5
			bd = 0.2
			ws = tf.get_variable( 'Ws', [n_units_1, A_DIM], tf.float32, tf.initializers.random_uniform( -wmax, wmax ), trainable=trainable )
			bs = tf.get_variable( 'Bs', [A_DIM], tf.float32, tf.initializers.random_uniform( bc - bd, bc + bd ), trainable=trainable )
			os = tf.add( tf.matmul( as1, ws ), bs )

			mu = om
			#sigma = os
			#mu = tf.nn.tanh( om )
			sigma = tf.nn.softplus( os )

		norm_dist = tf.distributions.Normal( mu, sigma, allow_nan_stats=False )
		#norm_dist = tfp.distributions.Normal( mu, sigma, allow_nan_stats=False )

		#params = [ w1, b1, wm, bm, ws, bs ]
		params = [ w1, b1, wm, bm, ws, bs, ws1, bs1 ]
		return mu, sigma, norm_dist, params

	def update( self ) :
		global GLOBAL_BATCH_COUNTER, GLOBAL_WORKER_COUNTER, GLOBAL_EP
		niter = 0
		while not COORD.should_stop() :
			# Wait for a new batch of data:
			UPDATE_EVENT.wait()
			if ROLLING_EVENT.is_set() :	# If an interruption has been raised
				break

			niter += 1
			n_ep = GLOBAL_EP
			batch_size = GLOBAL_BATCH_COUNTER

			# Update old_Pi parameters:
			self.sess.run( self.update_oldpi_op )

			# Collect data from all workers:
			data = [ QUEUE.get() for _ in range( QUEUE.qsize() ) ]
			data = vstack( data )
			s, a, r, v = data[:,:S_DIM], data[:,S_DIM:S_DIM+A_DIM], data[:,-2:-1], data[:,-1:]

			#batch_stat = self.sess.run( [ self.batch_mean, self.batch_var ], {self.tf_states: s} )
			#print( ( 'BATCH MEAN:' + ' %f'*S_DIM + ' | BATCH VAR:' + ' %f'*S_DIM ) % tuple( tuple( batch_stat[0] ) + tuple( batch_stat[1] ) ) )

			#TODO BOOTSTRAP BATCH SAMPLES

			# Train the networks:
			for _ in range( UPDATE_STEP ) :
				_, L = self.sess.run( [ self.train_op, self.L ], {self.tf_states: s, self.tf_actions: a, self.tf_returns: r, self.tf_values: v} )

			GLOBAL_BATCH_COUNTER = 0
			GLOBAL_WORKER_COUNTER = -1
			UPDATE_EVENT.clear()
			# Launch the next roll-out:
			ROLLING_EVENT.set()

			self.eval_env.reset()
			stddev_m = 0
			for t in range( EP_LEN ) :
				a, stddev = GLOBAL_PPO.best_action( self.eval_env.get_obs(), return_stddev=True )
				stddev_m += stddev
				self.eval_env.step( a )
			stddev_m /= EP_LEN
			print( 'It %i | Ep %i | bs %i | Lt %+8.4f | Sd %+5.2f | ' % ( niter, n_ep, batch_size, L, stddev_m ), end='', flush=True )
			self.eval_env.print_eval()

		# Free blocked workers:
		ROLLING_EVENT.set()

	def stoch_action( self, s ) :
		a = self.sess.run( self.sample_op, {self.tf_states: s[newaxis, :] if s.ndim < 2 else s} )
		if s.ndim < 2 : a = a[0][0]
		return a[0]

	def best_action( self, s, return_stddev=False ) :
		mu, sigma = self.sess.run( [ self.pi_mu, self.pi_sigma ], {self.tf_states: s[newaxis, :] if s.ndim < 2 else s} )
		if s.ndim < 2 : mu = mu[0][0] ; sigma = sigma[0][0]
		if return_stddev :
			return mu, sigma
		return mu

	def get_value( self, s ) :
		v = self.sess.run( self.value, {self.tf_states: s[newaxis, :] if s.ndim < 2 else s} )
		if s.ndim < 2 : v = v[0][0]
		return v
	
	def save( self, filename ) :
		self.saver.save( self.sess, filename )
	
	def load( self, filename ) :
		self.saver.restore( self.sess, filename )
	
	#def __del__( self ) :
		#self.sess.close()


class Worker( object ) :

	def __init__( self, wid, Env ) :
		self.wid = wid
		self.env = Env()
		self.ppo = GLOBAL_PPO

	def compute_gae( self, next_value, rewards, masks, values, gamma, lambd=0.95 ) :
		values = values + [next_value]
		gae = 0
		returns = []
		for t in reversed( range( len( rewards ) ) ) :
			delta = rewards[t] + gamma*values[t+1]*masks[t] - values[t]
			gae = delta + gamma*lambd*masks[t]*gae
			returns.insert( 0, gae + values[t] )
		return returns

	def work( self ) :
		global GLOBAL_BATCH_COUNTER, GLOBAL_WORKER_COUNTER, GLOBAL_EP
		while not COORD.should_stop() :

			# Reset the environment and the data for a new episode:
			#s = self.env.reset()
			s = self.env.reset( random.uniform( -180, 180 ) )
			states, actions, rewards, values, masks = [], [], [], [], []

			#expl = False
			for t in range( EP_LEN ) :

				# Choose a random action and execute the next step:
				a = self.ppo.stoch_action( s )
				#if random.rand() < 0.1 :
					#if expl :
						#expl = False
					#else :
						#expl = True
						#a = random.uniform( -1, 1 )
				#if not expl :
					#a = self.ppo.best_action( s )
				s_next, r, ep_done, _ = self.env.step( a )

				# Store the data for the episode:
				states.append( s )
				actions.append( a )
				rewards.append( r )
				values.append( self.ppo.get_value( s ) )
				masks.append( int( not ep_done ) )
				s = s_next

				GLOBAL_BATCH_COUNTER += 1

				if t >= EP_LEN - 1 or GLOBAL_BATCH_COUNTER >= MIN_BATCH_SIZE or ep_done :

					# Compute the generalized advantage estimation:
					next_value = self.ppo.get_value( s_next )
					returns = self.compute_gae( next_value, rewards, masks, values, GAMMA, LAMBDA )

					# Store data in the queue:
					ss, sa, sr, sv = vstack( states ), vstack( actions ), vstack( returns ), vstack( values )
					QUEUE.put( hstack( ( ss, sa, sr, sv ) ) )
					states, actions, rewards, values, masks = [], [], [], [], []

					if GLOBAL_BATCH_COUNTER >= MIN_BATCH_SIZE :
						# Stop collecting data:
						if GLOBAL_WORKER_COUNTER < 0 :
							GLOBAL_WORKER_COUNTER = N_WORKER - 2
							# Wait for other workers to store their data:
							ROLLING_EVENT.clear()
						elif GLOBAL_WORKER_COUNTER > 0 :
							GLOBAL_WORKER_COUNTER -= 1
						else :
							# Stop the training:
							if GLOBAL_EP >= EP_MAX :
								COORD.request_stop()
							# When all workers have store their data, launch an update of the networks:
							UPDATE_EVENT.set()

					# Wait until networks are updated:
					ROLLING_EVENT.wait()

				if COORD.should_stop() or ep_done :
					break

			GLOBAL_EP += 1

		# Free the update thread:
		UPDATE_EVENT.set()


if __name__ == '__main__' :

	Env = Pendulum

	backup_file = './data/' + script_name + '_test1'

	GLOBAL_PPO = PPO( Env, [ pi, 2*pi ] )


	if len( sys.argv ) == 1 or sys.argv[1] != 'eval' :

		if len( sys.argv ) > 1 and sys.argv[1] == 'load' :
			GLOBAL_PPO.load( backup_file )

		workers = [ Worker( i, Env ) for i in range( N_WORKER ) ]

		# Set the events in order to start with a roll-out:
		UPDATE_EVENT, ROLLING_EVENT = threading.Event(), threading.Event()
		UPDATE_EVENT.clear()
		ROLLING_EVENT.set()

		GLOBAL_BATCH_COUNTER, GLOBAL_EP = 0, 0
		GLOBAL_WORKER_COUNTER = -1
		COORD = tf.train.Coordinator()

		with COORD.stop_on_exception() :

			# Create the queue to transfer data form workers to update thread:
			QUEUE = queue.Queue()

			# Launch the worker threads:
			threads = []
			for worker in workers:
				t = threading.Thread( target=worker.work, args=() )
				t.start()
				threads.append( t )

			# Add a PPO update thread:
			threads.append( threading.Thread( target=GLOBAL_PPO.update, ) )
			threads[-1].start()

			# Wait for the algorithm to reach the maximal number of iterations:
			COORD.join( threads )

		try :
			# In case of an SIGINT signal, wait again for the threads to finish nicely:
			COORD.join( threads )
		except KeyboardInterrupt :
			pass

		answer = input( '\nSave network parameters as ' + backup_file + '? (y) ' )
		if answer.strip() == 'y' :
			GLOBAL_PPO.save( backup_file )
			print( 'Parameters saved.' )
		else :
			answer = input( 'Where to store network parameters? (leave empty to discard data) ' )
			if answer.strip() :
				GLOBAL_PPO.save( answer )
				print( 'Parameters saved as %s.' % answer )
			else :
				print( 'Data discarded.' )
		exit( 0 )


	GLOBAL_PPO.load( backup_file )

	#writer = tf.summary.FileWriter( '/tmp/' + script_name + '_graph', GLOBAL_PPO.sess.graph )

	test_env = Env( 180, store_data=True, include_stddev=True )
	for t in range( EP_LEN ) :
		u, u_stddev = GLOBAL_PPO.best_action( test_env.get_obs(), return_stddev=True )
		test_env.step( u, u_stddev )
		#test_env.step( GLOBAL_PPO.stoch_action( test_env.get_obs() ) )
	print( 'Trial result: ', end='' )
	test_env.print_eval()
	test_env.plot3D( GLOBAL_PPO.best_action, GLOBAL_PPO.get_value, include_stddev=True )
	test_env.plot_trial( plot_stddev=True )
	test_env.show()
