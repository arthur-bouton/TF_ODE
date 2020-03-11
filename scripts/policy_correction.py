#!/usr/bin/python
import pandas as pd
import numpy as np
from gmr import GMM
import pickle
import sys
import matplotlib.pyplot as plt


gmm_file = 'gmm.pkl'

trial_file = 'trial.dat'

model_data_file_list = []
model_data_file_list.append( 'transitions.dat' )



state = [ 'Direction', 'Steering angle', 'Roll angle', 'Pitch Angle', 'Boggie angle',
		  'Front $f_x$', 'Front $f_y$', 'Front $f_z$',
		  'Front $\\tau_x$','Front $\\tau_y$', 'Front $\\tau_z$',
		  'Rear $\\tau_x$', 'Rear $\\tau_y$', 'Rear $\\tau_z$' ]
actions = [ 'Steering rate', 'Boggie torque' ]

state_1 = [ name + ' s1' for name in state ]
state_2 = [ name + ' s2' for name in state ]
columns = state_1 + actions + state_2

dim_s = len( state )
dim_a = len( actions )
dim_t = len( columns )



if len( sys.argv ) > 1 and ( sys.argv[1] == 'train' or sys.argv[1] == 'score' ) :

	#######################
	# Load the model data #
	#######################

	df_list = []
	for f in model_data_file_list :
		df_list.append( pd.read_csv( f, sep=' ', header=None, names=columns ) )
	df = pd.concat( df_list )

	# Convert every field to np.float64 and replace strings by np.nan:
	df = df.apply( pd.to_numeric, errors='coerce' )

	# Remove rows that contain at least one NaN:
	df = df.dropna()

	# Shuffle the rows:
	#df = df.sample( frac=1 )

	# Reset the index:
	df.reset_index( drop=True, inplace=True )


	data = df.to_numpy()

	print( 'Number of transitions in the data: %d' % data.shape[0] )


	if sys.argv[1] == 'train' :

		#######################
		# Train the GMM model #
		#######################

		n_components = 20
		gmm = GMM( n_components=n_components )
		print( 'Training the model with %d gaussian units' % n_components )
		gmm.from_samples( data )
		print( 'Model ready' )

		# Save the model:
		with open( gmm_file, 'wb' ) as f :
			pickle.dump( gmm, f )


	elif sys.argv[1] == 'score' :

		# Load the model:
		with open( gmm_file, 'rb' ) as f :
			gmm = pickle.load( f )

		######################
		# Evaluate the model #
		######################

		N_test = 20
		i_test = np.random.choice( range( len( data ) ), N_test, replace=False )
		X_data = df.drop( actions, axis=1 ).to_numpy()[i_test,:]
		Y_data = df[ actions ].to_numpy()[i_test,:]

		X_indices = np.array( list( range( dim_s ) ) + list( range( dim_s + dim_a, dim_t ) ) )
		Y_pred = gmm.predict( X_indices, X_data )

		abs_errors = abs( Y_pred - Y_data ).mean( axis=0 )
		print( 'Absolute errors: %g %g' % tuple( abs_errors ) )
		print( Y_data )
		print( Y_pred )


	exit( 0 )


# Load the model:
with open( gmm_file, 'rb' ) as f :
	gmm = pickle.load( f )


#######################
# Load the trial data #
#######################

if len( sys.argv ) > 2 :
	trial_file = sys.argv[2]

df = pd.read_csv( trial_file, sep=' ', header=None, names=columns )

# Convert every field to np.float64 and replace strings by np.nan:
df = df.apply( pd.to_numeric, errors='coerce' )

# Remove rows that contain at least one NaN:
df = df.dropna()

# Reset the index:
df.reset_index( drop=True, inplace=True )


s_sprime_indices = np.array( list( range( dim_s ) ) + list( range( dim_s + dim_a, dim_t ) ) )
s_a_indices = np.array( range( dim_s + dim_a ) )

delta_a = []
for _, samp in df.iterrows() :
	s_a = samp[ state_1 + actions ].to_numpy()[np.newaxis,:]
	s_sprime = samp[ state_1 + state_2 ].to_numpy()
	#s_a = samp.iloc[ :, s_a_indices ].to_numpy()
	#s_sprime = samp.iloc[ :, s_sprime_indices ].to_numpy()
	sprime = samp[ state_2 ].to_numpy()

	expected_sprime = gmm.predict( s_a_indices, s_a )
	#print( s_sprime_indices.shape, s_sprime.shape )
	grad_a_sprime = gmm.condition_derivative( s_sprime_indices, s_sprime )

	delta_a.append( grad_a_sprime.dot( expected_sprime - sprime ) )


plt.plot( df[ actions ] )
plt.plot( delta_a )
