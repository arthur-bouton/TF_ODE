#!/usr/bin/python
import pandas as pd
import numpy as np
from gmr import GMM # https://github.com/Bouty92/gmr
from ModelTree.model_tree import Model_tree
import pickle
import sys
import matplotlib.pyplot as plt
from tqdm import tqdm

import warnings
warnings.filterwarnings( action='ignore', module='pandas', category=UserWarning )


gmm_file = 'gmm_t2e5_k500.pkl'

trial_file = 'trial_0.dat'
if len( sys.argv ) > 1 :
	trial_file = sys.argv[1]
print( trial_file )



forces = [ 'Front $f_x$', 'Front $f_y$', 'Front $f_z$',
		   'Front $\\tau_x$','Front $\\tau_y$', 'Front $\\tau_z$',
		   'Rear $f_x$', 'Rear $f_y$', 'Rear $f_z$',
		   'Rear $\\tau_x$', 'Rear $\\tau_y$', 'Rear $\\tau_z$' ]
state = [ 'Direction', 'Steering angle', 'Roll angle', 'Pitch Angle', 'Boggie angle' ] + forces
actions = [ 'Steering rate', 'Boggie torque' ]

state_1_full = [ name + ' s1' for name in state ]
state_2_full = [ name + ' s2' for name in state ]
columns = state_1_full + actions + state_2_full

features_to_drop = [ 'Rear $f_x$', 'Rear $f_y$', 'Rear $f_z$' ]

state = [ feature for feature in state if feature not in features_to_drop ]
state_1 = [ name + ' s1' for name in state ]
state_2 = [ name + ' s2' for name in state ]

dim_s = len( state )
dim_a = len( actions )
dim_t = 2*dim_s + dim_a




# Load the model:
with open( gmm_file, 'rb' ) as f :
	gmm = pickle.load( f )


#######################
# Load the trial data #
#######################

df = pd.read_csv( trial_file, sep=' ', header=None, names=columns )

# Convert every field to np.float64 and replace strings by np.nan:
df = df.apply( pd.to_numeric, errors='coerce' )

# Remove rows that contain at least one NaN:
df = df.dropna()

# Reset the index:
df.reset_index( drop=True, inplace=True )
#df = df.iloc[:3]

# Symmetrize the states and actions around the steering angle:
variables_to_flip = [ 'Direction', 'Steering angle', 'Roll angle', 'Boggie angle' ] + forces[1::2]
variables_to_flip = [ name + ' s1' for name in variables_to_flip ]
variables_to_flip = variables_to_flip + actions
df.loc[ df[ 'Steering angle s1' ] < 0, variables_to_flip ] *= -1

variables_to_flip = [ 'Direction', 'Steering angle', 'Roll angle', 'Boggie angle' ] + forces[1::2]
variables_to_flip = [ name + ' s2' for name in variables_to_flip ]
df.loc[ df[ 'Steering angle s2' ] < 0, variables_to_flip ] *= -1

# Drop some features:
df = df.drop( [ name + ' s1' for name in features_to_drop ], axis=1 )
df = df.drop( [ name + ' s2' for name in features_to_drop ], axis=1 )


s_sprime_indices = np.array( list( range( dim_s ) ) + list( range( dim_s + dim_a, dim_t ) ) )
s_a_indices = np.array( range( dim_s + dim_a ) )

delta_a = []
grad_a = []
delta_s_norm = []
for _, samp in tqdm( df.iterrows(), total=len( df ), leave=False, desc='Computation of the gradients' ) :
	s_a = samp[ state_1 + actions ].to_numpy()[np.newaxis,:]
	s_sprime = samp[ state_1 + state_2 ].to_numpy()
	sprime = samp[ state_2 ].to_numpy()

	expected_sprime = gmm.predict( s_a_indices, s_a )
	grad_a_sprime = gmm.condition_derivative( s_sprime_indices, s_sprime )[:,dim_s:]

	delta_a.append( grad_a_sprime.dot( ( expected_sprime - sprime ).T ) )

	# For plot only:
	grad_a.append( grad_a_sprime )
	delta_s_norm.append( np.linalg.norm( expected_sprime - sprime ) )



#######################################
# Load the trees and their parameters #
#######################################

alpha = 0.3

oblique = False
model_type='linear'
param_file = 'tree_params2_0_'
if len( sys.argv ) > 2 :
	param_file = sys.argv[2]

batch_dparams = [ {} ]*2

for action_n in range( 2 ) :
	model_tree = Model_tree( oblique=oblique, model=model_type )
	model_tree.load_tree_params( param_file + str( action_n + 1 ) )
	tree_params = model_tree.get_tree_params()

	for i, samp in df.iterrows() :
		s = samp[ state_1 ].to_numpy()

		_, node_id = model_tree.predict( s, return_node_id=True )

		if node_id not in batch_dparams[action_n] :
			batch_dparams[action_n][node_id] = np.zeros( dim_s + 1 )

		grad_mu = np.insert( s, 0, 1 )

		batch_dparams[action_n][node_id] += alpha*grad_mu/grad_mu.dot( grad_mu )*delta_a[i][action_n]


	# Modify the parameters of each regression model encountered:
	for node_id, dparams in batch_dparams[action_n].items() :
		for i in range( len( dparams ) ) :
			tree_params[node_id]['model params'][i] += float( dparams[i] )

	# Save the new parameters:
	new_param_file = param_file[:-2] + str( int( param_file[-2] ) + 1 ) + '_' + str( action_n + 1 )
	model_tree.set_tree_params( tree_params )
	model_tree.save_tree_params( new_param_file )


print( batch_dparams )



#########
# Plots #
#########

fig, ax = plt.subplots( 2, 1, sharex=True )
fig.canvas.set_window_title( 'Policy corrections' )

ax[0].set_title( 'Trial result' )
ax[0].plot( df[ actions ] )
ax[0].legend( [ 'Steering rate', 'Boggie torque' ] )

ax[1].set_title( 'Corrections' )
ax[1].plot( [ a[0] for a in delta_a ] )
ax[1].plot( [ a[1] for a in delta_a ] )
ax[1].legend( [ 'Steering rate', 'Boggie torque' ] )


fig, ax = plt.subplots( 3, 1, sharex=True )
fig.canvas.set_window_title( 'Gradient and state discrepancy' )

ax[0].set_title( 'Gradient of the steering rate' )
for i in range( len( grad_a[0][0] ) ) :
	ax[0].plot( [ g[0][i] for g in grad_a ] )
ax[0].legend( list( range( len( grad_a[0][0] ) ) ) )

ax[1].set_title( 'Gradient of the boggie torque' )
for i in range( len( grad_a[0][1] ) ) :
	ax[1].plot( [ g[1][i] for g in grad_a ] )
ax[1].legend( list( range( len( grad_a[0][1] ) ) ) )

ax[2].set_title( 'Delta s norm' )
ax[2].plot( delta_s_norm )


plt.show()
