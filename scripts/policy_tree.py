#!/usr/bin/python
import pandas as pd
import numpy as np
from ModelTree.model_tree import Model_tree


data_file_list = []
data_file_list.append( '../training_data/samples_mu06_period01_angle0_even.dat' )
data_file_list.append( '../training_data/samples_mu06_period01_angle0_odd.dat' )
data_file_list.append( '../training_data/samples_mu06_period01_angle1_even.dat' )
data_file_list.append( '../training_data/samples_mu06_period01_angle1_odd.dat' )
data_file_list.append( '../training_data/samples_mu06_period01_angle2_even.dat' )
data_file_list.append( '../training_data/samples_mu06_period01_angle2_odd.dat' )
data_file_list.append( '../training_data/samples_mu06_period01_angle5_even.dat' )
data_file_list.append( '../training_data/samples_mu06_period01_angle5_odd.dat' )


state_1 = [ 'Steering angle', 'Roll angle', 'Pitch Angle', 'Boggie angle' ]
state_2 = [ 'Front $f_x$', 'Front $f_y$', 'Front $f_z$', 'Front $\\tau_x$', 'Front $\\tau_y$', 'Front $\\tau_z$',
            'Rear $f_x$', 'Rear $f_y$', 'Rear $f_z$', 'Rear $\\tau_x$', 'Rear $\\tau_y$', 'Rear $\\tau_z$' ]
actions = [ 'Steering rate', 'Boggie torque' ]
columns = [ 'Direction' ] + state_1 + state_2 + actions


df_list = []
for f in data_file_list :
	df_list.append( pd.read_csv( f, sep=' ', header=None, names=columns, comment='t' ) )
df = pd.concat( df_list )

# Convert every field to np.float64 and replace strings by np.nan:
df = df.apply( pd.to_numeric, errors='coerce' )

# Remove rows that contain at least one NaN:
df = df.dropna()

# Shuffle the rows:
#df = df.sample( frac=1 )

# Reset the index:
df.reset_index( drop=True, inplace=True )


# Symmetrize the state and actions around the steering angle:
variables_to_flip = [ 'Direction', 'Steering angle', 'Roll angle', 'Boggie angle' ] + state_2[1::2] + actions
df.loc[ df[ 'Steering angle' ] < 0, variables_to_flip ] *= -1

# Drop the dimension that are highly correlated:
df = df.drop( [ 'Rear $f_x$', 'Rear $f_y$', 'Rear $f_z$' ], axis=1 )



X_data = df.iloc[:,:-2]
Y_data = df.iloc[:,-2:]

steering_rate_max = 15
boggie_torque_max = 20




oblique = False
max_depth_1 = 1
max_depth_2 = 0
min_samples = 20
loss_tol = 0.1
L1 = 1
grid = 20
param_file = 'tree_params_'

import sys
if len( sys.argv ) > 1 and sys.argv[1][0] == '{' :
	import yaml
	args = yaml.safe_load( sys.argv[1] )
	if not all( var in globals() for var in args ) :
		raise ValueError( 'Wrong variable' )
	globals().update( args )
if len( sys.argv ) > 2 :
	param_file = sys.argv[2]



#model_tree_1 = Model_tree( oblique=oblique, max_depth=max_depth_1, node_min_samples=min_samples, model='polynomial', interaction_only=True, loss_tol=loss_tol, L1=L1, search_grid=grid )
model_tree_1 = Model_tree( oblique=oblique, max_depth=max_depth_1, node_min_samples=min_samples, model='linear', loss_tol=loss_tol, L1=L1, search_grid=grid )
if len( sys.argv ) > 1 and sys.argv[1] == 'plot' :
	model_tree_1.load_tree_params( param_file + '1' )
else :
	print( '-- Training Model Tree 1 --' )
	model_tree_1.fit( X_data.to_numpy(), Y_data.iloc[:,0].to_numpy(), verbose=2 )
	model_tree_1.save_tree_params( param_file + '1' )


#model_tree_2 = Model_tree( oblique=oblique, max_depth=max_depth_2, node_min_samples=min_samples, model='polynomial', interaction_only=True, loss_tol=loss_tol, L1=L1, search_grid=grid )
model_tree_2 = Model_tree( oblique=oblique, max_depth=max_depth_2, node_min_samples=min_samples, model='linear', loss_tol=loss_tol, L1=L1, search_grid=grid )
if len( sys.argv ) > 1 and sys.argv[1] == 'plot' :
	model_tree_2.load_tree_params( param_file + '2' )
else :
	print( '\n-- Training Model Tree 2 --' )
	model_tree_2.fit( X_data.to_numpy(), Y_data.iloc[:,1].to_numpy(), verbose=2 )
	model_tree_2.save_tree_params( param_file + '2' )


Y_pred_1 = pd.DataFrame( model_tree_1.predict( X_data.to_numpy() ), columns=[ actions[0] ] )
Y_pred_2 = pd.DataFrame( model_tree_2.predict( X_data.to_numpy() ), columns=[ actions[1] ] )
Y_pred = pd.concat( [ Y_pred_1, Y_pred_2 ], axis=1 )

abs_errors = ( abs( Y_pred - Y_data ) ).mean()
quad_errors = ( ( Y_pred - Y_data )**2 ).mean()
n_params_1 = model_tree_1.get_number_of_params()
n_params_2 = model_tree_2.get_number_of_params()
print( '\nAbsolute errors: %.2f | %.2f -- Quadratic errors: %.2f | %.2f -- Parameters (non-zero/total): %i/%i | %i/%i\n'
% ( abs_errors[0], abs_errors[1], quad_errors[0], quad_errors[1], *n_params_1, *n_params_2 ) )

if len( sys.argv ) < 2 or sys.argv[1] != 'plot' :
	print( 'CSV entry: oblique, max_depth_1, max_depth_2, min_samples, loss_tol, L1, abs_err_1, abs_err_2, quad_err_1, quad_err_2, nz_params_1, t_params_1, nz_params_2, t_params_2' )
	print( '%s,%i,%i,%i,%f,%f,%f,%f,%f,%f,%i,%i,%i,%i'
	% ( oblique, max_depth_1, max_depth_2, min_samples, loss_tol, L1, abs_errors[0], abs_errors[1], quad_errors[0], quad_errors[1], *n_params_1, *n_params_2  ) )



if len( sys.argv ) > 1 and sys.argv[1] == 'plot' :
	from matplotlib.pyplot import *

	fig, ax = subplots( 2, 1, sharex=True )
	fig.canvas.set_window_title( 'Actions' )

	ax[0].set_title( actions[0] )
	ax[0].plot( Y_data.iloc[:,0] )
	ax[0].plot( Y_pred.iloc[:,0] )
	ax[0].legend( [ 'data', 'pred' ] )

	ax[1].set_title( actions[1] )
	ax[1].plot( Y_data.iloc[:,1] )
	ax[1].plot( Y_pred.iloc[:,1] )
	ax[1].legend( [ 'data', 'pred' ] )

	show()


