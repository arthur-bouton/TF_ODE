#!/usr/bin/python
from looptools import *
from ModelTree.model_tree import Model_tree
import numpy as np

data_file = 'experimental_data/experimental_trial_5.dat'
length = 100

state = [ 'Direction', 'Steering angle', 'Roll angle', 'Pitch Angle', 'Boggie angle',
          'Front $f_x$', 'Front $f_y$', 'Front $f_z$',
          'Front $\\tau_x$', 'Front $\\tau_y$', 'Front $\\tau_z$',
          'Rear $\\tau_x$', 'Rear $\\tau_y$', 'Rear $\\tau_z$' ]
actions = [ 'Steering rate', 'Boggie torque' ]

df = Datafile( data_file, '2:15,-2:x2', ncols=20, length=length )

plot = Monitor( [ 14, 14, 2, 2 ] )
for i in range( 2 ) :
	plot.axes[i].legend( state, ncol=2 )
	plot.axes[i+2].legend( actions )

model_tree = []
tree_params = []
for i in range( 2 ) :
	model_tree.append( Model_tree( oblique=False, model='linear' ).load_tree_params( 'tree_params2_' + str( i + 1 ) ) )
	tree_params.append( model_tree[-1].get_tree_params() )

for data in df :
	s = data[:-2]
	a = data[-2:]

	a_pred = []
	contribs = []
	for i in range( 2 ) :
		pred, node_id = model_tree[i].predict( np.array( s ), return_node_id=True )
		a_pred.append( pred )
		coeffs = tree_params[i][node_id]['model params']
		contribs.append( [ f*c for f, c in zip( s, coeffs ) ] )

	plot.add_data( *( contribs[0] + contribs[1] + a + a_pred ) )
