#!/usr/bin/python
from ModelTree.model_tree import Model_tree
import sys


if len( sys.argv ) < 2 :
	print( 'Please specify the parameter file of the tree.', file=sys.stderr )
	exit( -1 )

param_file = sys.argv[1]


features = [ 'Direction',
			 'Steering angle',
			 'Roll angle',
			 'Pitch Angle',
			 'Boggie angle',
			 'Front Fx',
			 'Front Fy',
			 'Front Fz',
			 'Front tx',
			 'Front ty',
			 'Front tz',
			 'Rear tx',
			 'Rear ty',
			 'Rear tz' ]


model_tree = Model_tree( oblique=False, model='linear' )
model_tree.load_tree_params( param_file )
model_tree.diagram( feature_names=features, float_format='{:.2f}' )
