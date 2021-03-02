#!/usr/bin/python3
import warnings
warnings.simplefilter( action='ignore', category=FutureWarning )

import numpy as np
import os
import sys
import pandas as pd
import shap
import pickle
import matplotlib.pyplot as plt
import glob
from tensorflow import keras

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'



data_file_list = glob.glob( "../training_data/samples/samples_Ry05t05c_eg87_end_ga001_lr1e-8_mu05_angle0_*.dat" )

model_path = '../training_data/Ry05t05c_eg87_end_ga001_lr1e-8/actor'

shap_values_file = 'shap_values.pkl'



###################
# Import the data #
###################

state_1 = [ 'Steering angle', 'Roll angle', 'Pitch Angle', 'Boggie angle' ]
state_2 = [ 'Front $f_x$', 'Front $f_y$', 'Front $f_z$', 'Front $\\tau_x$', 'Front $\\tau_y$', 'Front $\\tau_z$',
            'Rear $f_x$', 'Rear $f_y$', 'Rear $f_z$', 'Rear $\\tau_x$', 'Rear $\\tau_y$', 'Rear $\\tau_z$' ]
actions = [ 'Steering rate', 'Boggie torque' ]
columns = [ 'Direction' ] + state_1 + state_2 + actions


df_list = []
for f in data_file_list :
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


X_data = df.iloc[:,:-2]



if len( sys.argv ) > 1 and sys.argv[1] == 'load' :
	with open( shap_values_file, 'rb' ) as f :
		shap_values = pickle.load( f )

else :
	####################
	# Import the model #
	####################

	model = keras.models.load_model( model_path, compile=False )


	###########################
	# Compute the SHAP values #
	###########################

	explainer = shap.KernelExplainer( model.predict, X_data )
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
