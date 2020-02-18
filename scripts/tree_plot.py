#!/usr/bin/python
import pandas as pd
from matplotlib.pyplot import *

#rcParams['text.usetex']=True
#rcParams['text.latex.unicode']=True


data_file = 'tree_selection.csv'

columns = [ 'oblique', 'max_depth_1', 'max_depth_2', 'min_samples', 'loss_tol', 'L1_reg',
            'abs_err_1', 'abs_err_2', 'quad_err_1', 'quad_err_2', 'nz_params_1', 't_params_1', 'nz_params_2', 't_params_2' ]


df = pd.read_csv( data_file, header=None, names=columns )

# Convert every field to np.float64 and replace strings by np.nan:
#df = df.apply( pd.to_numeric, errors='coerce' )

# Remove rows that contain at least one NaN:
#df = df.dropna()


max_depth = 8


fig, ax = subplots( 2, 1, figsize=( 10, 10 ) )
fig.canvas.set_window_title( 'Steering rate\'s straight LMT' )

df[ ~df.oblique & ( df.max_depth_1 <= max_depth ) ].abs_err_1.groupby( df.L1_reg ).plot( legend=True, x='max_depth_1', use_index=False, ax=ax[0] )
df[ ~df.oblique & ( df.max_depth_1 <= max_depth ) ].nz_params_1.groupby( df.L1_reg ).plot( legend=True, x='max_depth_1', use_index=False, ax=ax[1] )
ax[0].legend( [ 'L1_reg=' + l for l in ax[0].get_legend_handles_labels()[1] ] )
ax[1].legend( [ 'L1_reg=' + l for l in ax[0].get_legend_handles_labels()[1] ] )
ax[0].set_title( 'Absolute loss' )
ax[1].set_title( 'Number of non-zero parameters' )
ax[1].set_xlabel( 'Depth of the tree' )


fig, ax = subplots( 2, 1, figsize=( 10, 10 ) )
fig.canvas.set_window_title( 'Boggie torque\'s straight LMT' )

df[ ~df.oblique & ( df.max_depth_2 <= max_depth ) ].abs_err_2.groupby( df.L1_reg ).plot( legend=True, x='max_depth_2', use_index=False, ax=ax[0] )
df[ ~df.oblique & ( df.max_depth_2 <= max_depth ) ].nz_params_2.groupby( df.L1_reg ).plot( legend=True, x='max_depth_2', use_index=False, ax=ax[1] )
ax[0].legend( [ 'L1_reg=' + l for l in ax[0].get_legend_handles_labels()[1] ] )
ax[1].legend( [ 'L1_reg=' + l for l in ax[0].get_legend_handles_labels()[1] ] )
ax[0].set_title( 'Absolute loss' )
ax[1].set_title( 'Number of non-zero parameters' )
ax[1].set_xlabel( 'Depth of the tree' )


fig, ax = subplots( 2, 1, figsize=( 10, 10 ) )
fig.canvas.set_window_title( 'Steering rate\'s oblique LMT' )

df[ df.oblique & ( df.max_depth_1 <= max_depth ) ].abs_err_1.groupby( df.L1_reg ).plot( legend=True, x='max_depth_1', use_index=False, ax=ax[0] )
df[ df.oblique & ( df.max_depth_1 <= max_depth ) ].nz_params_1.groupby( df.L1_reg ).plot( legend=True, x='max_depth_1', use_index=False, ax=ax[1] )
ax[0].legend( [ 'L1_reg=' + l for l in ax[0].get_legend_handles_labels()[1] ] )
ax[1].legend( [ 'L1_reg=' + l for l in ax[0].get_legend_handles_labels()[1] ] )
ax[0].set_title( 'Absolute loss' )
ax[1].set_title( 'Number of non-zero parameters' )
ax[1].set_xlabel( 'Depth of the tree' )


fig, ax = subplots( 2, 1, figsize=( 10, 10 ) )
fig.canvas.set_window_title( 'Boggie torque\'s oblique LMT' )

df[ df.oblique & ( df.max_depth_2 <= max_depth ) ].abs_err_2.groupby( df.L1_reg ).plot( legend=True, x='max_depth_2', use_index=False, ax=ax[0] )
df[ df.oblique & ( df.max_depth_2 <= max_depth ) ].nz_params_2.groupby( df.L1_reg ).plot( legend=True, x='max_depth_2', use_index=False, ax=ax[1] )
ax[0].legend( [ 'L1_reg=' + l for l in ax[0].get_legend_handles_labels()[1] ] )
ax[1].legend( [ 'L1_reg=' + l for l in ax[0].get_legend_handles_labels()[1] ] )
ax[0].set_title( 'Absolute loss' )
ax[1].set_title( 'Number of non-zero parameters' )
ax[1].set_xlabel( 'Depth of the tree' )


show()
