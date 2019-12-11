#!/usr/bin/python
import pandas as pd
import seaborn as sns
from matplotlib.pyplot import *

#rcParams['text.usetex']=True
#rcParams['text.latex.unicode']=True


data_file = 'training_data/step_0.dat'

df = pd.read_csv( data_file, sep=' ', header=None, skiprows=6, skipfooter=1, engine='python' )

state_1 = [ 'Steering angle', 'Roll angle', 'Pitch Angle', 'Boggie angle' ]
state_2 = [ 'Front $f_x$', 'Front $f_y$', 'Front $f_z$', 'Front $\\tau_x$', 'Front $\\tau_y$', 'Front $\\tau_z$',
             'Rear $f_x$', 'Rear $f_y$', 'Rear $f_z$', 'Rear $\\tau_x$', 'Rear $\\tau_y$', 'Rear $\\tau_z$' ]
actions = [ 'Steering rate', 'Boggie torque' ]
columns = [ 'Direction' ] + state_1 + state_2 + actions



fig, ax = subplots( 4, 4, figsize=( 10, 10 ) )
fig.canvas.set_window_title( __file__ )

for i in range( 4 ) :

	ax[0,i].scatter( df[ 1 + i ], df[ 17 ] )
	ax[0,i].scatter( df[ 1 + i ], df[ 18 ] )

	ax[0,i].set_xlabel( state_1[ i ] )


for i in range( 4 ) :
	for j in range( 3 ) :

		ax[j+1,i].scatter( df[ 5 + i*3 + j ], df[ 17 ] )
		ax[j+1,i].scatter( df[ 5 + i*3 + j ], df[ 18 ] )

		ax[j+1,i].set_xlabel( state_2[ i*3 + j ] )


ax[0,0].legend( actions )

left  = 0.05
right = 0.98
bottom = 0.05
top = 0.98
wspace = 0.1
hspace = 0.3
subplots_adjust( left, bottom, right, top, wspace, hspace )



df.columns = columns

# Compute the correlation matrix
corr = df.corr()

# Generate a mask for the upper triangle
mask = np.zeros_like( corr, dtype=np.bool )
mask[np.triu_indices_from( mask )] = True

f, ax = subplots()
ax = sns.heatmap(
	corr, 
	mask=mask,
	cbar_kws = { 'ticks': [ -1, -0.5, 0, 0.5, 1 ], 'shrink': 0.8 },
	annot=True,
	annot_kws = { 'size': 8 },
	vmin=-1, vmax=1, center=0,
	cmap=sns.diverging_palette(20, 220, n=200),
	square=True
)
ax.set_xticklabels( ax.get_xticklabels(), rotation=45, horizontalalignment='right' )
ax.set_yticklabels( ax.get_yticklabels(), rotation=0 )



show()
