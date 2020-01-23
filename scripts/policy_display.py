#!/usr/bin/python
import pandas as pd
import seaborn as sns
from matplotlib.pyplot import *

#rcParams['text.usetex']=True
#rcParams['text.latex.unicode']=True


data_file = '../training_data/step_06_PER_1_no_sym.dat'

state_1 = [ 'Steering angle', 'Roll angle', 'Pitch Angle', 'Boggie angle' ]
state_2 = [ 'Front $f_x$', 'Front $f_y$', 'Front $f_z$', 'Front $\\tau_x$', 'Front $\\tau_y$', 'Front $\\tau_z$',
             'Rear $f_x$', 'Rear $f_y$', 'Rear $f_z$', 'Rear $\\tau_x$', 'Rear $\\tau_y$', 'Rear $\\tau_z$' ]
actions = [ 'Steering rate', 'Boggie torque' ]
columns = [ 'Direction' ] + state_1 + state_2 + actions


df = pd.read_csv( data_file, sep=' ', header=None, names=columns )

# Convert every field to np.float64 and replace strings by np.nan:
df = df.apply( pd.to_numeric, errors='coerce' )

# Remove rows that contain at least one NaN:
df = df.dropna()


# Plot all the action values in relation to each dimension of the state space:

fig, ax = subplots( 4, 4, figsize=( 10, 10 ) )
fig.canvas.set_window_title( __file__ )
dot_size = 2

for i in range( 4 ) :

	ax[0,i].scatter( df.iloc[:,i], df.iloc[:,17], s=dot_size )
	ax[0,i].scatter( df.iloc[:,i], df.iloc[:,18], s=dot_size )

	ax[0,i].set_xlabel( state_1[ i ] )


for i in range( 4 ) :
	for j in range( 3 ) :

		ax[j+1,i].scatter( df.iloc[:, 4 + i*3 + j ], df.iloc[:,17], s=dot_size )
		ax[j+1,i].scatter( df.iloc[:, 4 + i*3 + j ], df.iloc[:,18], s=dot_size )

		ax[j+1,i].set_xlabel( state_2[ i*3 + j ] )


ax[0,0].legend( actions )

left  = 0.05
right = 0.98
bottom = 0.05
top = 0.98
wspace = 0.1
hspace = 0.3
subplots_adjust( left, bottom, right, top, wspace, hspace )


# Plot the correlations between each dimension:

# Compute the correlation matrix:
corr = df.corr()

# Generate a mask for the upper triangle:
mask = np.zeros_like( corr, dtype=np.bool )
mask[np.triu_indices_from( mask )] = True

f, ax = subplots( figsize=( 12, 12 ) )
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
ax.set_xticklabels( ax.get_xticklabels()[:-1], rotation=45, horizontalalignment='right' )
ax.set_yticklabels( [''] + ax.get_yticklabels()[1:], rotation=0 )


# Print the variance of each feature:

print( df.var().to_frame( 'Variances' ) )


# Print the ordered most significant correlations:

mask = np.zeros_like( corr, dtype=np.bool )
mask[np.triu_indices_from( mask, 1 )] = True
corr_list = corr.where( mask ).stack().reset_index()
corr_list.columns = [ 'Feature 1', 'Feature 2', 'Correlations' ]

ranking = corr_list.iloc[ :, 2 ].abs().sort_values( ascending=False )

print( corr_list.iloc[ ranking.head( 10 ).index ] )



show()
