#!/usr/bin/python
import pandas as pd
from matplotlib.pyplot import *

#rcParams['text.usetex']=True
#rcParams['text.latex.unicode']=True


data_file = 'training_data/step_0.dat'

df = pd.read_csv( data_file, sep=' ', header=None, skiprows=6, skipfooter=1, engine='python' )



fig, ax = subplots( 3, 4 )
fig.canvas.set_window_title( __file__ )


labels = [ 'front $f_x$', 'front $f_y$', 'front $f_z$', 'front $\\tau_x$', 'front $\\tau_y$', 'front $\\tau_z$',
           'rear $f_x$', 'rear $f_y$', 'rear $f_z$', 'rear $\\tau_x$', 'rear $\\tau_y$', 'rear $\\tau_z$' ]

for i in range( 4 ) :
	for j in range( 3 ) :

		ax[j,i].scatter( df[ 5 + i*3 + j ], df[ 17 ] )
		ax[j,i].scatter( df[ 5 + i*3 + j ], df[ 18 ] )

		ax[j,i].set_xlabel( labels[ i*3 + j ] )

show()
