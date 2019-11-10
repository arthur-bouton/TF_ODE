#!/usr/bin/python3
import sys
from matplotlib.pyplot import *
from numpy import *


if len( sys.argv ) > 1 :
	file_name = sys.argv[1]
else :
	sys.stderr.write( 'Veuillez passer en argument le nom du fichier contenant les données.\n' )
	exit( 1 )

try :
	start = int( sys.argv[2] )
except :
	start = 0

try :
	stop = int( sys.argv[3] )
except :
	stop = -1

file = open( file_name, 'r' )

file.seek( 0, 0 )
for i in range( start ) :
	line = file.readline();

x = [ 0, 0 ]

t = [ 0, 0 ]

N = 500
i = 1
y0 = [ 0, 0 ]

Te = 0.001 # s
fc = 0.5 # Hz

wc = 2*pi*fc
ewt = exp( -wc*Te )
y1 = [ 0, 0 ]

tgwt = tan( wc*Te/2 )
khy = ( 1 - tgwt )/( 1 + tgwt )
khx = tgwt/( 1 + tgwt )
y2 = [ 0, 0 ]

den = 2 + wc*Te
kby = ( 2 - wc*Te )/den
kbx = wc*Te/den
y3 = [ 0, 0 ]

w0 = 2*pi*1
Q = 0.5
den = 2*w0*Te + 4*Q + Q*w0**2*Te**2
ky1 = 2*Q*( 4 - w0**2*Te**2 )/den
ky2 = ( 2*w0*Te - 4*Q - Q*w0**2*Te**2 )/den
kx0 = Q*w0**2*Te**2/den
kx1 = 2*kx0
kx2 = kx0
y4 = [ 0, 0 ]

stop_count = 0

while True :

	line = file.readline().split()

	if not line :
		break
	
	if len( line ) < 1 :
		continue

	#if stop_count > 2000 :
		#break
	#elif stop_count >= 100 :
		#line[0] = 1
	#else :
		#line[0] = 0

	try :
		x.append( float( line[0] ) )
		t.append( t[-1] + Te )
		y0.append( sum( x[-min( i, N ):] )/N )
		i += 1
		y1.append( ewt*y1[-1] + ( 1 - ewt )*x[-2] )
		y2.append( khy*y2[-1] + khx*( x[-1] + x[-2] ) )
		y3.append( kby*y3[-1] + kbx*( x[-1] + x[-2] ) )
		y4.append( ky1*y4[-1] + ky2*y4[-2] + kx0*x[-1] + kx1*x[-2] + kx2*x[-3] )
	except ValueError :
		continue

	stop_count += 1

	if stop != -1 and stop_count > stop :
		break

file.close()


fig, ax = subplots( 4, sharex=True )
fig.canvas.set_window_title( __file__ + ' ' + file_name )

ax[0].set_title( 'Original' )
ax[0].plot( t, x )
ax[0].grid( True )

ax[1].set_title( 'Moving mean' )
ax[1].plot( t, y0 )
ax[1].grid( True )

#ax[2].set_title( 'Step response' )
#ax[2].plot( t, y1 )
#ax[2].grid( True )

#ax[2].set_title( 'First order homography' )
#ax[2].plot( t, y2 )
#ax[2].grid( True )

ax[2].set_title( 'First order bilinear' )
ax[2].plot( t, y3 )
ax[2].grid( True )

ax[3].set_title( 'Second order bilinear' )
ax[3].plot( t, y4 )
ax[3].grid( True )

show()
