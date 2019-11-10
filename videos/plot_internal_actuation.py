#!/usr/bin/python3
import sys
import os
from matplotlib.pyplot import *
from matplotlib.spines import Spine


# rock_step_3:
#time_f = 24
# rock_step_3_bis:
#time_f = 16.5
# groove:
time_f = 25

# rock_step_3:
#ymin1 = -21
#ymax1 = 21
#ymin2 = -6
#ymax2 = 6
# groove:
ymin1 = -15.5
ymax1 = 15.5
ymin2 = -20
ymax2 = 20

#step = 0.001
step = 0.1
fps = 25

#pretime = 0
#postime = -1
pretime = 1
#postime = 2
postime = 3

plot_color = 'w'
demo_bg_color = ( 179./255, 71./255, 0./255 )


#rate = int( 1./fps/step )
next_frame = 0.
frame_t = step/( max( 0., pretime ) + time_f + max( 0., postime ) )*100

if len( sys.argv ) > 1 :
	file_name = sys.argv[1]
else :
	sys.stderr.write( 'Veuillez passer en argument le nom du fichier contenant les données.\n' )
	exit( 1 )

w = False
next_arg = 2
if len( sys.argv ) > 2 and sys.argv[2] == 'w' :
	w = True
	next_arg = 3

try :
	start = int( sys.argv[next_arg] )
except :
	start = 0

try :
	stop = int( sys.argv[next_arg + 1] )
except :
	stop = -1

out_file = './%s.mp4' % os.path.splitext( os.path.basename( file_name ) )[0]
#tmp_base = '/tmp/%s_' % os.path.splitext( os.path.basename( file_name ) )[0]
tmp_base = '/tmp/ia_plot_'
tmp_files = tmp_base + '%05d.png'


#rcParams['text.usetex']=True
#rcParams['text.latex.unicode']=True
rcParams['axes.linewidth'] = 3
rc('font', weight='bold')
rcParams['axes.titleweight'] = 'bold'
rcParams['axes.labelweight'] = 'bold'
if not w :
	rcParams['figure.facecolor'] = demo_bg_color
	rcParams['axes.facecolor'] = demo_bg_color

Lt = []
L1 = []
L2 = []

fig, ax = subplots( 2, sharex=True, figsize=( 16, 4 ) )
fig.canvas.set_window_title( '%s (%s)' % ( __file__, file_name ) )
if w :
	fig.patch.set_alpha( 0 )

ax[0].set_title( u'Hinge rate (deg/s)', fontsize=15, pad=5 )
#ax[0].set_ylabel( u'$V$', fontsize=15, labelpad=-2 )
l1, = ax[0].plot( Lt, L1, linewidth=3, zorder=0, c=plot_color )
#ax[0].grid( True )
ax[0].spines['abscissa'] = Spine( ax[0], 'bottom', ax[0].spines['bottom'].get_path(), ls='dashed', alpha=1 )
ax[0].set_ylim(( ymin1, ymax1 ))

ax[1].set_title( u'Boggie torque (N.m)', fontsize=15, pad=5 )
#ax[1].set_ylabel( u'$T$', fontsize=15, labelpad=-2 )
l2, = ax[1].plot( Lt, L2, linewidth=3, zorder=0, c=plot_color )
#ax[1].grid( True )
ax[1].spines['abscissa'] = Spine( ax[1], 'bottom', ax[1].spines['bottom'].get_path(), ls='dashed', alpha=1 )
ax[1].set_ylim(( ymin2, ymax2 ))

ax[1].set_xlim(( 0, time_f ))
#ax[1].set_xlabel( u'\\textrm{Time $(s)$}', fontsize=15, labelpad=-2 )
ax[1].set_xlabel( u'Time (s)', fontsize=15, labelpad=-2 )

subplots_adjust( left=0.03, right=0.99, top=0.94, bottom=0.1 )


count = 0

if w :
	while pretime >= 0 :
		savefig( tmp_files % count, transparent=True )
		count += 1
		pretime -= 1./fps

f = open( file_name, 'r' )

f.seek(0, 0)
for i in range( start ):
	line = f.readline();

stop_count = 0
now = 0

while True :

	line = f.readline().split()

	if not line :
		break
	
	if len( line ) < 3 :
		continue
	
	try :
		v1 = float( line[1] )
		v2 = float( line[2] )
		L1.append( v1 )
		L2.append( v2 )
	except ValueError :
		continue

	Lt.append( now*step )

	if w :
		l1.set_xdata( Lt )
		l2.set_xdata( Lt )

		l1.set_ydata( L1 )
		l2.set_ydata( L2 )

		#if now % rate == 0 :
		while now*step >= next_frame :
			next_frame += 1./fps
			print( '\rCapture\'s progress: %d%%' % ( now*frame_t ), end='' )
			sys.stdout.flush()
			fig.canvas.draw()
			savefig( tmp_files % ( count ), transparent=True )
			count += 1
	now += 1

	stop_count += 1

	if stop != -1 and stop_count > stop:
		break

f.close()


if w :

	while postime >= 0 :
		savefig( tmp_files % count, transparent=True )
		count += 1
		postime -= 1./fps

	print( '\rCaptures made           ' )
	#os.system( 'avconv -r ' + str( fps ) + ' -i ' + tmp_files + ' ' + out_file )
	#os.system( 'rm %s*.png' % tmp_base )
else :

	l1.set_xdata( Lt )
	l2.set_xdata( Lt )

	l1.set_ydata( L1 )
	l2.set_ydata( L2 )

	fig.canvas.draw()
	show()
