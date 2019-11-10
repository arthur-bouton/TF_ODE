#!/usr/bin/python2.7
import sys

usage = 'usage : ' + __file__ + ' [-r] h kp[ERP] kd[CFM]'


if ( len( sys.argv ) == 4 ) :
	try :
		h = float( sys.argv[1] )
		kp = float( sys.argv[2] )
		kd = float( sys.argv[3] )
	except ValueError :
		print usage

	print 'ERP = %f\nCFM = %f' % ( h*kp/( h*kp + kd ), 1/( h*kp + kd ) )

elif ( len( sys.argv ) == 5 and sys.argv[1] == '-r' ) :
	try :
		h = float( sys.argv[2] )
		ERP = float( sys.argv[3] )
		CFM = float( sys.argv[4] )
	except ValueError :
		print usage

	print 'kp = %f\nkd = %f' % ( 1/h*ERP/CFM, 1/CFM - ERP/CFM )

else :
	print usage
