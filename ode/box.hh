/*
** Copyright (C) 2004 mandor
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
** 
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
** 
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef   	BOX_HH_
# define   	BOX_HH_

#include <iostream>
#include "object.hh"

namespace ode
{


/// a simple wrapper around the ODE's box class
class Box : public Object
{
	public:
	static constexpr double standard_mass = 1;
	Box( Environment& env, const Eigen::Vector3d& pos, double mass, double l, double w, double h,
		 bool casts_shadow = true, bool create_geom = true ) :
		Object( env, pos ), _w( w ), _h( h ), _l( l ), _mass( mass )
	{
		_casts_shadow = casts_shadow;
		init( create_geom );
	}

	double get_length()	const { return _l; }
	double get_width()	const { return _w; }
	double get_height()	const { return _h; }

	/// const visitor
	virtual void accept( ConstVisitor &v ) const
	{
		assert( _body );
		v.visit( *this );
	}

	protected:

	void init( bool create_geom )
	{
		Object::init();
		dMassSetBoxTotal( &_m, _mass, _l, _w, _h );
		dBodySetMass( _body, &_m );

		if ( create_geom )
			add_box_geom( _l, _w, _h );
	}

	double _w, _h, _l;
	double _mass;
};


}


#endif	    /* !BOX_HH_ */
