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

#ifndef   	SPHERE_HH_
# define   	SPHERE_HH_

#include <iostream>
#include "object.hh"


namespace ode
{

/// a simple wrapper around the ODE's sphere class
class Sphere : public Object
{
	public:
	static constexpr double standard_mass = 1;
	Sphere( Environment& env, const Eigen::Vector3d& pos, double mass, double radius,
		    bool casts_shadow = true, bool create_geom = true ) :
		Object( env, pos ),
		  _radius( radius ), _mass( mass )
	{
	  _casts_shadow = casts_shadow;
	  init( create_geom );
	}

	double get_radius() const { return _radius; }

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
		dMassSetSphereTotal( &_m, _mass, _radius );
		dBodySetMass( _body, &_m );

		if ( create_geom )
			add_sphere_geom( _radius );
	}

	double _radius;
	double _mass;
};


}


#endif	    /* !SPHERE_HH_ */
