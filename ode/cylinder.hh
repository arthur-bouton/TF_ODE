/* 
** Copyright (C) 2014 Arthur BOUTON
** 
** This program is free software: you can redistribute it and/or modify  
** it under the terms of the GNU General Public License as published by  
** the Free Software Foundation, version 3.
**
** This program is distributed in the hope that it will be useful, but 
** WITHOUT ANY WARRANTY; without even the implied warranty of 
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
** General Public License for more details.
**
** You should have received a copy of the GNU General Public License 
** along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CYLINDER_HH_
#define CYLINDER_HH_

#include <iostream>
#include "ode/object.hh"


namespace ode
{

class Cylinder : public Object
{
	public:

	static constexpr double standard_mass = 1;

	Cylinder( Environment& env, const Eigen::Vector3d& pos, double mass, double radius, double length,
	          bool casts_shadow = true, bool create_geom = true ) :
	       Object( env, pos ), _mass( mass ), _radius( radius ), _length( length )
	{
		_casts_shadow = casts_shadow;
		init( create_geom );
	}

	double get_radius()	  const { return _radius;   }
	double get_length() const { return _length; }

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
	    assert(_body);
		dMassSetCylinderTotal( &_m, _mass, 3, _radius, _length );
		dBodySetMass( _body, &_m );

		if ( create_geom )
			add_cylinder_geom( _radius, _length );
	}

	double _mass;
	double _radius;
	double _length;
};

}


#endif
