/*
** Copyright (C) 2006 Jean-baptiste MOURET
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

#ifndef   	CAPPEDCYL_HH_
# define   	CAPPEDCYL_HH_

#include "object.hh"


namespace ode
{
  /// this class encapsulates the CappedCylinder class in ODE
  /// WARNING: length include the cap!
class CappedCyl : public Object
{
	public:
	CappedCyl( Environment& env, const Eigen::Vector3d& pos, double mass, double radius, double length,
			   bool casts_shadow = true, bool create_geom = true ) :
	  Object( env, pos ), _mass( mass ), _radius( radius ), _length( length )
	{
		_casts_shadow = casts_shadow;
		init( create_geom );
	}

	double get_radius()	const { return _radius; }
	double get_length()	const { return _length; }

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
		/*dMassSetCylinderTotal(&_m, _mass,
				  3, // direction = 3 (along z)
				  _radius, _length - _radius * 2);*/
		dMassSetCapsuleTotal(&_m, _mass,
				  3, // direction = 3 (along z)
				  _radius, _length - _radius * 2);
		dBodySetMass(_body, &_m);

		if ( create_geom )
			add_capcyl_geom( _radius, _length );
	}

	// atributes
	double _mass;
	double _radius;
	double _length;
};


}


#endif	    /* !CAPPEDCYL_HH_ */
