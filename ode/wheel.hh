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

#ifndef WHEEL_HH_
#define WHEEL_HH_

#include <iostream>
#include "ode/object.hh"


namespace ode
{

class Wheel : public Object
{
	public:

	static constexpr double standard_mass = 1;

	Wheel( Environment& env, const Eigen::Vector3d& pos, double mass, double radius, double width, int def,
	       bool casts_shadow = true ) :
	       Object( env, pos ), _mass( mass ), _radius( radius - width/2 ), _width( width ), _def( def )
	{
		if ( def == 0 )
			_radius = radius;
		_casts_shadow = casts_shadow;
		init();
	}

	double get_radius() const { return _radius; }
	double get_width() const { return _width; }
	double get_def() const { return _def; }

	/// const visitor
	virtual void accept ( ConstVisitor &v ) const
	{
		assert( _body ); assert( !_geoms.empty() );
		v.visit( *this );
	}

	protected:

	void init()
	{
		Object::init();
	    assert( _body );
		dMassSetCylinderTotal( &_m, _mass, 3, _radius, _width );
		dBodySetMass( _body, &_m );

		dGeomID rim = dCreateCylinder( _env.get_space(), _radius, _width );
		dGeomSetBody( rim, _body );
		_geoms.push_back( rim );

		for ( int i = 0 ; i < _def ; i++ )
		{
			dGeomID tire = dCreateSphere( _env.get_space(), _width/2 );
			dGeomSetBody( tire, _body );
			dGeomSetOffsetPosition( tire, _radius*cos( i*2*M_PI/_def ), _radius*sin( i*2*M_PI/_def ), 0 );
			_geoms.push_back( tire );
		}
	}

	double _mass;
	double _radius;
	double _width;
	int _def;
};

}


#endif
