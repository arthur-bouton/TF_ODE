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

	Wheel( Environment& env, const Eigen::Vector3d& pos, double mass, double rad, double width, int def, bool casts_shadow = true ) :
	       Object( env, pos ), _mass( mass ), _rad( rad - width/2 ), _width( width ), _def( def )
	{
		if ( def == 0 )
			_rad = rad;
		_casts_shadow = casts_shadow;
		init();
	}

	Wheel( const Wheel& o, Environment& env ) : Object( env, o.get_pos() ), _mass( o._mass ), _rad( o._rad ), _width( o._width )
	{
		_casts_shadow = o._casts_shadow;
		_copy(o);
		_geom = dCreateCylinder( _env.get_space(), _rad, _width );
		dGeomSetBody(_geom, _body);

		_tire = (dGeomID*) calloc( _def, sizeof( dGeomID ) );

		for ( int i = 0 ; i < _def ; i++ )
		{
			_tire[i] = dCreateSphere( _env.get_space(), _width/2 );
			dGeomSetBody( _tire[i], _body );
			dGeomSetOffsetPosition( _tire[i], _rad*cos( i*2*M_PI/_def ), _rad*sin( i*2*M_PI/_def ), 0 );
		}
	}

	virtual Object::ptr_t clone( Environment& env ) const 
	{
		return Object::ptr_t( new Wheel( *this, env ) );
	}

	double get_rad()	  const { return _rad;   }
	double get_width() const { return _width; }
	double get_def() const { return _def; }

	void init_again()
	{
		if (_body)
			dBodyDestroy(_body);
		if (_geom)
			dGeomDestroy(_geom);
		init();
	}

	/// const visitor
	virtual void accept ( ConstVisitor &v ) const
	{
		assert(_body); assert(_geom);
		v.visit(*this);
	}


	virtual void set_collision_group( const char* group )
	{
		Object::set_collision_group( group );

		for ( int i = 0 ; i < _def ; i++ )
		{
			collision_feature* feature = (collision_feature*) dGeomGetData( _tire[i] );
			if ( feature != NULL )
				feature->group = group;
			else
				dGeomSetData( _tire[i], new collision_feature( group ) );
		}
	}


	virtual void set_contact_type( contact_type type )
	{
		Object::set_contact_type( type );

		for ( int i = 0 ; i < _def ; i++ )
		{
			collision_feature* feature = (collision_feature*) dGeomGetData( _tire[i] );
			if ( feature != NULL )
				feature->type = type;
			else
				dGeomSetData( _tire[i], new collision_feature( type ) );
		}
	}


	virtual ~Wheel()
	{
		for ( int i = 0 ; i < _def ; i++ )
			if (_tire[i])
			{
				delete (collision_feature*) dGeomGetData( _tire[i] );
				dGeomDestroy( _tire[i] );
			}
		delete _tire;
	}

	protected:

	void init()
	{
		Object::init();
	    assert(_body);
		dMassSetCylinderTotal( &_m, _mass, 3, _rad, _width );
		dBodySetMass( _body, &_m );

		_geom = dCreateCylinder( _env.get_space(), _rad, _width );
	    assert(_geom);
		dGeomSetBody( _geom, _body );

		_tire = (dGeomID*) calloc( _def, sizeof( dGeomID ) );

		for ( int i = 0 ; i < _def ; i++ )
		{
			_tire[i] = dCreateSphere( _env.get_space(), _width/2 );
			dGeomSetBody( _tire[i], _body );
			dGeomSetOffsetPosition( _tire[i], _rad*cos( i*2*M_PI/_def ), _rad*sin( i*2*M_PI/_def ), 0 );
		}
	}

	double _mass;
	double _rad;
	double _width;
	int _def;

	dGeomID* _tire;
};

}


#endif
