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

	Cylinder( Environment& env, const Eigen::Vector3d& pos, double mass, double rad, double width, bool casts_shadow = true ) :
	       Object( env, pos ), _mass( mass ), _rad( rad ), _width( width )
	{
		_casts_shadow = casts_shadow;
		init();
	}

	Cylinder( const Cylinder& o, Environment& env ) : Object( env, o.get_pos() ),
													  _mass( o._mass ),
													  _rad( o._rad ),
													  _width( o._width )
	{
		_casts_shadow = o._casts_shadow;
		_copy(o);
		_geom = dCreateCylinder( _env.get_space(), _rad, _width );
		dGeomSetBody(_geom, _body);
	}

	virtual Object::ptr_t clone( Environment& env ) const 
	{
		return Object::ptr_t( new Cylinder( *this, env ) );
	}

	double get_rad()	  const { return _rad;   }
	double get_width() const { return _width; }

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
	}

	double _mass;
	double _rad;
	double _width;
};

}


#endif
