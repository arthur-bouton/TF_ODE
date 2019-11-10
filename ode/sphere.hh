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
    Sphere(Environment& env, 
	   const Eigen::Vector3d& pos, 
	   double mass, 
	   double rad,
	   bool casts_shadow = true) :
      Object(env, pos),
      _rad(rad), _mass(mass)
    {
      _casts_shadow = casts_shadow;
      init();
    }
    Sphere(const Sphere& o, Environment& env) :
      Object(env, o.get_pos()),
      _rad(o._rad),
      _mass(o._mass)
    {
	  _casts_shadow = o._casts_shadow;
      _copy(o);
      _geom = dCreateSphere(_env.get_space(), _rad);
      dGeomSetBody(_geom, _body);
    }

    virtual Object::ptr_t clone(Environment& env) const 
    { return Object::ptr_t(new Sphere(*this, env)); }

    double get_rad()	const { return _rad; }

    void init_again()
    {
      if (_body)
	dBodyDestroy(_body);
      if (_geom)
	dGeomDestroy(_geom);
      init();
    }
  
    /// const visitor
    virtual void accept (ConstVisitor &v) const
    {
      assert(_body); assert(_geom);
      v.visit(*this);
    }
  protected:
    void init()
    {
      Object::init();
      dMassSetSphereTotal(&_m, _mass, _rad);
      dBodySetMass(_body, &_m);
      _geom = dCreateSphere(_env.get_space(), _rad);
      dGeomSetBody(_geom, _body);
    }
    double _rad;
    double _mass;
  };
}


#endif	    /* !SPHERE_HH_ */
