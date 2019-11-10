/*
** Copyright (C) 2014 Arthur BOUTON
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

#ifndef  ENVIRONMENT_HH_
#define  ENVIRONMENT_HH_

#define  dDOUBLE

#include <iostream>
#include <ode/ode.h>
#include <ode/common.h>
#include <set>
#include "misc.hh"

namespace ode
{


	typedef enum contact_type
	{
		HARD,
		SOFT,
		DISABLED
	} contact_type;

	typedef struct collision_feature
	{
		const char* group;
		contact_type type;
		collision_feature( const char* arg ) : group( arg ) {}
		collision_feature( contact_type arg ) : group( "\0" ), type( arg ) {}
	} collision_feature;


  class Object;
   //singleton : only one env
  class Environment
  {
    public:
      static constexpr double time_step = 0.05;
       // constructor
    Environment() :
        _ground(0x0), _pitch(0), _roll(0), _z(0), _mu(0.7)
      {
        _init(true);
      }
    Environment(double mu) :
        _ground(0x0), _pitch(0), _roll(0), _z(0), _mu(mu)
      {
        _init(true);
      }
    Environment(double mu, double angle) :
        _ground(0x0), _pitch(0), _roll(0), _z(0), _mu(mu)
      {
        _init(true,angle);
      }
    Environment(double pitch, double roll, double z) :
        _ground(0x0), _pitch(pitch), _roll(roll), _z(z), _mu(0.7)
      {
        _init(true);
      }
    Environment(double mu, double pitch, double roll, double z) :
        _ground(0x0), _pitch(pitch), _roll(roll), _z(z), _mu(mu)
      {
        _init(true);
      }

    Environment(bool add_ground) :
        _ground(0x0), _pitch(0), _roll(0), _z(0), _mu(0.7)
      {
        _init(add_ground);
      }

    Environment(bool add_ground, double mu) :
        _ground(0x0), _pitch(0), _roll(0), _z(0), _mu(mu)
      {
        _init(add_ground);
      }

     ~Environment()
      {

        if (_ground)
		{
			delete ( collision_feature* ) dGeomGetData( _ground );
          dGeomDestroy(_ground);
		}
        dSpaceDestroy(get_space());
        dWorldDestroy(get_world());
        dJointGroupDestroy(_contactgroup);
      }
       //interfaces
      dWorldID get_world()        const
      {
        return _world_id;
      }
      dSpaceID get_space()        const
      {
        return _space_id;
      }
      dGeomID get_ground()        const
      {
        return _ground;
      }
      dJointGroupID get_contactgroup() const
      {
        return _contactgroup;
      }
       //update sim
      void next_step(double dt = time_step)
      {
         //check collisions
        dSpaceCollide(_space_id, (void *)this, &_near_callback);
         //next step
        dWorldStep(_world_id, dt);
         //dWorldQuickStep(_world_id, dt);
         // remove all contact joints
        dJointGroupEmpty(_contactgroup);
      }
      void disable_gravity()
      {
        dWorldSetGravity(_world_id, 0, 0, 0);
      }
      void set_gravity(double x, double y, double z)
      {
        dWorldSetGravity(_world_id, x, y, z);
      }
      double get_pitch() const { return _pitch; }
      double get_roll() const { return _roll; }
      double get_z() const { return _z; }
    protected:
    void _init(bool add_ground,double angle=0);
      static void _near_callback(void *data, dGeomID o1, dGeomID o2)
      {
        Environment*env = reinterpret_cast<Environment *>(data);
        env->_collision(o1, o2);
      }
      virtual void _collision(dGeomID o1, dGeomID o2);
    //public: // ??
       // attributes
      dWorldID _world_id;
      dSpaceID _space_id;
      dGeomID _ground;
      dJointGroupID _contactgroup;
      double _pitch, _roll, _z;
    double angle;
	double _mu;
  };
}


#endif
