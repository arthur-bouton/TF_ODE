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

#ifndef OBJECT_HH
#define OBJECT_HH

#include <iostream>
#include <assert.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include "environment.hh"
#include "visitor.hh"

#include <time.h>
#include <sys/time.h>


namespace ode
{


class Servo;


class Object
{
	public:

	typedef boost::shared_ptr<Object> ptr_t;

	Object( Environment & env, const Eigen::Vector3d & pos );

	virtual ~Object();

	inline Eigen::Vector3d get_init_pos() const { return _init_pos; }

	virtual dBodyID get_body() const;
	virtual std::vector<dGeomID> get_geoms() const;

	/// ask the current position to ode
	Eigen::Vector3d get_pos() const;
	Eigen::Vector3d get_rot() const;
	Eigen::Vector3d get_vel() const;
	Eigen::Vector3d get_angvel() const;

	/// set an absolute rotation ( euler angles )
	void set_rotation( double phi, double theta, double psi );
	/// set an absolute rotation ( axis and angle )
	void set_rotation( double ax, double ay, double az, double angle );
	void set_rotation( const Eigen::Vector3d& a1, const Eigen::Vector3d& a2 );

	/// const visitor, useful for example for a 3d renderer
	virtual void accept( ConstVisitor& v ) const = 0;

	/// connect a servo. Used by Panels ( called by the constructor ) to
	/// change their shape according to their sweep angle
	void add_servo( Servo*servo );
	void add_servo2( Servo*servo2 );
	const Servo& get_servo( void ) const;
	const Servo& get_servo2( void ) const;

	/// ask to ode the speed of a given point in the object
	/// the given point must be in world
	/// the result is in world coordinate
	Eigen::Vector3d get_vground( const Eigen::Vector3d& v ) const;

	///attach the object to the static environment
	void fix();
	/// detach the object
	void unfix();
	/// attach to the static environment along the axis 'axis'
	/// ( the object will be able to move only along this axis )
	void fix_along_axis( const Eigen::Vector3d& axis );
	/// true if fixed
	bool get_fix() const;

	const Environment& get_env() const;

	virtual double get_mass() const;
	virtual Eigen::Vector3d get_cg() const;
	virtual Eigen::Matrix3d get_inertia() const;
	virtual void set_mass_parameters( double m,
									  Eigen::Vector3d cg,
								  	  Eigen::Matrix3d I );
	virtual void set_inertia( double I11, double I22, double I33,
							  double I12 = 0, double I13 = 0, double I23 = 0 );

	const char* get_collision_group() const;
	virtual void set_collision_group( const char* group );

	contact_type get_contact_type() const;
	virtual void set_contact_type( contact_type type );

	bool casts_shadow() const;
	void disable_shadow_casting();

	const float* get_color() const;
	virtual void set_color( float r, float g, float b );

	float get_alpha() const;
	virtual void set_alpha( float a );

	inline void set_mesh( const char* path ) { _mesh_path = path; }
	inline const char* get_mesh_path() const { return _mesh_path; }

	Object* add_box_geom( double l, double w, double h );
	Object* add_sphere_geom( double r );
	Object* add_cylinder_geom( double r, double l );
	Object* add_capcyl_geom( double r, double l );

	Object* set_geom_abs_pos( const Eigen::Vector3d& pos, int index = -1 );
	Object* set_geom_rel_pos( const Eigen::Vector3d& pos, int index = -1 );
	Object* set_geom_rot( double phi, double theta, double psi, int index = -1 );
	Object* set_geom_rot( double ax, double ay, double az, double angle, int index = -1 );
	Object* set_geom_rot( const Eigen::Vector3d& a1, const Eigen::Vector3d& a2, int index = -1 );

	protected:

	void init();

	// does not copy servos ( they must be copied later )
	void _copy( const Object& o );

	dMass _m;
	dBodyID _body;
	std::vector<dGeomID> _geoms;
	Eigen::Vector3d _init_pos;
	Environment& _env;
	Servo*_servo;
	Servo*_servo2;
	dJointID _fix;
	bool _casts_shadow;
	float *_RGB, _alpha;
	const char* _mesh_path;
};


}

#endif
