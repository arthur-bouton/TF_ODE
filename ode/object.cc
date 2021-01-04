/*
** Copyright (C) 2014 Arthur BOUTON
** Copyright (C) 2010 mandor
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

#include "object.hh"


namespace ode
{


Object::Object( Environment & env, const Eigen::Vector3d& pos ) :
				_body( 0x0 ),
				_init_pos( pos ),
				_env( env ),
				_servo( 0x0 ),
				_servo2( 0x0 ),
				_fix( 0x0 ),
				_casts_shadow( true ),
				_RGB( NULL ), _alpha( 1 ),
				_mesh_path( nullptr )
{
}


Object::~Object()
{
	if ( _body )
		dBodyDestroy( _body );
	if ( ! _geoms.empty() )
	{
		for ( dGeomID g : _geoms )
		{
			delete ( collision_feature* ) dGeomGetData( g );
			dGeomDestroy( g );
		}
		_geoms.clear();
	}
	if ( _RGB != NULL )
		delete _RGB;
}

dBodyID Object::get_body() const { return _body; }
std::vector<dGeomID> Object::get_geoms() const { return _geoms; }


Eigen::Vector3d Object::get_pos() const
{
	const dReal* pos = dBodyGetPosition( get_body() );
	return Eigen::Vector3d( pos[0], pos[1], pos[2] );
}

Eigen::Vector3d Object::get_rot() const
{
	//x-y-z convention
	// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	// see also : http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler
	// ode : [ w, x, y, z ], where w is the real part and (x, y, z) form the vector part.
	const dReal* q = dBodyGetQuaternion( get_body() );
	double
	phi = 0, // bank
	theta = 0, // attitude
	psi = 0; // heading
	double k = q[1] * q[2] + q[3] * q[0];

	if ( fabs( k - 0.5 ) < 1e-5 ) // north pole
	{
		psi = 2 * atan2( q[1], q[0] );
		theta = M_PI / 2.0;
		phi = 0;
	}
	else if ( ( k + 0.5 ) < 1e-5 )// south pole
	{
		psi = -2 * atan2( q[1], q[0] );
		theta = -M_PI / 2.0;
		phi = 0;
	}
	else
	{
		phi = atan2( 2 * ( q[0] * q[1] + q[2] * q[3] ),
		1 - 2 * ( q[1] * q[1] + q[2] * q[2] ) );
		theta = asin( 2 * ( q[0] * q[2] -q[3] * q[1] ) );
		psi = atan2( 2 * ( q[0] * q[3] + q[1] * q[2] ),
		1 - 2 * ( q[2] * q[2] + q[3] * q[3] ) );
	}
	return Eigen::Vector3d( phi, theta, psi );
}

Eigen::Vector3d Object::get_vel() const
{
	const dReal* vel = dBodyGetLinearVel( get_body() );
	return ode_to_vectord( vel );
}

Eigen::Vector3d Object::get_angvel() const
{
	const dReal* angvel = dBodyGetAngularVel( get_body() );
	return ode_to_vectord( angvel );
}


void Object::set_rotation( double phi, double theta, double psi )
{
	dMatrix3 r;
	dRFromEulerAngles( r, phi, theta, psi );
	dBodySetRotation( get_body(), r );
}

void Object::set_rotation( double ax, double ay, double az, double angle )
{
	dMatrix3 r;
	dRFromAxisAndAngle( r, ax, ay, az, angle );
	dBodySetRotation( get_body(), r );
}

void Object::set_rotation( const Eigen::Vector3d& a1, const Eigen::Vector3d& a2 )
{
	dMatrix3 r;
	dRFrom2Axes ( r, a1.x(), a1.y(), a1.z(), a2.x(), a2.y(), a2.z() );
	dBodySetRotation( get_body(), r );
}


void Object::add_servo( Servo*servo ) { _servo = servo; }

void Object::add_servo2( Servo*servo2 ) { _servo2 = servo2; }

const Servo& Object::get_servo( void ) const { assert( _servo ); return *_servo; }

const Servo& Object::get_servo2( void ) const { assert( _servo2 ); return *_servo2; }


Eigen::Vector3d Object::get_vground( const Eigen::Vector3d& v ) const
{
	dReal res[3];
	dBodyGetPointVel( get_body(), v.x(), v.y(), v.z(), res );
	return ode_to_vectord( res );
}


void Object::fix()
{
	if ( _fix )
		return;
	fix_along_axis( Eigen::Vector3d( 0, 0, 1 ) );
	dJointSetSliderParam( _fix, dParamLoStop, 0 );
	dJointSetSliderParam( _fix, dParamHiStop, 0 );
}

void Object::unfix()
{
	if ( !_fix )
		return;
	dJointDestroy( _fix );
	_fix = 0x0;
}

void Object::fix_along_axis( const Eigen::Vector3d& axis )
{
	if ( _fix )
		return;
	_fix = dJointCreateSlider( _env.get_world(), 0 );
	dJointAttach( _fix, _body, 0 );
	dJointSetSliderAxis( _fix, axis[0], axis[1], axis[2] );
}

bool Object::get_fix() const { return _fix == 0; }


const Environment& Object::get_env() const { return _env; }


double Object::get_mass() const { return _m.mass; }

Eigen::Vector3d Object::get_cg() const
{
	return Eigen::Vector3d( _m.c[0], _m.c[1], _m.c[2] );
}

Eigen::Matrix3d Object::get_inertia() const
{
	Eigen::Matrix3d I;
	I << _m.I[0*4+0], _m.I[1*4+0], _m.I[2*4+0],
		 _m.I[0*4+1], _m.I[1*4+1], _m.I[2*4+1],
		 _m.I[0*4+2], _m.I[1*4+2], _m.I[2*4+2];
	return I;
}

void Object::set_mass_parameters( double m,
							  Eigen::Vector3d cg,
							  Eigen::Matrix3d I )
{
	dMassSetParameters( &_m, m,
						cg.x(), cg.y(), cg.z(),
						I( 1, 1 ), I( 2, 2 ), I( 3, 3 ),
						I( 1, 2 ), I( 1, 3 ), I( 2, 3 ) );
}

void Object::set_inertia( double I11, double I22, double I33,
					  double I12, double I13, double I23 )
{
	dMassSetParameters( &_m, _m.mass,
						_m.c[0], _m.c[1], _m.c[2],
						I11, I22, I33,
						I12, I13, I23 );
}


const char* Object::get_collision_group( int index ) const
{
	if ( _geoms.size() > index )
	{
		collision_feature* feature = ( collision_feature* ) dGeomGetData( _geoms[index] );
		if ( feature != NULL )
			return feature->group;
	}
	return NULL;
}

void Object::set_all_collision_group( const char* group )
{
	if ( ! _geoms.empty() )
	{
		for ( dGeomID g : _geoms )
		{
			collision_feature* feature = ( collision_feature* ) dGeomGetData( g );
			if ( feature != NULL )
				feature->group = group;
			else
				dGeomSetData( g, new collision_feature( group ) );
		}
	}
}

void Object::set_collision_group( const char* group, int index )
{
	if ( ! _geoms.empty() )
	{
		collision_feature* feature = ( collision_feature* ) dGeomGetData( index < 0 ? _geoms.back() : _geoms[index] );
		if ( feature != NULL )
			feature->group = group;
		else
			dGeomSetData( ( index < 0 ? _geoms.back() : _geoms[index] ), new collision_feature( group ) );
	}
}

contact_type Object::get_contact_type( int index ) const
{
	if ( _geoms.size() > index )
	{
		collision_feature* feature = ( collision_feature* ) dGeomGetData( _geoms[index] );
		if ( feature != NULL )
			return feature->type;
		else
			return HARD;
	}
	else
		return DISABLED;
}

void Object::set_contact_type( contact_type type, int index )
{
	if ( ! _geoms.empty() )
	{
		collision_feature* feature = ( collision_feature* ) dGeomGetData( index < 0 ? _geoms.back() : _geoms[index] );
		if ( feature != NULL )
			feature->type = type;
		else
			dGeomSetData( ( index < 0 ? _geoms.back() : _geoms[index] ), new collision_feature( type ) );
	}
}

void Object::set_all_contact_type( contact_type type )
{
	if ( ! _geoms.empty() )
	{
		for ( dGeomID g : _geoms )
		{
			collision_feature* feature = ( collision_feature* ) dGeomGetData( g );
			if ( feature != NULL )
				feature->type = type;
			else
				dGeomSetData( g, new collision_feature( type ) );
		}
	}
}

void Object::set_collision_callback( std::function<void(collision_feature*)> callback, int index )
{
	if ( ! _geoms.empty() )
	{
		collision_feature* feature = ( collision_feature* ) dGeomGetData( index < 0 ? _geoms.back() : _geoms[index] );
		if ( feature != NULL )
			feature->callback = callback;
		else
			dGeomSetData( ( index < 0 ? _geoms.back() : _geoms[index] ), new collision_feature( callback ) );
	}
}

void Object::set_all_collision_callback( std::function<void(collision_feature*)> callback )
{
	if ( ! _geoms.empty() )
	{
		for ( dGeomID g : _geoms )
		{
			collision_feature* feature = ( collision_feature* ) dGeomGetData( g );
			if ( feature != NULL )
				feature->callback = callback;
			else
				dGeomSetData( g, new collision_feature( callback ) );
		}
	}
}


void Object::disable_shadow_casting() { _casts_shadow = false; }

bool Object::casts_shadow() const { return _casts_shadow; }


const float* Object::get_color() const { return _RGB; }

void Object::set_color( float r, float g, float b )
{
	if ( _RGB == NULL )
		_RGB = new float[3];
	_RGB[0] = r;
	_RGB[1] = g;
	_RGB[2] = b;
}


float Object::get_alpha() const { return _alpha; }

void Object::set_alpha( float a ) { _alpha = a; }


void Object::init()
{
	_body = dBodyCreate( _env.get_world() );
	dBodySetPosition( _body,
					  _init_pos.x(),
					  _init_pos.y(),
					  _init_pos.z() );
	dBodySetData( _body, this );
}


void Object::_copy( const Object& o )
{
	_body = dBodyCreate( _env.get_world() );

	dQuaternion quat;
	dBodyCopyQuaternion( o._body, quat );
	dVector3 pos;
	dBodyCopyPosition( o._body, pos );

	dBodySetPosition( _body, pos[0], pos[1], pos[2] );
	dBodySetQuaternion( _body, quat );

	dBodyGetMass( o._body, &_m );
	dBodySetMass( _body, &_m );

	_fix = o._fix;
	_init_pos = o.get_pos();

	dBodySetData( _body, this );
}


Object* Object::set_geom_abs_pos( const Eigen::Vector3d& pos, int index )
{
	dGeomSetOffsetWorldPosition( ( index < 0 ? _geoms.back() : _geoms[index] ), pos.x(), pos.y(), pos.z() );
	return this;
}

Object* Object::set_geom_rel_pos( const Eigen::Vector3d& pos, int index )
{
	dGeomSetOffsetPosition( ( index < 0 ? _geoms.back() : _geoms[index] ), pos.x(), pos.y(), pos.z() );
	return this;
}

Object* Object::set_geom_rot( double phi, double theta, double psi, int index )
{
	dMatrix3 r;
	dRFromEulerAngles( r, phi, theta, psi );
	dGeomSetOffsetRotation( ( index < 0 ? _geoms.back() : _geoms[index] ), r );
	return this;
}

Object* Object::set_geom_rot( double ax, double ay, double az, double angle, int index )
{
	dMatrix3 r;
	dRFromAxisAndAngle( r, ax, ay, az, angle );
	dGeomSetOffsetRotation( ( index < 0 ? _geoms.back() : _geoms[index] ), r );
	return this;
}

Object* Object::set_geom_rot( const Eigen::Vector3d& a1, const Eigen::Vector3d& a2, int index )
{
	dMatrix3 r;
	dRFrom2Axes( r, a1.x(), a1.y(), a1.z(), a2.x(), a2.y(), a2.z() );
	dGeomSetOffsetRotation( ( index < 0 ? _geoms.back() : _geoms[index] ), r );
	return this;
}


Object* Object::add_box_geom( double l, double w, double h )
{
	dGeomID g = dCreateBox( _env.get_space(), l , w, h );
	dGeomSetBody( g, _body );
	_geoms.push_back( g );
	return this;
}

Object* Object::add_sphere_geom( double r )
{
	dGeomID g = dCreateSphere( _env.get_space(), r );
	dGeomSetBody( g, _body );
	_geoms.push_back( g );
	return this;
}

Object* Object::add_cylinder_geom( double r, double l )
{
	dGeomID g = dCreateCylinder( _env.get_space(), r, l );
	dGeomSetBody( g, _body );
	_geoms.push_back( g );
	return this;
}

Object* Object::add_capcyl_geom( double r, double l )
{
	dGeomID g = dCreateCCylinder( _env.get_space(), r, l - r*2 );
	dGeomSetBody( g, _body );
	_geoms.push_back( g );
	return this;
}


}
