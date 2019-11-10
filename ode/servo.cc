#include "servo.hh"


#define RAD_TO_DEG 57.29577951308232
#define DEG_TO_RAD 0.017453292519943295


namespace ode
{


void Servo::_build()
{
	_joint = dJointCreateHinge( _env.get_world(), 0 );
	dJointAttach( _joint, _o1.get_body(), _o2.get_body() );
	dJointSetHingeAnchor( _joint, _anchor.x(), _anchor.y(), _anchor.z() );
	dJointSetHingeAxis( _joint, _axis.x(), _axis.y(), _axis.z() );

	dJointSetHingeParam( _joint, dParamFMax, dInfinity );
	//dJointSetHingeParam( _joint, dParamLoStop, _min );
	//dJointSetHingeParam( _joint, dParamHiStop, _max );

	_o2.add_servo( this );
	_o1.add_servo2( this );
}


Servo::Servo( Environment& env,
			  Object& o1, Object& o2,
			  const Eigen::Vector3d& anchor,
			  const Eigen::Vector3d& axis,
			  double Kp,
			  double vel_max,
			  dReal min, dReal max ) :
			  _env( env ),
			  _o1( o1 ), _o2( o2 ),
			  _anchor( anchor ),
			  _axis( axis ),
			  _min( min ), _max( max ),
			  _passive( false ),
			  _Kp( Kp ),
			  _vel_max( vel_max ),
			  _angle( 0 ),
			  _mode( POS ),
			  _vel( 0 )
{
	_build();
}


Servo::Servo( const Servo& s, Environment& env, Object& o1, Object& o2 ) :
			  _env( env ),
			  _o1( o1 ), _o2( o2 ),
			  _anchor( s.get_anchor() ),
			  _axis( s.get_axis() ),
			  _min( s.get_lim_min() ), _max( s.get_lim_max() ),
			  _passive( s.is_passive() ),
			  _Kp( s.get_Kp() ),
			  _vel_max( s.get_vel_max() ),
			  _angle( s.get_angle() )
{
	_build();
}


Servo::ptr_t Servo::clone( Environment& env, Object& o1, Object& o2 ) const
{ return ptr_t( new Servo( *this, env, o1, o2 ) ); }


const Object& Servo::get_o1() const { return _o1; }
const Object& Servo::get_o2() const { return _o2; }
const Eigen::Vector3d& Servo::get_anchor() const { return _anchor; }
const Eigen::Vector3d& Servo::get_axis() const { return _axis; }


const dJointID& Servo::get_joint() const { return _joint; }


dReal Servo::get_lim_min() const { return _min; }
dReal Servo::get_lim_max() const { return _max; }
void Servo::set_lim_min( dReal min )
{
	_min = min;
	//dJointSetHingeParam( _joint, dParamLoStop, _min );
}
void Servo::set_lim_max( dReal max )
{
	_max = max;
	//dJointSetHingeParam( _joint, dParamHiStop, _max );
}


bool Servo::is_passive() const { return _passive; }
void Servo::set_passive()
{
	_passive = true;
	dJointSetHingeParam( _joint, dParamFMax, 0 );
}
void Servo::set_active()
{
	_passive = false;
	dJointSetHingeParam( _joint, dParamFMax, dInfinity );
}


double Servo::get_angle() const { return _angle; }
double Servo::set_angle( double angle )
{
	angle = std::max( _min, angle );
	angle = std::min( angle, _max );
	_angle = angle;
	_mode = POS;
	return _angle;
}


double Servo::get_vel() const { return _vel; }
double Servo::set_vel( double vel )
{
	vel = std::max( -_vel_max, vel );
	vel = std::min( vel, _vel_max );
	dReal angle = dJointGetHingeAngle( _joint );
	if ( angle <= _min and vel < 0 || angle >= _max and vel > 0 )
		vel = 0;
	_vel = vel;
	_mode = VEL;
	dJointSetHingeParam( _joint, dParamVel, _vel );
	return _vel;
}


double Servo::get_Kp() const { return _Kp; }
void Servo::set_Kp( double Kp ) { _Kp = Kp; }


double Servo::get_vel_max() const { return _vel_max; }
void Servo::set_vel_max( double vel_max ) { _vel_max = vel_max; }


void Servo::next_step( double dt )
{
	if ( !_passive )
	{
		dReal angle = dJointGetHingeAngle( _joint );

		if ( _mode == POS )
		{
			double vel = _Kp/dt*( _angle - angle );
			vel = std::max( -_vel_max, vel );
			vel = std::min( vel, _vel_max );
			dJointSetHingeParam( _joint, dParamVel, vel );
		}
		else if ( _mode == VEL )
		{
			if ( angle <= _min and _vel < 0 || angle >= _max and _vel > 0 )
			{
				_vel = 0;
				dJointSetHingeParam( _joint, dParamVel, 0 );
			}
		}
	}
}


double Servo::get_real_angle() const
{
	return dJointGetHingeAngle( _joint );
}


double Servo::get_real_vel() const
{
	return dJointGetHingeAngleRate( _joint );
}


Servo::~Servo()
{
	dJointDestroy( _joint );
}


}
