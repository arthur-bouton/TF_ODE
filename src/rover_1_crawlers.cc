#include "rover.hh"


using namespace ode;
using namespace Eigen;
using namespace std;


namespace robot
{


Crawler_1::Crawler_1( Environment& env, const Vector3d& pose, float torque_amplitude, float dt_torque, float angle_rate, float angle_span ) :
                 Rover_1( env, pose ), _torque_amplitude( torque_amplitude ), _angle_rate( angle_rate ), _angle_span( angle_span ), _phase( 1 )
{
	if ( angle_rate < 0 )
		_angle_rate = steering_max_vel;
	
	if ( angle_span < 0 )
		_angle_span = steering_angle_max;
	
	boggie_max_torque = torque_amplitude;
	
	_dtorque_dt = 2*torque_amplitude/dt_torque;

	_crawling_mode = true;
}


void Crawler_1::PrintControls( double time )
{
	printf( "%f,", time );
	for ( auto wheel_speed : _W )
		printf( "%f,", wheel_speed );
	printf( "%f,%f\n", _steering_rate, _boggie_torque );
	fflush( stdout );
}


void Crawler_1::_InternalControl( double delta_t )
{
	switch ( _phase )
	{
		case 1 :
			_boggie_torque += _dtorque_dt*delta_t;
			if ( _boggie_torque >= _torque_amplitude )
			{
				_boggie_torque = _torque_amplitude;
				++_phase;
			}
			break;
		case 2 :
			_steering_rate = _angle_rate;
			if ( GetSteeringTrueAngle() >= _angle_span )
			{
				_steering_rate = 0;
				++_phase;
			}
			break;
		case 3 :
			_boggie_torque -= _dtorque_dt*delta_t;
			if ( _boggie_torque <= -_torque_amplitude )
			{
				_boggie_torque = -_torque_amplitude;
				++_phase;
			}
			break;
		case 4 :
			_steering_rate = -_angle_rate;
			if ( GetSteeringTrueAngle() <= -_angle_span )
			{
				_steering_rate = 0;
				_phase = 1;
			}
			break;
	}
}


}
