#include "quadruped.hh"
#include "ode/box.hh"
#include "ode/capped_cyl.hh"
#include <random>


#define RAD_TO_DEG 57.29577951308232
#define DEG_TO_RAD 0.017453292519943295


using namespace ode;
using namespace Eigen;
namespace p = boost::python;


namespace robot
{


Quadruped::Quadruped( Environment& env, const Vector3d& pose, const char* path_to_actor_model_dir, const int seed ) :
					  _ic_period( 0 ),
					  _ic_clock( 0 ),
					  _ic_activated( true ),
					  _total_reward( 0 ),
					  _exploration( false ),
					  _collision( false )
{

	float body_mass   = 5;
	float body_length = 0.40;
	float body_height = 0.10;
	float body_width  = 0.20;
	float body_elev   = 0.30;

	_main_body = Object::ptr_t( new Box( env,
										 pose + Vector3d( 0, 0, body_elev ),
										 body_mass, body_length, body_width, body_height ) );
	_bodies.push_back( _main_body );

	
	float shoulder_mass      = 0.5;
	float shoulder_radius    = 0.025;
	float shoulder_length    = 0.07;
	float shoulder_max_angle = 30;
	float shoulder_Kp        = 1;
	float shoulder_max_speed = 90;
	
	float upperleg_mass       = 0.2;
	float upperleg_radius     = 0.02;
	float upperleg_length     = 0.15;
	float upperleg_init_angle = -30;
	float upperleg_max_angle  = 30;
	float upperleg_Kp         = 1;
	float upperleg_max_speed  = 90;
	
	float lowerleg_mass       = 0.1;
	float lowerleg_radius     = 0.01;
	float lowerleg_length     = 0.15;
	float lowerleg_init_angle = 30;
	float lowerleg_max_angle  = 30;
	float lowerleg_Kp         = 1;
	float lowerleg_max_speed  = 90;

	// Assign a callback to detect if the shoulders touch the ground:
	std::function<void(collision_feature*)> collision_callback = [&]( collision_feature* collided_object )
	{
		if ( collided_object != nullptr && strcmp( collided_object->group, "ground" ) == 0 )
			_collision = true;
	};

	for ( int i = 0 ; i < 4 ; i++ )
	{
		ode::Object::ptr_t shoulder( new CappedCyl( env,
													pose + Vector3d( ( i/2 ? -1 : 1 )*( body_length - shoulder_length )/2,
																	 ( i%2 ? -1 : 1 )*body_width/2,
																	 body_elev - body_height/2 + shoulder_radius ),
													shoulder_mass, shoulder_radius, shoulder_length ) );
		shoulder->set_rotation( 0, M_PI/2, 0 );
		shoulder->set_all_collision_callback( collision_callback );
		_bodies.push_back( shoulder );

		Servo::ptr_t shoulder_servo( new Servo( env,
												*shoulder, *_main_body,
												shoulder->get_pos(),
												Vector3d( 1, 0, 0 ),
												shoulder_Kp,
												shoulder_max_speed*DEG_TO_RAD,
												-shoulder_max_angle*DEG_TO_RAD, shoulder_max_angle*DEG_TO_RAD ) );
		_servos.push_back( shoulder_servo );


		ode::Object::ptr_t upperleg( new CappedCyl( env,
													pose + shoulder->get_pos() + Vector3d( upperleg_length*sin( upperleg_init_angle*DEG_TO_RAD ),
																						   0,
																						   -upperleg_length*cos( upperleg_init_angle*DEG_TO_RAD ) )/2,
													upperleg_mass, upperleg_radius, upperleg_length ) );
		upperleg->set_rotation( 0, upperleg_init_angle*DEG_TO_RAD, 0 );
		_bodies.push_back( upperleg );

		Servo::ptr_t upperleg_servo( new Servo( env,
												*upperleg, *shoulder,
												shoulder->get_pos(),
												Vector3d( 0, 1, 0 ),
												upperleg_Kp,
												upperleg_max_speed*DEG_TO_RAD,
												-upperleg_max_angle*DEG_TO_RAD, upperleg_max_angle*DEG_TO_RAD ) );
		_servos.push_back( upperleg_servo );


		Vector3d elbow_pos = shoulder->get_pos() + ( upperleg->get_pos() - shoulder->get_pos() )*2;
		ode::Object::ptr_t lowerleg( new CappedCyl( env,
													pose + elbow_pos + Vector3d( lowerleg_length*sin( lowerleg_init_angle*DEG_TO_RAD ),
																				 0,
																				 -lowerleg_length*cos( lowerleg_init_angle*DEG_TO_RAD ) )/2,
													lowerleg_mass, lowerleg_radius, lowerleg_length ) );
		lowerleg->set_rotation( 0, lowerleg_init_angle*DEG_TO_RAD, 0 );
		_bodies.push_back( lowerleg );

		Servo::ptr_t lowerleg_servo( new Servo( env,
												*lowerleg, *upperleg,
												elbow_pos,
												Vector3d( 0, 1, 0 ),
												lowerleg_Kp,
												lowerleg_max_speed*DEG_TO_RAD,
												-lowerleg_max_angle*DEG_TO_RAD, lowerleg_max_angle*DEG_TO_RAD ) );
		_servos.push_back( lowerleg_servo );
	}


	// Import the actor model:
	_actor_model_ptr = TF_model<float>::ptr_t( new TF_model<float>( path_to_actor_model_dir, { 17 }, { 12 } ) );
	

	// Initialization of the random number engine:
	if ( seed < 0 )
	{
		std::random_device rd;
		_rd_gen = std::mt19937( rd() );
	}
	else
		_rd_gen = std::mt19937( seed );
    _normal_distribution = std::normal_distribution<double>( 0., 1. );
	_uniform_distribution = std::uniform_real_distribution<double>( -1., 1. );

	// State scaling before feeding the neural network:
	_state_scaling = { 90, 90, 90, 90, 90 };
	for ( int i = 0 ; i < _servos.size() ; i++ )
	{
		_state_scaling.push_back( shoulder_max_angle );
		_state_scaling.push_back( upperleg_max_angle );
		_state_scaling.push_back( lowerleg_max_angle );
	}

	// Action scaling from the outputs of the neural network:
	for ( int i = 0 ; i < _servos.size() ; i++ )
	{
		_action_scaling.push_back( shoulder_max_speed );
		_action_scaling.push_back( upperleg_max_speed );
		_action_scaling.push_back( lowerleg_max_speed );
	}

	// Initialize the actions:
	for ( int i = 0 ; i < _servos.size() ; i++ )
		_actions.append( 0 );


	_last_pos = GetPosition();
}


Vector3d Quadruped::GetPosition() const
{
	dVector3 center_pos;
	dBodyGetRelPointPos( _main_body->get_body(), 0, 0, 0, center_pos );
	return Vector3d( center_pos[0], center_pos[1], center_pos[2] );
}


double Quadruped::GetDirection() const
{
	dVector3 vec;
	dBodyVectorToWorld( _main_body->get_body(), 1, 0, 0, vec );
	double direction = asin( vec[1] )*RAD_TO_DEG;
	if ( vec[0] < 0 )
		direction = ( vec[1] > 0 ? 1 : -1 )*180 - direction;
	return direction;
}


bool Quadruped::IsUpsideDown() const
{
	dVector3 vec;
	dBodyVectorToWorld( _main_body->get_body(), 0, 0, 1, vec );
	return ( vec[2] < 0 ? true : false );
}


double Quadruped::GetRollAngle() const
{
	dVector3 vec;
	dBodyVectorToWorld( _main_body->get_body(), 0, 1, 0, vec );
	return asin( vec[2] )*RAD_TO_DEG;
}


double Quadruped::GetPitchAngle() const
{
	dVector3 vec;
	dBodyVectorToWorld( _main_body->get_body(), 1, 0, 0, vec );
	return asin( -vec[2] )*RAD_TO_DEG;
}


void Quadruped::GetTiltRates( double& roll_rate, double& pitch_rate ) const
{
	const dReal* angular_vel = dBodyGetAngularVel( _main_body->get_body() );
	dVector3 vec;
	dBodyVectorFromWorld( _main_body->get_body(), angular_vel[0], angular_vel[1], angular_vel[2], vec );
	roll_rate = vec[0]*RAD_TO_DEG;
	pitch_rate = vec[1]*RAD_TO_DEG;
}


p::list Quadruped::GetState() const
{
	p::list state;

	state.append( GetDirection() );
	state.append( GetRollAngle() );
	state.append( GetPitchAngle() );
	double roll_rate, pitch_rate;
	GetTiltRates( roll_rate, pitch_rate );
	state.append( roll_rate );
	state.append( pitch_rate );
	for ( auto servo : _servos )
		state.append( servo->get_true_angle()*RAD_TO_DEG );

	return state;
}


double Quadruped::_ComputeReward( double delta_t )
{
	Vector3d new_pos = GetPosition();
	Vector3d pos_diff = new_pos - _last_pos;

	// Reward the forward advance:
	double reward = pos_diff[0];
	// Scaling:
	reward *= 2/delta_t;

	// Penalise side deviation:
	reward -= fabs( new_pos[1] )*2;

	// Add a penalty if a motor bulk touches an obstacle:
	//if ( _collision )
	//{
		//reward -= 1;
		//_collision = false;
	//}

	//printf( "dx: %f y: %f reward: %f\n", pos_diff[0], fabs( new_pos[1] ), reward );
	_last_pos = new_pos;
	return reward;
}


void Quadruped::_InternalControl( double delta_t )
{
	// Get the reward obtained since last call:
	double reward = _ComputeReward( delta_t );
	_total_reward += reward;

	// Get the current state of the robot:
	p::list current_state = GetState();

	// Store the latest experience:
	if ( p::len( _last_state ) > 0 )
		_experience.append( p::make_tuple( _last_state, _actions, reward, false, current_state ) );


	// Determine the next action:

	 //Setup the inputs:
	std::vector<float> input_vector;
	for ( int i = 0 ; i < p::len( current_state ) ; i++ )
		input_vector.push_back( float( p::extract<float>( current_state[i] ) )/_state_scaling[i] );

	// Run the model:
	std::vector<std::vector<float>> output_vectors = _actor_model_ptr->infer( { input_vector } );

	for ( int i = 0 ; i < _servos.size() ; i++ )
	{
		// Extract the unscaled action:
		double unscaled_speed = output_vectors[0][i];

		// Add exploration noise:
		if ( _exploration )
		{
			unscaled_speed += _normal_distribution( _rd_gen )*0.05;
			unscaled_speed = std::max( -1., std::min( unscaled_speed, 1. ) );
		}
		//double draw = ( _uniform_distribution( _rd_gen ) + 1 )/2;
		//if ( _exploration && draw > 0.95 )
			//unscaled_speed = _uniform_distribution( _rd_gen );

		// Scale the action:
		double desired_speed = _action_scaling[i]*unscaled_speed;

		// Apply the control:
		_servos[i]->set_desired_vel( desired_speed*DEG_TO_RAD );

		// Store the action:
		_actions[i] = desired_speed;
	}


#ifdef PRINT_STATE
	for ( int i = 0 ; i < p::len( current_state ) ; i++ )
		printf( "%f ", float( p::extract<float>( current_state[i] ) ) );
	printf( "\n" );
	fflush( stdout );
#endif

#ifdef PRINT_ACTIONS
	for ( int i = 0 ; i < p::len( _actions ) ; i++ )
		printf( "%f ", float( p::extract<float>( _actions[i] ) ) );
	printf( "\n" );
	fflush( stdout );
#endif


	_last_state = current_state;
}


void Quadruped::next_step( double dt )
{
	_ic_clock += dt;
	if ( _ic_activated && _ic_clock >= _ic_period )
	{
		_InternalControl( _ic_clock );

		_ic_clock = 0;
		_ic_tick = true;
	}
	else
		_ic_tick = false;

	Robot::next_step( dt );
}


//Quadruped::~Quadruped()
//{
//}


}
