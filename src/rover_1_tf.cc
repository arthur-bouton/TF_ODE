#include "rover_tf.hh"
#include <random>


using namespace ode;
using namespace Eigen;
namespace p = boost::python;


namespace robot
{


Rover_1_tf::Rover_1_tf( Environment& env, const Vector3d& pose, const char* path_to_actor_model_dir, const int seed ) :
                        Rover_1( env, pose ),
						_total_reward( 0 ),
						_exploration( false )
{
	_last_pos = GetPosition();


	// Import the actor model:
	_actor_model_ptr = TF_model<float>::ptr_t( new TF_model<float>( path_to_actor_model_dir, { 17 }, { 2, 2 } ) );
	

	// Initialization of the random number engine:
	if ( seed < 0 )
	{
		std::random_device rd;
		_rd_gen = std::mt19937( rd() );
	}
	else
		_rd_gen = std::mt19937( seed );
    _normal_distribution = std::normal_distribution<double>( 0., 1. );
}


p::list Rover_1_tf::GetState() const
{
	p::list state;

	state.append( GetDirection() );
	state.append( GetSteeringTrueAngle() );
	state.append( GetRollAngle() );
	state.append( GetPitchAngle() );
	state.append( GetBoggieAngle() );
	const Vector3d* list[] = { _front_ft_sensor.GetForces(), _front_ft_sensor.GetTorques(), _rear_ft_sensor.GetForces(), _rear_ft_sensor.GetTorques() };
	for ( int i = 0 ; i < 4 ; i++ )
		for ( int j = 0 ; j < 3 ; j++ )
			state.append( list[i]->coeff( j ) );
	//for ( int i = 0 ; i < NBWHEELS ; i++ )
		//state.append( _torque_output[i] );

	return state;
}


double Rover_1_tf::_ComputeReward( double delta_t )
{
	Vector3d new_pos = GetPosition();
	Vector3d pos_diff = new_pos - _last_pos;

	// Reward the forward advance:
	//double reward = pos_diff[0]*fabs( pos_diff[0] ) + pos_diff[2]*fmax( 0., pos_diff[2] );
	//double reward = -pos_diff[0]*fabs( pos_diff[0] );
	double reward = -pos_diff[0];
	// Scaling:
	//reward *= 100./( delta_t*delta_t );
	reward *= 25./delta_t;

	// Penalise side deviation:
	reward -= fabs( new_pos[1] )*0.5;

	// Penalise the use of boggie torque:
	reward -= fabs( _boggie_torque )/boggie_max_torque*0.5;

	_last_pos = new_pos;
	return reward;
}


void Rover_1_tf::_InternalControl( double delta_t )
{
	// Get the reward obtained since last call:
	double reward = _ComputeReward( delta_t );
	_total_reward += reward;

	// Get the current state of the robot:
	p::list current_state = GetState();

	// Store the latest experience:
	if ( p::len( _last_state ) > 0 )
		_experience.append( p::make_tuple( _last_state, p::make_tuple( _steering_rate, _boggie_torque ), reward, false, current_state ) );


#ifdef PRINT_TRANSITIONS
	if ( p::len( _last_state ) > 0 )
	{
		for ( int i = 0 ; i < p::len( _last_state ) ; i++ )
			printf( "%f ", float( p::extract<float>( _last_state[i] ) ) );
		printf( "%f %f", _steering_rate, _boggie_torque );
		for ( int i = 0 ; i < p::len( current_state ) ; i++ )
			printf( " %f", float( p::extract<float>( current_state[i] ) ) );
		printf( "\n" );
		fflush( stdout );
	}
#endif


	// Determine the next action:

	// Setup the inputs:
	std::vector<float> input_vector;
	for ( int i = 0 ; i < p::len( current_state ) ; i++ )
		input_vector.push_back( float( p::extract<float>( current_state[i] ) ) );

	// Run the model:
	std::vector<std::vector<float>> output_vectors = _actor_model_ptr->infer( { input_vector } );

	// Extract the unbounded mean values of the action:
	double unbounded_steering_rate = output_vectors[0][0];
	double unbounded_boggie_torque = output_vectors[0][1];

	// Add exploration noise:
	if ( _exploration )
	{
		unbounded_steering_rate += _normal_distribution( _rd_gen )*output_vectors[1][0];
		unbounded_boggie_torque += _normal_distribution( _rd_gen )*output_vectors[1][1];
	}

	// Squash and assign the next action:
	_steering_rate =  steering_max_vel*tanh( unbounded_steering_rate );
	_boggie_torque = boggie_max_torque*tanh( unbounded_boggie_torque );


#ifdef PRINT_STATE_AND_ACTIONS
	for ( int i = 0 ; i < p::len( current_state ) ; i++ )
		printf( "%f ", float( p::extract<float>( current_state[i] ) ) );
	printf( "%f %f\n", _steering_rate, _boggie_torque );
	fflush( stdout );
#endif


	_last_state = current_state;
}


}
