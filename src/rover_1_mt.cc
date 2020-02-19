#include "rover_mt.hh"


using namespace ode;
using namespace Eigen;
using namespace std;


namespace robot
{


Rover_1_mt::Rover_1_mt( Environment& env, const Vector3d& pose, const std::string yaml_file_path_1, const std::string yaml_file_path_2, const bool oblique_trees ) :
            Rover_1( env, pose ), node_1( 0 ), node_2( 0 )
{
	_lmt_ptr_1 = Linear_model_tree<double>::ptr_t( new Linear_model_tree<double>( yaml_file_path_1, oblique_trees ) );
	_lmt_ptr_2 = Linear_model_tree<double>::ptr_t( new Linear_model_tree<double>( yaml_file_path_2, oblique_trees ) );
}


vector<double> Rover_1_mt::GetState( const bool flip ) const
{
	// Flip or not the left and right to account for the robot's symmetry:
	int flip_coeff = flip ? -1 : 1;

	vector<double> state;

	state.push_back( flip_coeff*GetDirection() );
	state.push_back( flip_coeff*GetSteeringTrueAngle() );
	state.push_back( flip_coeff*GetRollAngle() );
	state.push_back( GetPitchAngle() );
	state.push_back( flip_coeff*GetBoggieAngle() );
	const Vector3d* list[] = { _front_ft_sensor.GetForces(), _front_ft_sensor.GetTorques(), _rear_ft_sensor.GetForces(), _rear_ft_sensor.GetTorques() };
	for ( int i = 0 ; i < 4 ; i++ )
		for ( int j = 0 ; j < 3 ; j++ )
			if ( i != 2 )
				state.push_back( ( ( i + j )%2 == 0 ? 1 : flip_coeff )*list[i]->coeff( j ) );
	//for ( int i = 0 ; i < NBWHEELS ; i++ )
		//state.push_back( _torque_output[i] );

	return state;
}


void Rover_1_mt::InferAction( const vector<double>& state, double& steering_rate, double& boggie_torque, const bool flip )
{
	steering_rate = ( flip ? -1 : 1 )*_lmt_ptr_1->predict( state, node_1 );
	boggie_torque = ( flip ? -1 : 1 )*_lmt_ptr_2->predict( state, node_2 );

#ifdef PRINT
	for ( auto val : state )
		printf( "%f ", val );
	printf( "%f %f\n", _steering_rate, _boggie_torque );
	fflush( stdout );
#endif
}


void Rover_1_mt::_InternalControl( double delta_t )
{
	// Flip the role of left and right if the steering angle is negative:
	bool flip = GetSteeringTrueAngle() < 0;

	// Get the current state of the robot:
	vector<double> state = GetState( flip );

	// Infer the new action:
	InferAction( state, _steering_rate, _boggie_torque, flip );
}


}
