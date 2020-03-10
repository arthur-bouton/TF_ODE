#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "rover_tf.hh"
#include "ode/box.hh"
//#include "ode/heightfield.hh"
#include "renderer/sim_loop.hh"
#include "renderer/osg_text.hh"
//#include <boost/python.hpp>
#include <random>


#define DEFAULT_PATH_TO_TF_MODEL "../training_data/step_06_PER_1_no_sym/selected/rover_training_1_0005"



int main( int argc, char* argv[] )
{
	Py_Initialize();


	// Normal distribution:
	std::random_device rd;
	std::mt19937 gen( rd() );
	std::normal_distribution<double> randn( 0, 1 );
	std::uniform_real_distribution<double> uniform( -1, 1 );


	// [ Dynamic environment ]

	dInitODE();
	// Set the global friction coefficient:
	//ode::Environment env( 0.7 );
	ode::Environment env( 0.6 );


	// [ Robot ]

	const char* path_to_tf_model = DEFAULT_PATH_TO_TF_MODEL;
	if ( argc > 3 && strncmp( argv[3], "--", 3 ) != 0 )
		path_to_tf_model = argv[3];

	robot::Rover_1_tf robot( env, Eigen::Vector3d( 0, 0, 0 ), path_to_tf_model );
	//robot.SetCmdPeriod( 0.1 );
	robot.SetCmdPeriod( 0.5 );
	robot.DeactivateIC();


	// [ Terrain ]

	// Orientation angle of the step:
	double orientation;
	if ( argc > 1 && strncmp( argv[1], "--", 3 ) != 0 )
	{
		char* endptr;
		orientation = strtod( argv[1], &endptr );
		if ( *endptr != '\0' )
			throw std::runtime_error( std::string( "Invalide orientation " ) + std::string( argv[1] ) );
	}
	else
	{
		// Maximum angle to be chosen randomly when not specified:
		float max_rot( 2 );
		orientation = uniform( gen )*max_rot;
	}
	float step_height( 0.105*2 );
	ode::Box step( env, Eigen::Vector3d( 1.5, 0, step_height/2 ), 1, 1, 3, step_height, false );
	step.set_rotation( 0, 0, orientation*M_PI/180 );
	step.fix();
	step.set_collision_group( "ground" );

	ode::Box step_c( env, Eigen::Vector3d( 2.5, 0, step_height/2 ), 1, 2, 3, step_height, false );
	step_c.fix();
	step_c.set_collision_group( "ground" );


	// [ Simulation rules ]

	// Cruise speed of the robot:
	float speedf( 0.15 );
	// Time to reach cruise speed:
	float term( 0.5 );
	// Duration before starting the internal control:
	float IC_start( 3.75 + 0.25*uniform( gen ) );
	// Timeout of the simulation:
	float timeout( 20 );
	// Maximum distance to travel ahead:
	//float x_goal( 2 );
	float x_goal( 1.5 );
	// Maximum lateral deviation permitted:
	float y_max( 0.5 );
	// Standard deviation of the noise on the steering rate control:
	float sr_stddev( 5 );
	// Standard deviation of the noise on the boggie torque control:
	float bt_stddev( 10 );

	float speed = 0;

	std::function<bool(float,double)>  step_function = [&]( float timestep, double time )
	{
		if ( speed <= speedf )
		{
			speed += speedf/term*timestep;
			robot.SetRobotSpeed( speed );
		}

		if ( ! robot.IsICActivated() && time >= IC_start )
			robot.ActivateIC();

		env.next_step( timestep );
		robot.next_step( timestep );

		if ( robot.ICTick() )
		{
			robot.SetSteeringRate( robot.GetSteeringRateCmd() + randn( gen )*sr_stddev );
			robot.SetBoggieTorque( robot.GetBoggieTorque() + randn( gen )*bt_stddev );
		}

		//printf( "x: %f y: %f\n", robot.GetPosition().x(), robot.GetPosition().y() );

		// If the robot has reached the goal, is out of track or has tipped over, end the simulation:
		if ( time >= timeout || fabs( robot.GetPosition().y() ) >= y_max || robot.GetPosition().x() >= x_goal || robot.IsUpsideDown() )
			return true;

		return false;
	};


	// [ Display ]

	renderer::OsgVisitor* display_ptr;

	if ( argc > 2 && ( strncmp( argv[2], "display", 8 ) == 0 || strncmp( argv[2], "capture", 8 ) == 0 ) )
	{
		// Parameters of the window:
		int x( 200 ), y( 200 ), width( 1024 ), height( 768 );
		//int x( 0 ), y( 0 ), width( 1920 ), height( 1080 );
		//display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( -0.7, -2, 0.6 ), osg::Vec3( 0, 0, -0.1 ) );
		display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( -2.2, 0, 0.5 ), osg::Vec3( 0, 0, -0.1 ) );

		display_ptr->set_window_name( "Data collection tf" );
		//display_ptr->disable_shadows();
		display_ptr->get_keh()->set_pause();

		robot.accept( *display_ptr );
		step.accept( *display_ptr );
		step_c.accept( *display_ptr );

		robot::RoverControl* keycontrol = new robot::RoverControl( &robot, display_ptr->get_viewer() );


		std::function<bool(renderer::OsgText*)> update_text = [&robot]( renderer::OsgText* text )
		{
			char buff[100];
			snprintf( buff, sizeof( buff ), "Forward speed: %5.1f cm/s\nSteering rate: %5.1f °/s\nBoggie torque: %5.1f N\u00B7m\nx: %5.2f m\ny: %5.2f m",
			          robot.GetRobotSpeed()*100, robot.GetSteeringRateCmd(), robot.GetBoggieTorque(), robot.GetPosition().x(), robot.GetPosition().y() );
			text->set_text( buff );

			return false;
		};

		renderer::OsgText::ptr_t text = display_ptr->add_text( "hud" );
		text->set_pos( 3 );
		text->set_size( 3.5 );
		text->add_background();
		text->set_callback( update_text );
	}
	else
		display_ptr = nullptr;


	// [ Simulation loop ]

	Sim_loop sim( 0.001, display_ptr, true, 0 );

	// Record screenshots of the simulation:
	if ( argc > 2 && strncmp( argv[2], "capture", 8 ) == 0 )
		sim.start_captures();

	sim.loop( step_function );


	return 0;
}
