/* 
** Program to be compiled both as a module to be called by the python script
** in charge of the training of a rover controlled by a TensorFlow model
** and a standalone executable to evaluate the learned policy.
**
** First arguments accepted (optional):
** display: Create a window with a graphical rendering of the simulation (default).
** capture: Record screenshots of the simulation (in /tmp and at 25 fps by default).
** explore: Enable the exploration together with the graphical rendering.
** trial:   Do a training trial with exploration and no rendering.
** eval:    Evaluate the policy without rendering.
**
** Second argument (optional):
** path to the TensorFlow model to be used.
**
** Third argument (optional):
** Orientation of the step.
*/

#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "rover_tf.hh"
#include "ode/box.hh"
#include "ode/heightfield.hh"
#include "renderer/sim_loop.hh"
#include "renderer/osg_text.hh"
#include <boost/python.hpp>
#include <random>


#define DEFAULT_PATH_TO_TF_MODEL "../training_data/step_06_PER_1_no_sym/selected/rover_training_1_0005"


namespace p = boost::python;


p::list simulation( const char* option = "", const char* path_to_tf_model = DEFAULT_PATH_TO_TF_MODEL, int argc = 0, char* argv[] = nullptr )
{
	// Uniform random generator:
	std::random_device rd;
	std::mt19937 gen( rd() );
	std::uniform_real_distribution<double> uniform( -1, 1 );


	// [ Dynamic environment ]

	dInitODE();
	// Set the global friction coefficient:
	ode::Environment env( 0.5 );


	// [ Robot ]

	robot::Rover_1_tf robot( env, Eigen::Vector3d( 0, 0, 0 ), path_to_tf_model );
	robot.SetCrawlingMode( true );
	robot.SetCmdPeriod( 0.1 );
//#ifdef EXE
	//robot.SetCmdPeriod( 0.1 );
//#endif
	robot.DeactivateIC();
	if ( strncmp( option, "trial", 6 ) == 0 || strncmp( option, "explore", 8 ) == 0 )
		robot.SetExploration( true );


	// [ Terrain ]

	// Orientation angle of the step:
	double orientation;
	if ( strncmp( option, "eval", 5 ) == 0 )
		orientation = 0;
	else if ( argc > 3 )
	{
		char* endptr;
		orientation = strtod( argv[3], &endptr );
		if ( *endptr != '\0' )
			throw std::runtime_error( std::string( "Invalide argument " ) + std::string( argv[3] ) );
	}
	else
	{
		// Maximum angle to be chosen randomly when not specified:
		float max_rot( 5 );
		orientation = max_rot*uniform( gen );
	}
	float step_height( 0.105*2 );
	ode::Box step( env, Eigen::Vector3d( -1.25, 0, step_height/2 ), 1, 1, 3, step_height, false );
	step.set_rotation( 0, 0, orientation*M_PI/180 );
	step.fix();
	step.set_collision_group( "ground" );

	ode::Box step_c( env, Eigen::Vector3d( -2.25, 0, step_height/2 ), 1, 2, 3, step_height, false );
	step_c.fix();
	step_c.set_collision_group( "ground" );


	// [ Simulation rules ]

	// Cruise speed of the robot:
	float speedf( -0.10 );
	// Time to reach cruise speed:
	float term( 0.5 );
	// Duration before starting the internal control:
	float IC_start( 1 );
	if ( strncmp( option, "trial", 6 ) == 0 )
		IC_start += 0.05*uniform( gen );
	// Timeout of the simulation:
	float timeout( 60 );
	// Maximum distance to travel ahead:
	float x_goal( -1.5 );
	// Maximum lateral deviation permitted:
	float y_max( 0.6 );

	float speed = 0;

	std::function<bool(float,double)>  step_function = [&]( float timestep, double time )
	{
		if ( speed >= speedf )
		{
			speed += speedf/term*timestep;
			robot.SetRobotSpeed( speed );
		}

		if ( ! robot.IsICActivated() && time >= IC_start )
			robot.ActivateIC();

		env.next_step( timestep );
		robot.next_step( timestep );

		// If the robot has reached the goal, is out of track or has tipped over, end the simulation:
		if ( time >= timeout || fabs( robot.GetPosition().y() ) >= y_max || robot.GetPosition().x() <= x_goal || robot.IsUpsideDown() )
			return true;

		return false;
	};


	// [ Display ]

	renderer::OsgVisitor* display_ptr;

	if ( strncmp( option, "display", 8 ) == 0 || strncmp( option, "capture", 8 ) == 0 || strncmp( option, "explore", 8 ) == 0 )
	{
		// Parameters of the window:
		int x( 200 ), y( 200 ), width( 1024 ), height( 768 );
		//int x( 0 ), y( 0 ), width( 1920 ), height( 1080 );
		display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( -0.7, -2, 0.6 ), osg::Vec3( 0, 0, -0.1 ) );

		display_ptr->set_window_name( "Rover training 1" );
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
	if ( strncmp( option, "capture", 8 ) == 0 )
		sim.start_captures();

	sim.loop( step_function );


	// Print the result of the trial:
	if ( strncmp( option, "trial", 6 ) != 0 )
	{
		printf( "%s t %6.3f | x %5.3f | y %+6.3f | Rmoy %7.3f\n",
		( robot.GetPosition().x() <= x_goal ? "\033[1;32m[Success]\033[0;39m" : "\033[1;31m[Failure]\033[0;39m" ),
		sim.get_time(), robot.GetPosition().x(), robot.GetPosition().y(), robot.GetTotalReward()/sim.get_time() );
		fflush( stdout );
	}


	// Fetch the stored experience from the trial:
	p::list experience = robot.GetExperience();

	// Penalise if the rover has gone too far sideway:
	if ( fabs( robot.GetPosition().y() ) >= y_max )
		experience[-1] = p::make_tuple( experience[-1][0], experience[-1][1], experience[-1][2] - 2, true, experience[-1][4] );
	// Penalise if the rover has tipped over:
	else if ( robot.IsUpsideDown() )
		experience[-1] = p::make_tuple( experience[-1][0], experience[-1][1], experience[-1][2] - 5, true, experience[-1][4] );
	else
		experience[-1] = p::make_tuple( experience[-1][0], experience[-1][1], experience[-1][2], true, experience[-1][4] );

	return experience;
}


int main( int argc, char* argv[] )
{
	Py_Initialize();

	const char* path_to_tf_model = DEFAULT_PATH_TO_TF_MODEL;
	if ( argc > 2 && strncmp( argv[2], "--", 3 ) != 0 )
		path_to_tf_model = argv[2];

	simulation( argc > 1 ? argv[1] : "display", path_to_tf_model, argc, argv );

	return 0;
}


p::list trial( const char* path_to_tf_model )
{
	return simulation( "trial", path_to_tf_model );
}


void eval( const char* path_to_tf_model )
{
	simulation( "eval", path_to_tf_model );
}


BOOST_PYTHON_MODULE( rover_training_1_module )
{
    p::def( "trial", trial );
    p::def( "eval", eval );
}
