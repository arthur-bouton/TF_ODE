/* 
** Program to be compiled both as a module to be called by the python script
** in charge of the training of a robot controlled by a TensorFlow model
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
*/

#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "quadruped.hh"
#include "ode/box.hh"
#include "renderer/sim_loop.hh"
#include "renderer/osg_text.hh"
#include <boost/python.hpp>
#include <csignal>


#define DEFAULT_PATH_TO_MODEL_DIR "../training_data/Rt05/actor"


namespace p = boost::python;


p::list simulation( const char* option = "", const char* path_to_model_dir = DEFAULT_PATH_TO_MODEL_DIR, int argc = 0, char* argv[] = nullptr )
{
	// [ Dynamic environment ]

	dInitODE();
	// Set the global friction coefficient:
	ode::Environment env( 0.9 );


	// [ Robot ]

	robot::Quadruped robot( env, Eigen::Vector3d( 0, 0, 0 ), path_to_model_dir );
	robot.SetCmdPeriod( 0.5 );
	robot.DeactivateIC();
	if ( strncmp( option, "trial", 6 ) == 0 || strncmp( option, "explore", 8 ) == 0 )
		robot.SetExploration( true );


	// [ Simulation rules ]

	// Timeout of the simulation:
	float timeout( 60 );
	// Maximum distance to travel ahead:
	float x_goal( 1 );
	// Maximum lateral deviation permitted:
	float y_max( 0.5 );
	// Duration before starting the internal control:
	float IC_start( 0.5 );

	std::function<bool(float,double)> step_function = [&]( float timestep, double time )
	{
		if ( ! robot.IsICActivated() && time >= IC_start )
			robot.ActivateIC();

		env.next_step( timestep );
		robot.next_step( timestep );

		// If the robot has reached the goal, is out of track or has tipped over, end the simulation:
		if ( time >= timeout || fabs( robot.GetPosition().y() ) >= y_max || fabs( robot.GetPosition().x() ) >= x_goal || robot.IsUpsideDown() )
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

		display_ptr->set_window_name( "Quadruped training" );
		//display_ptr->disable_shadows();
		display_ptr->get_keh()->set_pause();

		robot.accept( *display_ptr );


		std::function<bool(renderer::OsgText*)> update_text = [&robot]( renderer::OsgText* text )
		{
			char buff[100];
			snprintf( buff, sizeof( buff ), "x: %5.2f m\ny: %5.2f m",
			          robot.GetPosition().x(), robot.GetPosition().y() );
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
		( fabs( robot.GetPosition().x() ) >= x_goal ? "\033[1;32m[Success]\033[0;39m" : "\033[1;31m[Failure]\033[0;39m" ),
		sim.get_time(), robot.GetPosition().x(), robot.GetPosition().y(), robot.GetTotalReward()/sim.get_time() );
		fflush( stdout );
	}


	// Fetch the stored experience from the trial:
	p::list experience = robot.GetExperience();

	// Penalise if the robot has tipped over:
	if ( robot.IsUpsideDown() )
		experience[-1] = p::make_tuple( experience[-1][0], experience[-1][1], -1, true, experience[-1][4] );
	// Penalise if the robot has gone too far sideway:
	//else if ( fabs( robot.GetPosition().y() ) >= y_max )
		//experience[-1] = p::make_tuple( experience[-1][0], experience[-1][1], -1, true, experience[-1][4] );
	else
		experience[-1] = p::make_tuple( experience[-1][0], experience[-1][1], experience[-1][2], true, experience[-1][4] );

	return experience;
}


int main( int argc, char* argv[] )
{
	Py_Initialize();
	signal( SIGINT, SIG_DFL );

	const char* path_to_model_dir = DEFAULT_PATH_TO_MODEL_DIR;
	if ( argc > 2 && strncmp( argv[2], "--", 3 ) != 0 )
		path_to_model_dir = argv[2];

	simulation( argc > 1 ? argv[1] : "display", path_to_model_dir, argc, argv );

	return 0;
}


p::list trial( const char* path_to_model_dir )
{
	return simulation( "trial", path_to_model_dir );
}


void eval( const char* path_to_model_dir )
{
	simulation( "eval", path_to_model_dir );
}


BOOST_PYTHON_MODULE( quadruped_training_module )
{
	signal( SIGINT, SIG_DFL );

    p::def( "trial", trial );
    p::def( "eval", eval );
}
