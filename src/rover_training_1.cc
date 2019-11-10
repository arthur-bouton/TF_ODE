#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "rover_tf.hh"
#include "ode/cylinder.hh"
#include "ode/box.hh"
#include "ode/heightfield.hh"
#include "utils/sim_loop.hh"
#include <boost/python.hpp>

//#include <X11/Xlib.h>
//#include <iostream>
//#include <fstream>


#define DEFAULT_PATH_TO_TF_MODEL "../training_data/rover_training_1"


namespace p = boost::python;


p::list simulation( const char* option = "", const char* path_to_tf_model = DEFAULT_PATH_TO_TF_MODEL )
{
	// [ Dynamic environment ]

	dInitODE();
	//ode::Environment env( 0.7 );
	//ode::Environment env( 0.5 );
	ode::Environment env( false, 0.5 );


	// [ Robot ]

	robot::Rover_1_tf robot( env, Eigen::Vector3d( 0, 0, 0 ), path_to_tf_model );
	robot.SetCmdPeriod( 0.5 );
	#ifdef EXE
		robot.SetCmdPeriod( 0.1 );
	#endif
	robot.DeactivateIC();
	if ( strncmp( option, "trial", 6 ) == 0 || strncmp( option, "explore", 8 ) == 0 )
		robot.SetExploration( true );


	// [ Obstacles ]

	//ode::Box obs1( env, Eigen::Vector3d( 0.7, -0.5, 0.1/2 ), 1, 0.1, 1, 0.1, false );
	//obs1.fix();
	//obs1.set_collision_group( "ground" );

	//ode::Box obs2( env, Eigen::Vector3d( 1.5, 0, 0.3/2 ), 1, 1, 2, 0.3, false );
	//ode::Box obs2( env, Eigen::Vector3d( 2, 0, 0.3/2 ), 1, 2, 3, 0.3, false );
	//obs2.fix();
	//obs2.set_collision_group( "ground" );

	//osg::Image* heightimage = osgDB::readImageFile( "../env_data/heightmap_rock_step_4.png" );
	//osg::Image* heightimage = osgDB::readImageFile( "../env_data/heightmap_rock_groove.png" );
	osg::Image* heightimage = osgDB::readImageFile( "../env_data/heightmap_rock_groove_large.png" );
	int nrow = heightimage->t();
	int ncol = heightimage->s();
	double* heightmap = (double*) calloc( nrow*ncol, sizeof( double ) );
	for ( int r = 0 ; r < nrow ; r++ )
		for ( int c = 0 ; c < ncol ; c++ )
			heightmap[(nrow-r-1)*ncol+c] = ( *heightimage->data( c, r ) )*0.3/255;
	//ode::HeightField field( env, Eigen::Vector3d( 2, 0, -0.01 ), heightmap, nrow, ncol, 3, 3, 0, -1, 1 );
	ode::HeightField field( env, Eigen::Vector3d( 1, 0, -0.3 ), heightmap, nrow, ncol, 6, 6, 0, -1, 1 );
	field.set_collision_group( "ground" );


	// [ Simulation rules ]

	// Cruise speed of the robot:
	float speedf( 0.15 );
	// Time to reach cruise speed:
	float term( 0.5 );
	// Duration before starting the internal control:
	float IC_start( 1 );
	// Timeout of the simulation:
	float timeout( 60 );
	// Maximum distance to travel ahead:
	//float x_goal( 1.5 );
	//float x_goal( 2 );
	float x_goal( 2.5 );
	// Maximum lateral deviation permitted:
	float y_max( 0.5 );

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

		//printf( "x: %f y: %f\n", robot.GetPosition().x(), robot.GetPosition().y() );
		if ( time >= timeout || fabs( robot.GetPosition().y() ) >= y_max || robot.GetPosition().x() >= x_goal || robot.IsUpsideDown() )
			return true;

		return false;
	};


	// [ Display ]

	renderer::OsgVisitor* display_ptr;

	if ( strncmp( option, "display", 8 ) == 0 || strncmp( option, "capture", 8 ) == 0 || strncmp( option, "explore", 8 ) == 0 )
	{
		//int x( 200 ), y( 200 ), width( 1024 ), height( 768 );
		int x( 200 ), y( 200 ), width( 1920 ), height( 1080 );
		//display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( -0.7, -2, 0.6 ), osg::Vec3( 0, 0, -0.1 ) );
		//display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 0, 0, osg::Vec3( -0.7, -2, 0.6 ), osg::Vec3( 0, 0, -0.1 ) );
		//display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( -2.2, -2.4, 1.4 ), osg::Vec3( -0.2, 0, -0.6 ) );

		// Point of view for the rock step:
		//display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( -2.2, -1.4, 0.5 ), osg::Vec3( -0.2, 0, -0.4 ) );
		// Point of view for the groove:
		display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( 0, -2.2, 0.4 ), osg::Vec3( -0.15, 0, -0.35 ) );
		//display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( 0, 2.2, 0.4 ), osg::Vec3( -0.15, 0, -0.35 ),
		                                        //renderer::OsgVisitor::TRACK, true, osg::Vec3( 1, 2, 3 ) );

		display_ptr->set_window_name( "Rover training 1" );
		//display_ptr->disable_shadows();
		display_ptr->set_pause();

		robot.accept( *display_ptr );
		//obs1.accept( *display_ptr );
		//obs2.accept( *display_ptr );
		field.accept( *display_ptr );

		robot::RoverControl* keycontrol = new robot::RoverControl( &robot, display_ptr->get_viewer() );
	}
	else
		display_ptr = nullptr;


	// [ Simulation loop ]

	Sim_loop sim( 0.001, display_ptr, 0 );

	if ( strncmp( option, "capture", 8 ) == 0 )
		sim.start_captures();

	sim.loop( step_function );


	if ( strncmp( option, "trial", 6 ) != 0 )
	{
		printf( "%s t %6.3f | x %5.3f | y %+6.3f | Rmoy %7.3f\n",
		( robot.GetPosition().x() >= x_goal ? "\033[1;32m[Success]\033[0;39m" : "\033[1;31m[Failure]\033[0;39m" ),
		sim.get_time(), robot.GetPosition().x(), robot.GetPosition().y(), robot.GetTotalReward()/sim.get_time() );
		fflush( stdout );
	}


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

	simulation( argc > 1 ? argv[1] : "display", argc > 2 ? argv[2] : DEFAULT_PATH_TO_TF_MODEL );

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
