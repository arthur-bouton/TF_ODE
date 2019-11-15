#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "rover.hh"
#include "ode/cylinder.hh"
#include "ode/box.hh"
#include "ode/heightfield.hh"
#include "utils/sim_loop.hh"

//#include <X11/Xlib.h>
//#include <iostream>
//#include <fstream>


int main( int argc, char* argv[] )
{
	// [ Dynamic environment ]

	dInitODE();
	ode::Environment env( 0.5 );


	// [ Robot ]

	robot::Rover_1 robot( env, Eigen::Vector3d( 0, 0, 0 ) );
	//robot.SetCmdPeriod( 0.5 );
	robot.DeactivateIC();


	// [ Obstacles ]

	float x( 1.2 );
	float y( -0.61/2 );
	float h( 0.15 );
	float l( 0.15 );
	float w( 0.30 );
	float slope( 20 );

	float l2 = h/sin( slope*M_PI/180 );
	float h2 = h/cos( slope*M_PI/180 );
	float x2 = l/2 + sqrt( l2*l2 + h2*h2 )/2 - h*tan( slope*M_PI/180 );

	ode::Box ramp_part1( env, Eigen::Vector3d( x, y, 0 ), 1, l, w, h*2, false );

	ode::Box ramp_part2( env, Eigen::Vector3d( x + x2, y, 0 ), 1, l2, w, h2, false );
	ramp_part2.set_rotation( 0, -slope*M_PI/180, 0 );

	ode::Box ramp_part3( env, Eigen::Vector3d( x - x2, y, 0 ), 1, l2, w, h2, false );
	ramp_part3.set_rotation( 0, slope*M_PI/180, 0 );

	ramp_part1.fix();
	ramp_part2.fix();
	ramp_part3.fix();
	ramp_part1.set_collision_group( "ground" );
	ramp_part2.set_collision_group( "ground" );
	ramp_part3.set_collision_group( "ground" );


	// [ Simulation rules ]

	// Cruise speed of the robot:
	float speedf( 0.15 );
	// Time to reach cruise speed:
	float term( 0.5 );
	// Maximum distance to travel ahead:
	float x_goal( 2.5 );
	// Period with which to print the rover state:
	float print_period( 0.1 );

	float speed = 0;
	float print_clock = 0;

	std::function<bool(float,double)>  step_function = [&]( float timestep, double time )
	{
		if ( speed <= speedf )
		{
			speed += speedf/term*timestep;
			robot.SetRobotSpeed( speed );
		}

		env.next_step( timestep );
		robot.next_step( timestep );

		print_clock += timestep;
		if ( print_clock >= print_period )
		{
			printf( "%f %f %f ", time, robot.GetRollAngle(), robot.GetPitchAngle() );
			robot.PrintForkTorsors();

			print_clock = 0;
		}

		//printf( "x: %f y: %f\n", robot.GetPosition().x(), robot.GetPosition().y() );
		if ( robot.GetPosition().x() >= x_goal )
			return true;

		return false;
	};


	// [ Display ]

	renderer::OsgVisitor* display_ptr;

	if ( argc > 1 && strncmp( argv[1], "nodisplay", 10 ) == 0 )
		display_ptr = nullptr;
	else
	{
		int x( 200 ), y( 200 ), width( 1024 ), height( 768 );
		display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( -0.7, -2, 0.6 ), osg::Vec3( 0, 0, -0.1 ) );

		display_ptr->set_window_name( "Ramp" );
		//display_ptr->disable_shadows();
		display_ptr->set_pause();

		robot.accept( *display_ptr );
		ramp_part1.accept( *display_ptr );
		ramp_part2.accept( *display_ptr );
		ramp_part3.accept( *display_ptr );

		robot::RoverControl* keycontrol = new robot::RoverControl( &robot, display_ptr->get_viewer() );
	}

	
	// [ Simulation loop ]

	Sim_loop sim( 0.001, display_ptr, 1 );
	//sim.set_fps( 25 );

	if ( argc > 1 && strncmp( argv[1], "capture", 8 ) == 0 )
		sim.start_captures();

	sim.loop( step_function );


	return 0;
}
