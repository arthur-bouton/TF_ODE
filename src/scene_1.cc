#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "rover.hh"
#include "ode/box.hh"
#include "ode/heightfield.hh"
#include "renderer/sim_loop.hh"
#include "renderer/osg_text.hh"

//#include <X11/Xlib.h>
//#include <iostream>
//#include <fstream>


int main( int argc, char* argv[] )
{
	// [ Dynamic environment ]

	dInitODE();
	//ode::Environment env( 0.7 );
	ode::Environment env( 0.5 );
	//ode::Environment env( false, 0.5 );
	//ode::Environment env( false, 0.7 );


	// [ Robot ]

	robot::Rover_1 robot( env, Eigen::Vector3d( 0, 0, 0 ) );
	//robot.SetCmdPeriod( 0.5 );
	robot.DeactivateIC();


	// [ Terrain ]

	double rot( 0 );
	float step_height( 0.105*2 );
	ode::Box step( env, Eigen::Vector3d( 1.5, 0, step_height/2 ), 1, 1, 3, step_height, false );
	step.set_rotation( 0, 0, rot*M_PI/180 );
	step.fix();
	step.set_collision_group( "ground" );

	ode::Box step_c( env, Eigen::Vector3d( 2.5, 0, step_height/2 ), 1, 2, 3, step_height, false );
	step_c.fix();
	step_c.set_collision_group( "ground" );

	//ode::HeightField field( env, Eigen::Vector3d( 2, 0, -0.01 ), "../env_data/heightmap_rock_step.png", 0.3, 3, 3, 0, -1, 1 );
	//ode::HeightField field( env, Eigen::Vector3d( 1, 0, -0.3 ), "../env_data/heightmap_rock_groove.png", 0.3, 3, 3, 0, -1, 1 );
	//field.set_collision_group( "ground" );


	// [ Simulation rules ]

	// Cruise speed of the robot:
	float speedf( 0.15 );
	// Time to reach cruise speed:
	float term( 0.5 );
	// Timeout of the simulation:
	float timeout( 60 );
	// Maximum distance to travel ahead:
	float x_goal( 2 );
	// Maximum lateral deviation permitted:
	float y_max( 0.5 );

	float speed = 0;

	std::function<bool(float,double)> step_function = [&]( float timestep, double time )
	{
		if ( speed <= speedf )
		{
			speed += speedf/term*timestep;
			robot.SetRobotSpeed( speed );
		}

		env.next_step( timestep );
		robot.next_step( timestep );

		if ( time >= timeout || fabs( robot.GetPosition().y() ) >= y_max || robot.GetPosition().x() >= x_goal || robot.IsUpsideDown() )
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
		//display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 0, 0, osg::Vec3( -0.7, -2, 0.6 ), osg::Vec3( 0, 0, -0.1 ) );
		//display_ptr->set_ground_texture( "../env_data/mars_checker.tga" );
		//display_ptr->set_background_color( 179, 71, 0 );

		display_ptr->set_window_name( "Scene 1" );
		//display_ptr->disable_shadows();
		display_ptr->set_pause();

		robot.accept( *display_ptr );
		step.accept( *display_ptr );
		step_c.accept( *display_ptr );
		//field.accept( *display_ptr );

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
		text->set_pos( 30, 30 );
		text->set_size( 28 );
		text->add_background();
		text->set_callback( update_text );
	}

	
	// [ Simulation loop ]

	Sim_loop sim( 0.001, display_ptr, false, 1 );
	//sim.set_fps( 25 );

	if ( argc > 1 && strncmp( argv[1], "capture", 8 ) == 0 )
		sim.start_captures();

	sim.loop( step_function );


	return 0;
}
