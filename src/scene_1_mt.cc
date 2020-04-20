#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "rover_mt.hh"
#include "ode/box.hh"
#include "ode/heightfield.hh"
#include "renderer/sim_loop.hh"
#include "renderer/osg_text.hh"


#define YAML_FILE_PATH "../scripts/tree_params2_"


int main( int argc, char* argv[] )
{
	// [ Dynamic environment ]

	dInitODE();
	ode::Environment env( 0.6 );
	//ode::Environment env( 0.7 );
	//ode::Environment env( false, 0.5 );
	//ode::Environment env( false, 0.7 );


	// [ Robot ]

	const char* yaml_file_path = YAML_FILE_PATH;
	if ( argc > 3 )
		yaml_file_path = argv[3];
	std::string yaml_file_path_1 = std::string( yaml_file_path ) + std::string( "1.yaml" );
	std::string yaml_file_path_2 = std::string( yaml_file_path ) + std::string( "2.yaml" );
	robot::Rover_1_mt robot( env, Eigen::Vector3d( 0, 0, 0 ), yaml_file_path_1, yaml_file_path_2, false, 1 );
	//robot::Rover_1_mt robot( env, Eigen::Vector3d( 0, 0, 0 ), yaml_file_path_1, yaml_file_path_2, false, 2, true );
	//robot.SetCmdPeriod( 0.1 );
	robot.SetCmdPeriod( 0.5 );


	// [ Terrain ]

	double orientation( 0 );
	if ( argc > 2 )
	{
		char* endptr;
		orientation = strtod( argv[2], &endptr );
		if ( *endptr != '\0' )
			throw std::runtime_error( std::string( "Invalide orientation: " ) + std::string( argv[2] ) );
	}
	float step_height( 0.105*2 );
	ode::Box step( env, Eigen::Vector3d( 1.5, 0, step_height/2 ), 1, 1, 3, step_height, false );
	step.set_rotation( 0, 0, orientation*M_PI/180 );
	step.fix();
	step.set_collision_group( "ground" );

	ode::Box step_c( env, Eigen::Vector3d( 2.5, 0, step_height/2 ), 1, 2, 3, step_height, false );
	step_c.fix();
	step_c.set_collision_group( "ground" );


	//ode::Box side_obstacle( env, Eigen::Vector3d( 1, -0.61/2, 0.21/2 ), 1, 0.2, 0.5, 0.21, false );
	//side_obstacle.fix();
	//side_obstacle.set_collision_group( "ground" );


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
	std::vector<double> prev_state;

	std::function<bool(float,double)> step_function = [&]( float timestep, double time )
	{
		if ( speed <= speedf )
		{
			speed += speedf/term*timestep;
			robot.SetRobotSpeed( speed );
		}

		env.next_step( timestep );
		robot.next_step( timestep );

		if ( robot.ICTick() )
		{
			std::vector<double> current_state = robot.GetFullState();

			if ( prev_state.size() > 0 )
			{
				// Print the transitions:
				for ( auto val : prev_state )
					printf( "%f ", val );
				printf( "%f %f", robot.GetSteeringRateCmd(), robot.GetBoggieTorque() );
				for ( auto val : current_state )
					printf( " %f", val );
				printf( "\n" );
				fflush( stdout );
			}

			prev_state = current_state;
		}

		//if ( time >= timeout || fabs( robot.GetPosition().y() ) >= y_max || robot.GetPosition().x() >= x_goal || robot.IsUpsideDown() )
			//return true;

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
		display_ptr->get_keh()->set_pause();

		robot.accept( *display_ptr );
		step.accept( *display_ptr );
		step_c.accept( *display_ptr );
		//side_obstacle.accept( *display_ptr );
		//field.accept( *display_ptr );

		//robot::RoverControl* keycontrol = new robot::RoverControl( &robot, display_ptr->get_viewer() );


		std::function<bool(renderer::OsgText*)> update_text = [&robot]( renderer::OsgText* text )
		{
			char buff[200];
			snprintf( buff, sizeof( buff ), "Forward speed: %5.1f cm/s\nSteering rate: %5.1f °/s\nBoggie torque: %5.1f N\u00B7m\nx: %5.2f m    Node 1: %2d\ny: %5.2f m    Node 2: %2d",
			          robot.GetRobotSpeed()*100, robot.GetSteeringRateCmd(), robot.GetBoggieTorque(), robot.GetPosition().x(), robot.node_1, robot.GetPosition().y(), robot.node_2 );
			text->set_text( buff );

			return false;
		};

		renderer::OsgText::ptr_t text = display_ptr->add_text( "hud" );
		text->set_pos( 3 );
		text->set_size( 3.5 );
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
