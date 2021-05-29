#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "renderer/sim_loop.hh"
#include "renderer/osg_text.hh"
#include "ode/robot.hh"
#include "ode/cylinder.hh"
#include "ode/sphere.hh"
#include "ode/capped_cyl.hh"


#define RAD_TO_DEG 57.29577951308232
#define DEG_TO_RAD 0.017453292519943295


using namespace ode;
using namespace robot;
using namespace Eigen;


class Arm : public Robot
{
	#define N_JOINTS 4

	public:

	Arm( ode::Environment& env, const Vector3d& pose )
	{
		_main_body = Object::ptr_t( new Cylinder( env, pose + Vector3d( 0, 0, 0.1 ), 1, 0.11, 0.2 ) );
		_main_body->fix();
		_bodies.push_back( _main_body );

		Object::ptr_t base( new Sphere( env, pose + Vector3d( 0, 0, 0.2 ), 1, 0.1 ) );
		_bodies.push_back( base );

		ode::Object::ptr_t segment_1( new CappedCyl( env, pose + Vector3d( 0, 0, 0.2 + l1/2 ), m1, 0.04, l1 ) );
		_bodies.push_back( segment_1 );

		ode::Object::ptr_t segment_2( new CappedCyl( env, pose + Vector3d( 0, l2/2, 0.2 + l1 ), m2, 0.03, l2 ) );
		segment_2->set_rotation( M_PI/2, 0, 0 );
		_bodies.push_back( segment_2 );

		ode::Object::ptr_t segment_3( new CappedCyl( env, pose + Vector3d( l3/2, l2, 0.2 + l1 ), m3, 0.02, l3 ) );
		segment_3->set_rotation( 0, M_PI/2, 0 );
		_bodies.push_back( segment_3 );


		Vector3d axis_pos;

		_axes[0] = dJointCreateHinge( env.get_world(), 0 );
		dJointAttach( _axes[0], _main_body->get_body(), base->get_body() );
		dJointSetHingeAxis( _axes[0], 0, 0, 1 );
		axis_pos = base->get_pos();
		dJointSetHingeAnchor( _axes[0], axis_pos.x(), axis_pos.y(), axis_pos.z() );
		dJointSetHingeParam( _axes[0], dParamLoStop, -angle_max*DEG_TO_RAD );
		dJointSetHingeParam( _axes[0], dParamHiStop, angle_max*DEG_TO_RAD );

		_axes[1] = dJointCreateHinge( env.get_world(), 0 );
		dJointAttach( _axes[1], base->get_body(), segment_1->get_body() );
		dJointSetHingeAxis( _axes[1], 0, 1, 0 );
		axis_pos = base->get_pos();
		dJointSetHingeAnchor( _axes[1], axis_pos.x(), axis_pos.y(), axis_pos.z() );
		dJointSetHingeParam( _axes[1], dParamLoStop, -angle_max*DEG_TO_RAD );
		dJointSetHingeParam( _axes[1], dParamHiStop, angle_max*DEG_TO_RAD );

		_axes[2] = dJointCreateHinge( env.get_world(), 0 );
		dJointAttach( _axes[2], segment_1->get_body(), segment_2->get_body() );
		dJointSetHingeAxis( _axes[2], 1, 0, 0 );
		axis_pos = segment_1->get_pos() + Vector3d( 0, 0, l1/2 );
		dJointSetHingeAnchor( _axes[2], axis_pos.x(), axis_pos.y(), axis_pos.z() );
		dJointSetHingeParam( _axes[2], dParamLoStop, -angle_max*DEG_TO_RAD );
		dJointSetHingeParam( _axes[2], dParamHiStop, angle_max*DEG_TO_RAD );

		_axes[3] = dJointCreateHinge( env.get_world(), 0 );
		dJointAttach( _axes[3], segment_2->get_body(), segment_3->get_body() );
		dJointSetHingeAxis( _axes[3], 0, 0, 1 );
		axis_pos = segment_2->get_pos() + Vector3d( 0, l2/2, 0 );
		dJointSetHingeAnchor( _axes[3], axis_pos.x(), axis_pos.y(), axis_pos.z() );
		dJointSetHingeParam( _axes[3], dParamLoStop, -angle_max*DEG_TO_RAD );
		dJointSetHingeParam( _axes[3], dParamHiStop, angle_max*DEG_TO_RAD );
	}


	inline double get_axis_angle( dJointID axis ) { return dJointGetHingeAngle( axis )*RAD_TO_DEG; }


	inline double get_axis_rate( dJointID axis ) { return dJointGetHingeAngleRate( axis )*RAD_TO_DEG; }


	inline void set_axis_torque( dJointID axis, double torque ) { dJointAddHingeTorque( axis, torque ); }


	Vector3d get_effector_pos()
	{
		dVector3 pos;
		dBodyGetRelPointPos( _bodies.back()->get_body(), 0, 0, -l3/2, pos );
		Vector3d last_segment_pos( pos[0], pos[1], pos[2] );

		return last_segment_pos;
	}


	void set_new_target( Vector3d target, float velocity = 0.01 )
	{
	}


	void update_control( double delta_t )
	{
		//effector_trajectory.back()

		if ( effector_trajectory.size() > 1 )
			effector_trajectory.pop_back();
	}


	inline void set_control_period( double period ) { _control_period = period; }


	virtual void next_step( double dt = Environment::time_step )
	{
		_clock += dt;
		if ( _clock >= _control_period )
		{
			update_control( _clock );

			_clock = 0;
		}

		for ( int i = 0 ; i < N_JOINTS ; ++i )
		{
			double angle = get_axis_angle( _axes[i] );
			double rate = get_axis_rate( _axes[i] );
			double correction = -Kp*( angle - _joint_setpoints[i] ) - Kd*rate;
			set_axis_torque( _axes[i], correction );
		}
	}


	float Kp = 10;
	float Kd = 1;


	protected:

	float m1 = 5;
	float m2 = 2;
	float m3 = 2;

	float l1 = 0.6;
	float l2 = 0.5;
	float l3 = 0.4;

	float angle_max = 60;

	dJointID _axes[N_JOINTS];

	double _clock = 0;
	double _control_period = 0.1;
	double _joint_setpoints[N_JOINTS] = { 0 };

	std::vector<Vector3d> effector_trajectory;
};


int main( int argc, char* argv[] )
{
	// [ Dynamic environment ]

	dInitODE();
	ode::Environment env( 0.7 );


	// [ Robot ]

	Arm arm( env, Vector3d( 0, 0, 0 ) );
	arm.set_control_period( 0.1 );

	Vector3d target( 0.5, 0, 0.5 );
	arm.set_new_target( target, 0.01 );


	Sphere start( env, arm.get_effector_pos(), 1, 0.05 );
	start.fix();
	start.set_color( 1, 0, 0 );
	start.set_alpha( 0.3 );

	Sphere goal( env, target, 1, 0.05 );
	goal.fix();
	goal.set_color( 1, 0, 0 );
	goal.set_alpha( 0.3 );


	// [ Simulation rules ]

	std::function<bool(float,double)> step_function = [&]( float timestep, double time )
	{
		env.next_step( timestep );
		arm.next_step( timestep );

		return false;
	};


	// [ Display ]

	int x( 200 ), y( 200 ), width( 1024 ), height( 768 );
	renderer::OsgVisitor* display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( 1.2, -2, 0.6 ), osg::Vec3( 0.2, 0, 0.5 ) );

	display_ptr->set_window_name( "Arm" );
	//display_ptr->disable_shadows();
	display_ptr->get_keh()->set_pause();

	arm.accept( *display_ptr );
	start.accept( *display_ptr );
	goal.accept( *display_ptr );


	//std::function<bool(renderer::OsgText*)> update_text = [&arm]( renderer::OsgText* text )
	//{
		//char buff[100];
		//snprintf( buff, sizeof( buff ), "Error: %5.1f", 0.0 );
		//text->set_text( buff );

		//return false;
	//};

	//renderer::OsgText::ptr_t text = display_ptr->add_text( "hud" );
	//text->set_pos( 3 );
	//text->set_size( 3.5 );
	//text->add_background();
	//text->set_callback( update_text );

	
	// [ Simulation loop ]

	Sim_loop sim( 0.001, display_ptr, false, 1 );
	//sim.set_fps( 25 );

	if ( argc > 1 && strncmp( argv[1], "capture", 8 ) == 0 )
		sim.start_captures();

	sim.loop( step_function );


	return 0;
}
