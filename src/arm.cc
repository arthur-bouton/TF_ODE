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
	public:

	Arm( ode::Environment& env, const Vector3d& pose = Vector3d( 0, 0, 0 ) )
	{
		_main_body = Object::ptr_t( new Cylinder( env, pose + Vector3d( 0, 0, 0.1 ), 1, 0.11, 0.2 ) );
		_main_body->fix();
		_bodies.push_back( _main_body );
		_base_pos = pose + Vector3d( 0, 0, 0.2 );


		segment new_segment;

		new_segment.direction = Vector3d( 0, 0, 1 );
		new_segment.axis = Vector3d( 0, 1, 0 );
		new_segment.radius = 0.1;
		new_segment.length = 0;
		new_segment.mass = 2;
		new_segment.Kp = 20;
		new_segment.Kd = 0.1;
		_segments.push_back( new_segment );

		new_segment.direction = Vector3d( -1, 0, 1 );
		new_segment.axis = Vector3d( 0, 1, 0 );
		new_segment.radius = 0.04;
		new_segment.length = 0.5;
		new_segment.mass = 2;
		new_segment.Kp = 20;
		new_segment.Kd = 0.1;
		_segments.push_back( new_segment );

		new_segment.direction = Vector3d( 1, 0, 1 );
		new_segment.axis = Vector3d( 0, 1, 0 );
		new_segment.radius = 0.03;
		new_segment.length = 0.5;
		new_segment.mass = 1;
		new_segment.Kp = 20;
		new_segment.Kd = 0.2;
		_segments.push_back( new_segment );

		new_segment.direction = Vector3d( 1, 0, -1 );
		new_segment.axis = Vector3d( 0, 1, 0 );
		new_segment.radius = 0.02;
		new_segment.length = 0.4;
		new_segment.mass = 1;
		new_segment.Kp = 10;
		new_segment.Kd = 0.01;
		_segments.push_back( new_segment );

		new_segment.direction = Vector3d( 1, 0, -1 );
		new_segment.axis = Vector3d( 1, 0, -1 );
		new_segment.radius = 0.025;
		new_segment.length = 0.05;
		new_segment.mass = 0.5;
		new_segment.Kp = 10;
		new_segment.Kd = 0.01;
		_segments.push_back( new_segment );

		new_segment.direction = Vector3d( 1, 0, 0 );
		new_segment.axis = Vector3d( 0, 1, 0 );
		new_segment.radius = 0.01;
		new_segment.length = 0.15;
		new_segment.mass = 0.5;
		new_segment.Kp = 10;
		new_segment.Kd = 0.01;
		_segments.push_back( new_segment );


		Object::ptr_t prev_body = _main_body;
		Vector3d prev_end_pos = _base_pos;

		for ( segment& seg : _segments )
		{
			seg.direction.normalize();
			Object::ptr_t new_body( new CappedCyl( env, pose + prev_end_pos + seg.direction*seg.length/2,
			                                       seg.mass, seg.radius, seg.length ) );
			Vector3d y = Vector3d( 0, 0, 1 ).cross( seg.direction );
			if ( y == Vector3d::Zero() ) y = Vector3d( 0, 1, 0 );
			Vector3d x = y.cross( seg.direction );
			if ( x == Vector3d::Zero() ) x = Vector3d( 1, 0, 0 );
			new_body->set_rotation( x, y );
			_bodies.push_back( new_body );

			seg.axis.normalize();
			seg.jointID = dJointCreateHinge( env.get_world(), 0 );
			dJointAttach( seg.jointID, prev_body->get_body(), new_body->get_body() );
			dJointSetHingeAxis( seg.jointID, seg.axis[0], seg.axis[1], seg.axis[2] );
			dJointSetHingeAnchor( seg.jointID, prev_end_pos.x(), prev_end_pos.y(), prev_end_pos.z() );
			dJointSetHingeParam( seg.jointID, dParamLoStop, -seg.angle_max*DEG_TO_RAD );
			dJointSetHingeParam( seg.jointID, dParamHiStop, seg.angle_max*DEG_TO_RAD );

			prev_body = new_body;
			prev_end_pos = prev_end_pos + seg.direction*seg.length;
		}
	}


	Vector3d get_effector_pos()
	{
		Vector3d pos = _base_pos;

		Matrix3d rot = Eigen::Matrix3d::Identity();

		for ( segment seg : _segments )
		{
			rot = AngleAxisd( get_axis_angle( seg ), seg.axis ).toRotationMatrix()*rot;
			pos += rot*seg.direction*seg.length;
		}

		return pos;
	}


	void set_new_target( Vector3d target, float velocity = 0.01 )
	{
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

		for ( segment seg : _segments )
		{
			double angle = get_axis_angle( seg );
			double rate = get_axis_rate( seg );
			double correction = -seg.Kp*( angle - seg.angle_setpoint ) - seg.Kd*rate;
			set_axis_torque( seg, correction );
		}
	}


	protected:

	Vector3d _base_pos;

	typedef struct segment
	{
		float mass, radius, length;
		Vector3d direction, axis;
		dJointID jointID;
		float angle_max = 60;
		double angle_setpoint = 0;
		float Kp = 10;
		float Kd = 0.1;
	} segment;

	std::vector<segment> _segments;


	inline double get_axis_angle( segment seg ) { return dJointGetHingeAngle( seg.jointID )*RAD_TO_DEG; }

	inline double get_axis_rate( segment seg ) { return dJointGetHingeAngleRate( seg.jointID )*RAD_TO_DEG; }

	inline void set_axis_torque( segment seg, double torque ) { dJointAddHingeTorque( seg.jointID, torque ); }


	void update_control( double delta_t )
	{
		//_effector_trajectory.back()

		if ( _effector_trajectory.size() > 1 )
			_effector_trajectory.pop_back();
	}


	double _clock = 0;
	double _control_period = 0.1;

	std::vector<Vector3d> _effector_trajectory;
};


int main( int argc, char* argv[] )
{
	// [ Dynamic environment ]

	dInitODE();
	ode::Environment env( 0.7 );


	// [ Robot ]

	Arm arm( env );
	arm.set_control_period( 0.1 );

	Vector3d target( 0.5, -0.2, 0.4 );
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
	renderer::OsgVisitor* display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( 2, -2, 0.5 ), osg::Vec3( 0.2, 0, 0.4 ) );

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
