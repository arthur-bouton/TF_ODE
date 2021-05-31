#include "ode/environment.hh"
#include "renderer/osg_visitor.hh"
#include "renderer/sim_loop.hh"
#include "renderer/osg_text.hh"
#include "ode/robot.hh"
#include "ode/cylinder.hh"
#include "ode/sphere.hh"
#include "ode/capped_cyl.hh"
#include <deque>


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
		new_segment.axis = Vector3d( 0, 0, 1 );
		new_segment.radius = 0.1;
		new_segment.length = 0;
		new_segment.mass = 2;
		new_segment.Kp = 20;
		new_segment.Kd = 1;
		_segments.push_back( new_segment );

		new_segment.direction = Vector3d( -1, 0, 1 );
		new_segment.axis = Vector3d( 0, 1, 0 );
		new_segment.radius = 0.04;
		new_segment.length = 0.5;
		new_segment.mass = 2;
		new_segment.Kp = 20;
		new_segment.Kd = 1;
		_segments.push_back( new_segment );

		new_segment.direction = Vector3d( 1, 0, 1 );
		new_segment.axis = Vector3d( 0, 1, 0 );
		new_segment.radius = 0.03;
		new_segment.length = 0.5;
		new_segment.mass = 1;
		new_segment.Kp = 20;
		new_segment.Kd = 1;
		_segments.push_back( new_segment );

		new_segment.direction = Vector3d( 1, 0, -1 );
		new_segment.axis = Vector3d( 0, 1, 0 );
		new_segment.radius = 0.02;
		new_segment.length = 0.4;
		new_segment.mass = 1;
		new_segment.Kp = 20;
		new_segment.Kd = 1;
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
			dJointAttach( seg.jointID, new_body->get_body(), prev_body->get_body() );
			dJointSetHingeAxis( seg.jointID, seg.axis[0], seg.axis[1], seg.axis[2] );
			dJointSetHingeAnchor( seg.jointID, prev_end_pos.x(), prev_end_pos.y(), prev_end_pos.z() );
			dJointSetHingeParam( seg.jointID, dParamLoStop, -seg.angle_max*DEG_TO_RAD );
			dJointSetHingeParam( seg.jointID, dParamHiStop, seg.angle_max*DEG_TO_RAD );

			prev_body = new_body;
			prev_end_pos = prev_end_pos + seg.direction*seg.length;
		}
	}


	Vector3d get_effector_pos( bool real = false ) const
	{
		Vector3d pos = _base_pos;

		Matrix3d rot = Eigen::Matrix3d::Identity();

		for ( segment seg : _segments )
		{
			double angle = ( real ? _get_axis_angle( seg ) : seg.angle_setpoint );
			rot = AngleAxisd( angle*DEG_TO_RAD, rot*seg.axis ).toRotationMatrix()*rot;
			pos += rot*seg.direction*seg.length;
		}

		return pos;
	}


	void set_new_target( Vector3d target, float velocity = 0.01, bool add = true )
	{
		Vector3d start;
		if ( add && !_effector_trajectory.empty() )
			start = _effector_trajectory.back();
		else
		{
			start = get_effector_pos( true );
			_effector_trajectory.clear();
			_effector_trajectory.push_back( start );
		}

		Vector3d step_vector = ( target - start ).normalized()*velocity*_control_period;
		int n_timesteps = ( target - start ).norm()/( velocity*_control_period );

		for ( int i = 0 ; i < n_timesteps ; ++i )
			_effector_trajectory.push_back( _effector_trajectory.back() + step_vector );
		_effector_trajectory.push_back( target );
	}


	inline void set_control_period( double period ) { _control_period = period; }


	virtual void next_step( double dt = Environment::time_step )
	{
		_clock += dt;
		if ( _clock >= _control_period )
		{
			_update_control( _clock );

			_clock = 0;
		}

		for ( segment seg : _segments )
		{
			double angle = _get_axis_angle( seg );
			double rate = _get_axis_rate( seg );
			double correction = -seg.Kp*( angle - seg.angle_setpoint ) - seg.Kd*rate;
			_set_axis_torque( seg, correction );
		}
	}

	inline std::deque<Vector3d> get_trajectory() const { return _effector_trajectory; }


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


	inline double _get_axis_angle( segment seg ) const { return dJointGetHingeAngle( seg.jointID )*RAD_TO_DEG; }

	inline double _get_axis_rate( segment seg ) const { return dJointGetHingeAngleRate( seg.jointID )*RAD_TO_DEG; }

	inline void _set_axis_torque( segment seg, double torque ) { dJointAddHingeTorque( seg.jointID, torque ); }


	Eigen::MatrixXd _get_jacobian() const
	{
		std::vector<Vector3d> joint_pos, joint_axes;
		joint_pos.push_back( Vector3d::Zero() );

		Matrix3d rot = Eigen::Matrix3d::Identity();

		for ( segment seg : _segments )
		{
			joint_axes.push_back( rot*seg.axis );
			rot = AngleAxisd( _get_axis_angle( seg )*DEG_TO_RAD, joint_axes.back() ).toRotationMatrix()*rot;
			joint_pos.push_back( joint_pos.back() + rot*seg.direction*seg.length );
		}

		MatrixXd jacobian( 3, _segments.size() );
		for ( Eigen::Index i = 0 ; i < jacobian.cols() ; ++i )
			jacobian.col( i ) = joint_axes[i].cross( joint_pos.back() - joint_pos[i] )*DEG_TO_RAD;

		return jacobian;
	}


	void _update_control( double delta_t )
	{
		if ( _effector_trajectory.empty() )
			return;

		Vector3d step_vector = _effector_trajectory.front() - get_effector_pos( true );

		VectorXd dq = _get_jacobian().bdcSvd( ComputeThinU | ComputeThinV ).solve( step_vector );
		//VectorXd dq = _get_jacobian().colPivHouseholderQr().solve( step_vector );
		//VectorXd dq = _get_jacobian().fullPivHouseholderQr().solve( step_vector );

		for ( size_t i = 0 ; i < _segments.size() ; ++i )
			_segments[i].angle_setpoint = _get_axis_angle( _segments[i] ) + dq[i];

		if ( _effector_trajectory.size() > 1 )
			_effector_trajectory.pop_front();
	}


	double _clock = 0;
	double _control_period = 0.1;

	std::deque<Vector3d> _effector_trajectory;
};


int main( int argc, char* argv[] )
{
	// [ Dynamic environment ]

	dInitODE();
	ode::Environment env( 0.7 );


	// [ Robot ]

	Arm arm( env );
	arm.set_control_period( 0.01 );

	Vector3d target_0 = arm.get_effector_pos( true );
	Vector3d target_1( 0.5, -0.2, 0.4 );
	Vector3d target_2( 0.7, 0.2, 0.7 );

	arm.set_new_target( target_1, 0.2 );
	arm.set_new_target( target_2, 0.2 );
	arm.set_new_target( target_0, 0.2 );
	arm.set_new_target( target_1, 0.2 );
	arm.set_new_target( target_2, 0.2 );
	arm.set_new_target( target_0, 0.2 );


	Sphere start( env, target_0, 1, 0.05 );
	start.fix();
	start.set_color( 1, 0, 0 );
	start.set_alpha( 0.3 );

	Sphere goal_1( env, target_1, 1, 0.05 );
	goal_1.fix();
	goal_1.set_color( 1, 0, 0 );
	goal_1.set_alpha( 0.3 );

	Sphere goal_2( env, target_2, 1, 0.05 );
	goal_2.fix();
	goal_2.set_color( 1, 0, 0 );
	goal_2.set_alpha( 0.3 );


	// [ Display ]

	int x( 200 ), y( 200 ), width( 1024 ), height( 768 );
	renderer::OsgVisitor* display_ptr = new renderer::OsgVisitor( 0, width, height, x, y, 20, 20, osg::Vec3( 2, -2, 0.5 ), osg::Vec3( 0.2, 0, 0.4 ) );

	display_ptr->set_window_name( "Arm" );
	//display_ptr->disable_shadows();
	display_ptr->get_keh()->set_pause();

	arm.accept( *display_ptr );
	start.accept( *display_ptr );
	goal_1.accept( *display_ptr );
	goal_2.accept( *display_ptr );

	//for ( Vector3d wp : arm.get_trajectory() )
	//{
		//Sphere* pos = new Sphere( env, wp, 1, 0.02 );
		//pos->fix();
		//pos->set_color( 0, 1, 0 );
		//pos->set_alpha( 0.3 );
		//pos->accept( *display_ptr );
	//}

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


	// [ Simulation rules ]

	std::function<bool(float,double)> step_function = [&]( float timestep, double time )
	{
		env.next_step( timestep );
		arm.next_step( timestep );

		return false;
	};

	
	// [ Simulation loop ]

	Sim_loop sim( 0.001, display_ptr, false, 1 );
	//sim.set_fps( 25 );

	if ( argc > 1 && strncmp( argv[1], "capture", 8 ) == 0 )
		sim.start_captures();

	sim.loop( step_function );


	return 0;
}
