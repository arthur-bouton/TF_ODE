#ifndef ROVER_HH
#define ROVER_HH 

#include "ode/robot.hh"
#include "Filters/cpp/filters.hh" // https://github.com/Bouty92/Filters
#include "ode/ft_sensor.hh"

#include <osgViewer/Viewer>


#define NBWHEELS 4


namespace robot
{


class Rover_1 : public Robot
{
	public:

	Rover_1( ode::Environment& env, const Eigen::Vector3d& pose );

	void SetRobotSpeed( double speed );
	inline double GetRobotSpeed() const { return _robot_speed; }

	void SetSteeringAngle( double angle );
	double GetSteeringTrueAngle() const;

	void SetSteeringRate( double rate );
	inline double GetSteeringRateCmd() const { return _steering_rate; }
	double GetSteeringTrueRate() const;

	inline void SetCrawlingMode( bool crawl ) { _crawling_mode = crawl; }
	inline bool IsCrawlingMode() const { return _crawling_mode; }

	void SetBoggieTorque( double torque );
	inline double GetBoggieTorque() const { return _boggie_torque; }

	inline void SetCmdFreq( double freq ) { _ic_period = 1./freq; }
	inline double GetCmdFreq() const { return 1./_ic_period; }

	inline void SetCmdPeriod( double period ) { _ic_period = period; }
	inline double GetCmdPeriod() const { return _ic_period; }

	inline void ActivateIC() { _ic_activated = true; }
	inline void DeactivateIC() { _ic_activated = false; }
	inline bool IsICActivated() const { return _ic_activated; }
	inline bool ICTick() const { return _ic_tick; }

	Eigen::Vector3d GetPosition() const;
	double GetDirection() const;
	bool IsUpsideDown() const;
	double GetRollAngle() const;
	double GetPitchAngle() const;
	void GetTiltRates( double& roll_rate, double& pitch_rate ) const;
	double GetBoggieAngle() const;
	Eigen::Matrix<double,4,3> GetFT300Torsors() const;
	inline const double* GetWheelTorques() const { return _torque_output; }

	void PrintFT300Torsors( bool endl = true ) const;
	void PrintWheelTorques( bool endl = true ) const;

	virtual void next_step( double dt = ode::Environment::time_step );

	virtual ~Rover_1();

	double steering_max_vel;
	double boggie_max_torque;

	protected:

	virtual void _InternalControl( double delta_t );

	void _UpdateWheelControl();
	void _ApplyWheelControl();
	void _ApplySteeringControl();
	void _ApplyBoggieControl();

	void _UpdateTorqueFilters();
	void _UpdateFtFilters();
	
	double _robot_speed;
	double _steering_rate;
	double _boggie_torque;

	double wheelbase;
	double wheeltrack;

	double wheel_mass;
	double wheel_radius[NBWHEELS];
	double wheel_width;
	double wheel_def;
	double robot_max_speed;

	double belly_elev;

	double front_mass;
	double front_length;
	double front_height;
	double front_width;
	double front_x_offset;
	double front_y_offset;
	Eigen::Vector3d front_pos;

	double rear_mass;
	double rear_length;
	double rear_height;
	double rear_width;
	double rear_x_offset;
	double rear_y_offset;
	Eigen::Vector3d rear_pos;

	Eigen::Vector3d hinge_pos;
	double boggie_angle_max;

	double sea_elev;
	Eigen::Vector3d sea_pos;
	double steering_angle_max;

	double boggie_mass;
	double boggie_length;
	double boggie_height;
	double boggie_width;
	Eigen::Vector3d boggie_pos;

	double fork_elev;
	double fork_mass;
	double fork_length;
	double fork_height;
	double fork_width;

	ode::Object::ptr_t _front_fork;
	ode::Object::ptr_t _rear_fork;
	ode::Object::ptr_t _wheel[NBWHEELS];

	dJointID _boggie_hinge;
	dJointID _wheel_joint[NBWHEELS];

	dJointFeedback _wheel_feedback[NBWHEELS];
	filters::ptr_t<double> _torque_filter[NBWHEELS];
	double _torque_output[NBWHEELS];

	FT_sensor _front_ft_sensor;
	FT_sensor _rear_ft_sensor;
	filters::ptr_t<double> _ft_filter[12];

	double _W[NBWHEELS];

	bool _ic_activated;
	double _ic_period;
	double _ic_clock;
	bool _ic_tick;

	bool _crawling_mode;
};


class RoverControl : public osgGA::GUIEventHandler
{
	public:

	RoverControl( robot::Rover_1* robot, osgViewer::Viewer* viewer, double speed_sensi = 0.01, double turn_sensi = 1., double torque_sensi = 1. ) :
				  _robot_ptr( robot ), _viewer( viewer ), _speed_sensi( speed_sensi ), _turn_sensi( turn_sensi ), _torque_sensi( torque_sensi )
	{
		_viewer->addEventHandler( this );
	}

	virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
	{
		double speed, rate, torque;

		switch ( ea.getKey() )
		{
			case osgGA::GUIEventAdapter::KEY_Up :
				speed = _robot_ptr->GetRobotSpeed();
				speed += _speed_sensi;
				_robot_ptr->SetRobotSpeed( speed );
				return true;

			case osgGA::GUIEventAdapter::KEY_Down :
				speed = _robot_ptr->GetRobotSpeed();
				speed -= _speed_sensi;
				_robot_ptr->SetRobotSpeed( speed );
				return true;

			case osgGA::GUIEventAdapter::KEY_0 :
			case osgGA::GUIEventAdapter::KEY_Exclaim :
				_robot_ptr->SetRobotSpeed( 0 );
				return true;

			case osgGA::GUIEventAdapter::KEY_Left :
				rate = _robot_ptr->GetSteeringRateCmd();
				rate -= _turn_sensi;
				_robot_ptr->SetSteeringRate( rate );
				return true;

			case osgGA::GUIEventAdapter::KEY_Right :
				rate = _robot_ptr->GetSteeringRateCmd();
				rate += _turn_sensi;
				_robot_ptr->SetSteeringRate( rate );
				return true;

			case osgGA::GUIEventAdapter::KEY_Control_R :
				_robot_ptr->SetSteeringRate( 0 );
				return true;

			case osgGA::GUIEventAdapter::KEY_KP_Subtract :
			case osgGA::GUIEventAdapter::KEY_Page_Down :
				torque = _robot_ptr->GetBoggieTorque();
				torque -= _torque_sensi;
				_robot_ptr->SetBoggieTorque( torque );
				return true;

			case osgGA::GUIEventAdapter::KEY_KP_Add :
			case osgGA::GUIEventAdapter::KEY_Page_Up :
				torque = _robot_ptr->GetBoggieTorque();
				torque += _torque_sensi;
				_robot_ptr->SetBoggieTorque( torque );
				return true;

			case osgGA::GUIEventAdapter::KEY_KP_Multiply :
			case osgGA::GUIEventAdapter::KEY_Asterisk :
				_robot_ptr->SetBoggieTorque( 0 );
				return true;
		}
		return false;
	}

	void Detach()
	{
		_viewer->removeEventHandler( this );
	}

	protected:

	robot::Rover_1* _robot_ptr;
	osgViewer::Viewer* _viewer;
	double _speed_sensi;
	double _turn_sensi;
	double _torque_sensi;
};


class Crawler_1 : public Rover_1
{
	public:

	Crawler_1( ode::Environment& env, const Eigen::Vector3d& pose, float torque_amplitude, float dt_torque, float angle_rate = -1, float angle_span = -1 );

	void PrintControls( double time );

	protected:

	virtual void _InternalControl( double delta_t );

	float _torque_amplitude, _angle_rate, _angle_span;

	double _dtorque_dt;

	int _phase;
};


}

#endif
