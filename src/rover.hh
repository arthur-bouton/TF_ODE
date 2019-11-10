#ifndef ROVER_HH
#define ROVER_HH 

#include "ode/robot.hh"
#include "utils/filters.hh"
#include <osgViewer/Viewer>


#define NBWHEELS 4


namespace robot
{


class Rover_1 : public Robot
{
	public:

	Rover_1( ode::Environment& env, const Eigen::Vector3d& pose );

	virtual void SetRobotSpeed( double speed );
	inline double GetRobotSpeed() const { return _robot_speed; }

	virtual void SetSteeringAngle( double angle );
	virtual double GetSteeringTrueAngle() const;

	virtual void SetSteeringRate( double rate );
	inline double GetSteeringRateCmd() const { return _steering_rate; }
	virtual double GetSteeringTrueRate() const;

	virtual void SetBoggieTorque( double torque );
	inline double GetBoggieTorque() const { return _boggie_torque; }

	inline void SetCmdFreq( double freq ) { _ic_period = 1./freq; }
	inline double GetCmdFreq() const { return 1./_ic_period; }

	inline void SetCmdPeriod( double period ) { _ic_period = period; }
	inline double GetCmdPeriod() const { return _ic_period; }

	inline void ActivateIC() { _ic_activated = true; }
	inline void DeactivateIC() { _ic_activated = false; }
	inline bool IsICActivated() const { return _ic_activated; }

	virtual Eigen::Vector3d GetPosition() const;
	virtual double GetDirection() const;
	virtual bool IsUpsideDown() const;
	virtual double GetRollAngle() const;
	virtual double GetPitchAngle() const;
	virtual void GetTiltRates( double& roll_rate, double& pitch_rate ) const;
	virtual double GetBoggieAngle() const;
	inline const dVector3* GetForkTorsors() const { return _fork_output; }
	inline const double* GetWheelTorques() const { return _torque_output; }

	virtual void PrintForkTorsors( bool endl = true ) const;
	virtual void PrintWheelTorques( bool endl = true ) const;

	virtual void next_step( double dt = ode::Environment::time_step );

	virtual ~Rover_1();

	protected:

	virtual void _InternalControl( double delta_t );

	virtual void _UpdateWheelControl();
	virtual void _ApplyWheelControl();
	virtual void _ApplySteeringControl();
	virtual void _ApplyBoggieControl();

	virtual void _UpdateForkFilters();
	virtual void _UpdateTorqueFilters();
	
	double _robot_speed;
	double _steering_rate;
	double _boggie_torque;

	double steering_max_vel;
	double boggie_max_torque;

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
	Eigen::Vector3d front_pos;

	double rear_mass;
	double rear_length;
	double rear_height;
	double rear_width;
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

	dJointFeedback _front_fork_feedback;
	dJointFeedback _rear_fork_feedback;
	dJointFeedback _wheel_feedback[NBWHEELS];

	filters::LP_second_order<double>::ptr_t _fork_filter[4][3];
	filters::LP_second_order<double>::ptr_t _torque_filter[NBWHEELS];

	dVector3 _fork_output[4];
	double _torque_output[NBWHEELS];

	double _W[NBWHEELS];

	bool _ic_activated;
	double _ic_period;
	double _ic_clock;
};


class RoverControl : public osgGA::GUIEventHandler
{
	public:

	RoverControl( robot::Rover_1* robot, osgViewer::Viewer* viewer, double speed_sensi = 0.01, double turn_sensi = 1., double torque_sensi = 1. ) :
					 _robot_ptr( robot ), _viewer( viewer ),
					 _speed_sensi( speed_sensi ), _turn_sensi( turn_sensi ), _torque_sensi( torque_sensi )
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


}

#endif
