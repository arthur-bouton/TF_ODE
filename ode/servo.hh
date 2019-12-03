#ifndef SERVO_HH
#define SERVO_HH

#include <boost/shared_ptr.hpp>

#include "object.hh"


namespace ode
{


class Servo
{
	public:

	typedef boost::shared_ptr<Servo> ptr_t;

	typedef enum { POS, VEL } servo_mode_t;

	Servo( Environment& env,
		   Object& o1, Object& o2,
		   const Eigen::Vector3d& anchor,
		   const Eigen::Vector3d& axis,
		   double Kp = 1,
		   double vel_max = 180,
		   dReal min = -dInfinity, dReal max = dInfinity );

    Servo( const Servo& s, Environment& env, Object& o1, Object& o2 );

	virtual ptr_t clone( Environment& env, Object& o1, Object& o2 ) const;

	const Object& get_o1() const;
	const Object& get_o2() const;
	const Eigen::Vector3d& get_anchor() const;
	const Eigen::Vector3d& get_axis() const;

	const dJointID& get_joint() const;

	dReal get_lim_min() const;
	dReal get_lim_max() const;
	void set_lim_min( dReal min );
	void set_lim_max( dReal max );

	bool is_passive() const;
	void set_passive();
	void set_active();
	void set_torque_max( double max );

	double get_angle() const;
	virtual double set_angle( double angle );

	double get_vel() const;
	virtual double set_vel( double vel );

	inline servo_mode_t get_mode() const { return _mode; }

	double get_Kp() const;
	void set_Kp( double Kp );

	double get_vel_max() const;
	void set_vel_max( double vel_max );

	virtual void next_step( double dt );

	double get_real_angle() const;
	double get_real_vel() const;

	~Servo();

	protected:

	void _build();

	Environment& _env;
	Object& _o1;
	Object& _o2;
	Eigen::Vector3d _anchor;
	Eigen::Vector3d _axis;
	dJointID _joint;
	dReal _min, _max;
	bool _passive;
	double _angle;
	double _Kp;
	double _vel_max;
	servo_mode_t _mode;
	double _vel;
};


}

#endif 
