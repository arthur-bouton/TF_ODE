#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <map>
#include <boost/foreach.hpp>

#include "servo.hh"
#include "object.hh"
#include "visitor.hh"


namespace robot
{


class Robot
{
	public:

	typedef boost::shared_ptr<Robot> ptr_t;

	Robot() {}

	inline const std::vector<ode::Object::ptr_t>& bodies() const { return _bodies; }
	inline std::vector<ode::Object::ptr_t>& bodies() { return _bodies; }

	inline const std::vector<ode::Servo::ptr_t>& servos() const { return _servos; }
	inline std::vector<ode::Servo::ptr_t>& servos() { return _servos; }

	Eigen::Vector3d get_pos() const { return _main_body->get_pos(); }
	Eigen::Vector3d get_rot() const { return _main_body->get_rot(); }
	Eigen::Vector3d get_vel() const { return _main_body->get_vel(); }
	
	void disable_shadow_casting()
	{
		BOOST_FOREACH( ode::Object::ptr_t o, _bodies ) 
			o->disable_shadow_casting();
	}

	void set_collision_group( const char* group )
	{
		BOOST_FOREACH( ode::Object::ptr_t o, _bodies ) 
			o->set_collision_group( group );
	}

	void set_color( float r, float g, float b )
	{
		BOOST_FOREACH( ode::Object::ptr_t o, _bodies ) 
			o->set_color( r, g, b );
	}

	void set_alpha( float a )
	{
		BOOST_FOREACH( ode::Object::ptr_t o, _bodies ) 
			o->set_alpha( a );
	}

	virtual void accept( ode::ConstVisitor &v ) const { v.visit( _bodies ); }

	virtual void next_step( double dt = ode::Environment::time_step )
	{
		BOOST_FOREACH( ode::Servo::ptr_t s, _servos ) 
			s->next_step( dt );
	}

	protected:

	std::vector<ode::Object::ptr_t> _bodies;
	std::vector<ode::Servo::ptr_t> _servos;
	ode::Object::ptr_t _main_body;
};


}

#endif
