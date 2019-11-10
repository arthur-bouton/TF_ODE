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

	Robot( const Robot& o, ode::Environment& env )
	{
		using namespace ode;

		std::map<const Object*, Object::ptr_t> old_to_new;
		BOOST_FOREACH( Object::ptr_t b, o._bodies )
		{
			ode::Object::ptr_t b2 = b->clone( env );
			old_to_new[b.get()] = b2;
			_bodies.push_back( b2 );
		}
		BOOST_FOREACH( Servo::ptr_t s, o._servos )
			_servos.push_back( s->clone( env, *old_to_new[&s->get_o1()], *old_to_new[&s->get_o2()] ) );

		_main_body = old_to_new[o._main_body.get()];
	}

	virtual ptr_t clone( ode::Environment& env ) const { return ptr_t( new Robot( *this, env ) ); }

	const std::vector<ode::Object::ptr_t>& bodies() const { return _bodies; }
	std::vector<ode::Object::ptr_t>& bodies() { return _bodies; }

	const std::vector<ode::Servo::ptr_t>& servos() const { return _servos; }
	std::vector<ode::Servo::ptr_t>& servos() { return _servos; }

	Eigen::Vector3d get_pos() const { return _main_body->get_pos(); }
	Eigen::Vector3d get_rot() const { return _main_body->get_rot(); }
	Eigen::Vector3d get_vel() const { return _main_body->get_vel(); }
	
	virtual void disable_shadow_casting()
	{
		BOOST_FOREACH( ode::Object::ptr_t o, _bodies ) 
			o->disable_shadow_casting();
	}

	virtual void set_collision_group( const char* group )
	{
		BOOST_FOREACH( ode::Object::ptr_t o, _bodies ) 
			o->set_collision_group( group );
	}

	virtual void set_color( float r, float g, float b )
	{
		BOOST_FOREACH( ode::Object::ptr_t o, _bodies ) 
			o->set_color( r, g, b );
	}

	virtual void set_alpha( float a )
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
