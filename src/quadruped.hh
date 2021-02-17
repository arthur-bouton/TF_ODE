#ifndef QUADRUPED_HH
#define QUADRUPED_HH 

#include "ode/robot.hh"
#include <osgViewer/Viewer>
#include "tf_cpp_binding.hh" // https://github.com/Bouty92/MachineLearning/tree/master/tf_cpp_binding
#include <boost/python.hpp>
#include <random>


namespace robot
{


class Quadruped : public Robot
{
	public:

	Quadruped( ode::Environment& env, const Eigen::Vector3d& pose, const char* path_to_actor_model_dir, const int seed = -1 );

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
	double GetRollAngle() const;
	double GetPitchAngle() const;
	void GetTiltRates( double& roll_rate, double& pitch_rate ) const;
	bool IsUpsideDown() const;
	inline bool Collided() const { return _collision; }

	virtual void next_step( double dt = ode::Environment::time_step );

	boost::python::list GetState() const;
	inline void SetExploration( bool expl ) { _exploration = expl; }
	inline boost::python::list GetExperience() const { return _experience; }
	inline double GetTotalReward() const { return _total_reward; }

	//virtual ~Quadruped();

	protected:

	bool _ic_activated;
	double _ic_period;
	double _ic_clock;
	bool _ic_tick;

	double _ComputeReward( double delta_t );

	virtual void _InternalControl( double delta_t );

	TF_model<float>::ptr_t _actor_model_ptr;
	Eigen::Vector3d _last_pos;
	boost::python::list _last_state;
	boost::python::list _actions;
	boost::python::list _experience;
	double _total_reward;
	bool _exploration;
    std::mt19937 _rd_gen;
    std::normal_distribution<double> _normal_distribution;
	std::uniform_real_distribution<double> _uniform_distribution;
	std::vector<float> _state_scaling;
	std::vector<float> _action_scaling;
	bool _collision;
};


}

#endif
