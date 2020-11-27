#ifndef ROVER_TF_HH
#define ROVER_TF_HH 

#include "rover.hh"
#include <tensorflow/core/public/session.h>
#include <boost/python.hpp>


namespace robot
{


class Rover_1_tf : public Rover_1
{
	public:

	Rover_1_tf( ode::Environment& env, const Eigen::Vector3d& pose, const char* path_to_tf_model, const int seed = -1 );

	boost::python::list GetState() const;

	void InferAction( const boost::python::list& state, double& steering_rate, double& boggie_torque ) const;

	inline void SetExploration( bool expl ) { _exploration = expl; }

	inline boost::python::list GetExperience() const { return _experience; }

	inline double GetTotalReward() const { return _total_reward; }

	virtual ~Rover_1_tf();

	protected:

	double _ComputeReward( double delta_t );

	virtual void _InternalControl( double delta_t );

	tensorflow::Session* _tf_session_ptr;
	Eigen::Vector3d _last_pos;
	boost::python::list _last_state;
	boost::python::list _experience;
	double _total_reward;
	bool _exploration;
    std::mt19937 _rd_gen;
    std::uniform_real_distribution<double> _expl_dist;
    std::uniform_real_distribution<double> _ctrl_dist_uniform;
    std::normal_distribution<double> _ctrl_dist_normal;
};


}

#endif
