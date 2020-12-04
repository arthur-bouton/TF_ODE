#ifndef ROVER_TF_HH
#define ROVER_TF_HH 

#include "rover.hh"
#include "tf_cpp_binding.hh" // https://github.com/Bouty92/MachineLearning/tree/master/tf_cpp_binding
#include <boost/python.hpp>
#include <random>


namespace robot
{


class Rover_1_tf : public Rover_1
{
	public:

	Rover_1_tf( ode::Environment& env, const Eigen::Vector3d& pose, const char* path_to_actor_model_dir, const int seed = -1 );

	boost::python::list GetState() const;

	inline void SetExploration( bool expl ) { _exploration = expl; }

	inline boost::python::list GetExperience() const { return _experience; }

	inline double GetTotalReward() const { return _total_reward; }

	protected:

	double _ComputeReward( double delta_t );

	virtual void _InternalControl( double delta_t );

	TF_model<float>::ptr_t _actor_model_ptr;
	Eigen::Vector3d _last_pos;
	boost::python::list _last_state;
	boost::python::list _experience;
	double _total_reward;
	bool _exploration;
    std::mt19937 _rd_gen;
    std::normal_distribution<double> _normal_distribution;
};


}

#endif
