/* 
** Copyright (C) 2019 Arthur BOUTON
** 
** This program is free software: you can redistribute it and/or modify  
** it under the terms of the GNU General Public License as published by  
** the Free Software Foundation, version 3.
**
** This program is distributed in the hope that it will be useful, but 
** WITHOUT ANY WARRANTY; without even the implied warranty of 
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
** General Public License for more details.
**
** You should have received a copy of the GNU General Public License 
** along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SIM_LOOP_HH
#define SIM_LOOP_HH 

#include "renderer/osg_visitor.hh"
#include <sys/time.h>
#include <unistd.h>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include "renderer/osg_text.hh"


#define DEFAULT_TIMESTEP 0.001 // Seconds
#define DEFAULT_FPS 25 // Frames per second
#define DEFAULT_CAPTURE_PATH "/tmp/robdyn_%05d.bmp"


class Sim_loop
{
	public:

	Sim_loop( float timestep = DEFAULT_TIMESTEP, renderer::OsgVisitor* display_ptr = nullptr, bool print_time = false, int log_level = 0 );

	virtual void loop( std::function<bool(float,double)> step_function );

	inline void set_fps( int fps ) { _fps = fps; _ufperiod = 1e6/_fps; }
	inline double get_time() const { return _time*_timestep; }

	inline void set_timewarp( float warp_factor ) { _utimestep = _timestep*1e6/warp_factor; }

	virtual void set_fps_captures( int fps );
	virtual void start_captures( const char* path = DEFAULT_CAPTURE_PATH );
	virtual void stop_captures();

	virtual ~Sim_loop();

	protected:

	virtual void _update_chrono();
	virtual void _do_print_time();

	float _timestep;
	long _time;
	renderer::OsgVisitor* _display_ptr;
	int _log_level;
	bool _print_time;
	long _nsec;

	int _fps;
	timeval _tv;
	unsigned long long _old_clock, _new_clock;
	unsigned long _chrono;
	long _ufperiod, _utimestep;
	unsigned int _step_counter_u;
	bool _stopped;
	long _display_lead;

	bool _capture;
	const char* _path_captures;
	int _fps_captures;
	unsigned int _capture_rate;
	osg::ref_ptr<osg::Image> _image;
	unsigned int _step_counter_c;

	bool _is_paused;
	renderer::OsgText::ptr_t _paused_text;

	float _warp_factor;
	renderer::OsgText::ptr_t _warp_text;
};


#endif
