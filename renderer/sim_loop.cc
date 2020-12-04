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

#include "sim_loop.hh"


Sim_loop::Sim_loop( float timestep, renderer::OsgVisitor* display_ptr, bool print_time, int log_level ) :
                    _timestep( timestep ), _display_ptr( display_ptr ), _log_level( log_level ), _time( 0 ),
					_print_time( print_time ), _nsec( 0 ), _is_paused( false ), _warp_factor( 1 )
{
	if ( _display_ptr != nullptr )
	{
		_fps = DEFAULT_FPS;
		_ufperiod = 1e6/_fps;
		_utimestep = _timestep*1e6;
		_chrono = 0;
		_step_counter_u = 0;
		_stopped = false;
		_fps_captures = DEFAULT_FPS;
		_capture_rate = int( 1./_fps_captures/_timestep );
		_capture = false;
		_image = nullptr;

		_paused_text = _display_ptr->add_text( "", 0 );
		_paused_text->set_pos( 50, 95 );
		_paused_text->set_size( 5 );
		_paused_text->set_color( 10, 10, 10, 0.5 );

		_warp_text = _display_ptr->add_text( "warp", 2 );
		_warp_text->set_pos( 97 );
		_warp_text->set_size( 5 );
		_warp_text->set_color( 10, 10, 10, 0.5 );

		_display_ptr->update();

		gettimeofday( &_tv, nullptr );
		_old_clock = 1e6*_tv.tv_sec + _tv.tv_usec;
	}
}


void Sim_loop::set_fps_captures( int fps )
{
	_fps_captures = fps;
	_capture_rate = int( 1./_fps_captures/_timestep );
}


void Sim_loop::start_captures( const char* path )
{
	assert( _display_ptr != nullptr );

	_path_captures = path;

	if ( _image == nullptr )
		_image = new osg::Image;
	_image->allocateImage( _display_ptr->get_window_width(), _display_ptr->get_window_height(), 1, GL_RGBA, GL_UNSIGNED_BYTE );
	_display_ptr->get_viewer()->getCamera()->attach( osg::Camera::COLOR_BUFFER, _image.get() );
	_step_counter_c = 0;

	_capture = true;
}


void Sim_loop::stop_captures()
{
	//_display_ptr->get_viewer()->getCamera()->detach( osg::Camera::COLOR_BUFFER );
	//delete _image;

	_capture = false;
}


void Sim_loop::_update_chrono()
{
	gettimeofday( &_tv, nullptr );
	_new_clock = 1e6*_tv.tv_sec + _tv.tv_usec;
	_chrono += _new_clock - _old_clock;
	_old_clock = _new_clock;
}


void Sim_loop::_do_print_time()
{
	if ( _time*_timestep > _nsec + 1 )
	{
		_nsec++;
		fprintf( stderr, "> Simulation time: %lds   \r", _nsec );
		fflush( stderr );
	}
}


void Sim_loop::loop( std::function<bool(float,double)> step_function )
{
	if ( _display_ptr != nullptr )
	{
		while( !_display_ptr->done() )
		{
			_update_chrono();

			if ( _is_paused != _display_ptr->get_keh()->paused() )
			{
				_is_paused = !_is_paused;
				if ( _is_paused )
					_paused_text->set_text( "PAUSED" );
				else
					_paused_text->set_text( "" );
			}

			if ( _warp_factor != _display_ptr->get_keh()->get_warp_factor() )
			{
				_warp_factor = _display_ptr->get_keh()->get_warp_factor();
				set_timewarp( _warp_factor );
				if ( _warp_factor == 1 )
					_warp_text->set_text( "" );
				else if ( _warp_factor > 1 )
					_warp_text->set_text( std::string( "real time \u00D7" ) + std::to_string( int( _warp_factor ) ) );
				else if ( _warp_factor < 1 )
					_warp_text->set_text( std::string( "real time \u00F7" ) + std::to_string( int( 1/_warp_factor ) ) );
			}

			if ( _chrono >= _ufperiod )
			{
				_display_ptr->update();

				bool overrun = _utimestep*_step_counter_u < _ufperiod && !_stopped;

				if ( _log_level == 1 && overrun )
					fprintf( stderr, "\033[1;31mOverrun: %li µs\033[0;39m\n", _chrono - _utimestep*_step_counter_u );
				else if ( _log_level >= 2 )
				{
					osg::Vec3f eye, center, up;
					_display_ptr->get_viewer()->getCamera()->getViewMatrixAsLookAt( eye, center, up );
					printf( "%sDisplay lead: %+6li%s | eye: %f %f %f | center: %f %f %f | up: %f %f %f\n",
					        overrun ? "\033[1;31m" : "", overrun ? _utimestep*_step_counter_u - _chrono : _display_lead, overrun ? "\033[0;39m" : "",
							eye.x(), eye.y(), eye.z(), center.x(), center.y(), center.z(), up.x(), up.y(), up.z() );
				}

				//_chrono = 0;
				_chrono -= _ufperiod;
				_step_counter_u = 0;
				_stopped = false;
			}

			if ( !_display_ptr->get_keh()->paused() || _display_ptr->get_keh()->do_single_step() )
			{
				if ( _utimestep*_step_counter_u < _ufperiod )
				{
					if ( _print_time )
						_do_print_time();

					if ( step_function( _timestep, _time*_timestep ) )
						break;

					if ( _capture )
					{
						if ( _step_counter_c % _capture_rate == 0 )
						{
							char image_path[PATH_MAX];
							sprintf( image_path, _path_captures, _step_counter_c/_capture_rate );
							_display_ptr->update();
							osgDB::writeImageFile( *_image, image_path );
						}
						_step_counter_c++;
					}

					_step_counter_u++;

					_time++;
				}
				else
				{
					_update_chrono();
					_display_lead = _ufperiod - _chrono;
					if ( _display_lead > 0 )
						usleep( _display_lead );
				}
			}
			else
			{
				_stopped = true;

				_update_chrono();
				_display_lead = _ufperiod - _chrono;
				if ( _display_lead > 0 )
					usleep( _display_lead );
			}
		}
	}
	else
	{
		while( true )
		{
			if ( _print_time )
				_do_print_time();

			if ( step_function( _timestep, _time*_timestep ) )
				break;

			_time++;
		}
	}
}


Sim_loop::~Sim_loop()
{
	stop_captures();
}
