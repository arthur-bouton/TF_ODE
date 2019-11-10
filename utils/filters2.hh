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

#ifndef FILTERS2_HH
#define FILTERS2_HH 

#include <memory>
#include <math.h>
#include <stdexcept>


namespace filters
{


template <class T>
class LP_second_order
{
	public:

	typedef std::shared_ptr<LP_second_order> ptr_t;

	LP_second_order() : _y_k0( 0 ), _y_k1( 0 ), _y_k2( 0 ), _x_k1( 0 ), _x_k2( 0 ) {}

	virtual void init_bilinear( const float Te, const float w0, const float Q, const T* const x_ptr = nullptr, T* const y_ptr = nullptr )
	{
		_init( Te, w0, Q, x_ptr, y_ptr );

		double w2T2 = pow( w0*Te, 2 );
		double den = 2*w0*Te + 4*Q + Q*w2T2;
		_Cy1 = 2*Q*( 4 - w2T2 )/den;
		_Cy2 = ( 2*w0*Te - 4*Q - Q*w2T2 )/den;
		_Cx0 = Q*w2T2/den;
		_Cx1 = 2*_Cx0;
		_Cx2 = _Cx0;
	}

	virtual void reset()
	{
		_y_k0 = 0;
		_y_k1 = 0;
		_y_k2 = 0;
		_x_k1 = 0;
		_x_k2 = 0;
	}

	inline void set_input( const T* const x_ptr ) { _x_ptr = x_ptr; }
	inline void set_output( T* const y_ptr ) { _y_ptr = y_ptr; }

	virtual void update( const T x_k0 )
	{
		*_y_ptr = _Cy1*_y_k1 + _Cy2*_y_k2 + _Cx0*x_k0 + _Cx1*_x_k1 + _Cx2*_x_k2;
		_y_k2 = _y_k1;
		_y_k1 = *_y_ptr;
		_x_k2 = _x_k1;
		_x_k1 = x_k0;
	}

	virtual void update()
	{
		if ( _x_ptr == nullptr )
			throw std::runtime_error( "Undefined filter input!" );

		update( *_x_ptr );
	}

	inline float get_output() const { return *_y_ptr; }

	inline float get_Te() const { return _Te; }
	inline float get_w0() const { return _w0; }
	inline float get_Q() const { return _Q; }

	protected:

	virtual void _init( const float Te, const float w0, const float Q, const T* const x_ptr = nullptr, T* const y_ptr = nullptr )
	{
		_Te = Te;
		_w0 = w0;
		_Q = Q;
		_x_ptr = x_ptr;
		_y_ptr = y_ptr;

		if ( _y_ptr == nullptr )
			_y_ptr = &_y_k0;
	}

	float _Te, _w0, _Q;
	double _Cy1, _Cy2, _Cx0, _Cx1, _Cx2;
	const T* _x_ptr;
	T* _y_ptr;
	T _y_k0, _y_k1, _y_k2, _x_k1, _x_k2;
};


template <class T>
class LP_first_order
{
	public:

	typedef std::shared_ptr<LP_first_order> ptr_t;

	LP_first_order() : _y_k0( 0 ), _y_k1( 0 ), _x_k1( 0 ) {}

	virtual void init_bilinear( const float Te, const float wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr )
	{
		_init( Te, wc, x_ptr, y_ptr );

		_Cy1 = ( 2 - wc*Te )/( 2 + wc*Te );
		_Cx0 = wc*Te/( 2 + wc*Te );
		_Cx1 = _Cx0;
	}

	virtual void init_homographic( const float Te, const float wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr )
	{
		_init( Te, wc, x_ptr, y_ptr );

		double twt = tan( wc*Te/2 );
		_Cy1 = ( 1 - twt )/( 1 + twt );
		_Cx0 = twt/( 1 + twt );
		_Cx1 = _Cx0;
	}

	virtual void init_step_matching( const float Te, const float wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr )
	{
		_init( Te, wc, x_ptr, y_ptr );

		double ewt = exp( -wc*Te );
		_Cy1 = ewt;
		_Cx0 = 0;
		_Cx1 = 1 - ewt;
	}

	virtual void init_impulse_matching( const float Te, const float wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr )
	{
		_init( Te, wc, x_ptr, y_ptr );

		double ewt = exp( -wc*Te );
		_Cy1 = ewt;
		_Cx0 = wc;
		_Cx1 = 0;
	}

	virtual void reset()
	{
		_y_k0 = 0;
		_y_k1 = 0;
		_x_k1 = 0;
	}

	inline void set_input( const T* const x_ptr ) { _x_ptr = x_ptr; }
	inline void set_output( T* const y_ptr ) { _y_ptr = y_ptr; }

	virtual void update( const T x_k0 )
	{
		*_y_ptr = _Cy1*_y_k1 + _Cx0*x_k0 + _Cx1*_x_k1;
		_y_k1 = *_y_ptr;
		_x_k1 = x_k0;
	}

	virtual void update()
	{
		if ( _x_ptr == nullptr )
			throw std::runtime_error( "Undefined filter input!" );

		update( *_x_ptr );
	}

	inline float get_output() const { return *_y_ptr; }

	inline float get_Te() const { return _Te; }
	inline float get_wc() const { return _wc; }

	protected:

	virtual void _init( const float Te, const float wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr )
	{
		_Te = Te;
		_wc = wc;
		_x_ptr = x_ptr;
		_y_ptr = y_ptr;

		if ( _y_ptr == nullptr )
			_y_ptr = &_y_k0;
	}

	float _Te, _wc;
	double _Cy1, _Cx0, _Cx1;
	const T* _x_ptr;
	T* _y_ptr;
	T _y_k0, _y_k1, _x_k1;
};


}

#endif
