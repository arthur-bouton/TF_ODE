/* 
** Header containing the template prototypes for low-pass first and
** second order recursive filters with various approximations (bilinear
** or homographic transforms, step or impulse response matching).
** Parent filter templates cannot be instantiate (they are either
** abstracts or have a protected constructor)
**
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

#ifndef FILTERS_HH
#define FILTERS_HH 

#include <memory>


namespace filters
{


template <class T>
class LP_filter
{
	public:

	typedef std::shared_ptr<LP_filter> ptr_t;

	// Te is the sampling period, x_ptr and y_ptr are optional pointers to the input and output variables:
	LP_filter( const float Te, const T* const x_ptr = nullptr, T* const y_ptr = nullptr );

	// Set a pointer to the input variable:
	inline void set_input( const T* const x_ptr ) { _x_ptr = x_ptr; }
	// Set a pointer to the output variable:
	inline void set_output( T* const y_ptr ) { _y_ptr = y_ptr; }

	// Update the filter by passing the new input value as an argument:
	virtual void update( const T x_k0 ) = 0;
	// If no argument is provided, the pointer to the input variable is used:
	virtual void update();

	// Return the current ouput of the filter:
	inline float get_output() const { return *_y_ptr; }

	// Return the sampling period used:
	inline float get_Te() const { return _Te; }

	protected:

	float _Te;
	double _Cy1, _Cx0, _Cx1;
	const T* _x_ptr;
	T* _y_ptr;
	T _y_k0, _y_k1, _x_k1;
};


// Declaration of an alias declaration following the C++11 standard:
template <class T>
using ptr_t = std::shared_ptr<LP_filter<T>>;



//---------------------//
// First order filters //
//---------------------//


template <class T>
class LP_first_order : public LP_filter<T>
{
	public:

	virtual void update( const T x_k0 );

	inline float get_wc() const { return _wc; }

	protected:

	LP_first_order( const float Te, const double wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr );

	double _wc;
};


template <class T>
class LP_first_order_bilinear : public LP_first_order<T>
{
	public:

	LP_first_order_bilinear( const float Te, const double wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr );
};


template <class T>
class LP_first_order_homographic : public LP_first_order<T>
{
	public:

	LP_first_order_homographic( const float Te, const double wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr );
};


template <class T>
class LP_first_order_step_matching : public LP_first_order<T>
{
	public:

	LP_first_order_step_matching( const float Te, const double wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr );
};


template <class T>
class LP_first_order_impulse_matching : public LP_first_order<T>
{
	public:

	LP_first_order_impulse_matching( const float Te, const double wc, const T* const x_ptr = nullptr, T* const y_ptr = nullptr );
};



//----------------------//
// Second order filters //
//----------------------//


template <class T>
class LP_second_order : public LP_filter<T>
{
	public:

	virtual void update( const T x_k0 );

	inline double get_w0() const { return _w0; }
	inline float get_Q() const { return _Q; }

	protected:

	LP_second_order( const float Te, const double w0, const float Q, const T* const x_ptr = nullptr, T* const y_ptr = nullptr );

	float _Q;
	double _w0, _Cy2, _Cx2;
	T _y_k2, _x_k2;
};


template <class T>
class LP_second_order_bilinear : public LP_second_order<T>
{
	public:

	LP_second_order_bilinear( const float Te, const double w0, const float Q, const T* const x_ptr = nullptr, T* const y_ptr = nullptr );
};


}

#endif
