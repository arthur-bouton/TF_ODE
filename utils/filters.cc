/* 
** Definition of the first and second order filter templates
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

#include "filters.hh"
#include <math.h>
#include <stdexcept>


namespace filters
{


template class LP_second_order_bilinear<float>;
template class LP_second_order_bilinear<double>;
template class LP_first_order_bilinear<float>;
template class LP_first_order_bilinear<double>;
template class LP_first_order_homographic<float>;
template class LP_first_order_homographic<double>;
template class LP_first_order_step_matching<float>;
template class LP_first_order_step_matching<double>;
template class LP_first_order_impulse_matching<float>;
template class LP_first_order_impulse_matching<double>;


template <class T>
LP_filter<T>::LP_filter( const float Te, const T* const x_ptr, T* const y_ptr ) :
                         _Te( Te ), _x_ptr( x_ptr ), _y_ptr( y_ptr ), _y_k0( 0 ), _y_k1( 0 ), _x_k1( 0 )
{
	if ( _y_ptr == nullptr )
		_y_ptr = &_y_k0;
}


template <class T>
void LP_filter<T>::update()
{
	if ( _x_ptr == nullptr )
		throw std::runtime_error( "Undefined filter input!" );

	update( *_x_ptr );
}



//---------------------//
// First order filters //
//---------------------//


template <class T>
LP_first_order<T>::LP_first_order( const float Te, const double wc, const T* const x_ptr, T* const y_ptr ) :
                   LP_filter<T>( Te, x_ptr, y_ptr ), _wc( wc ) {}


template <class T>
void LP_first_order<T>::update( const T x_k0 )
{
	*this->_y_ptr = this->_Cy1*this->_y_k1 + this->_Cx0*x_k0 + this->_Cx1*this->_x_k1;
	this->_y_k1 = *this->_y_ptr;
	this->_x_k1 = x_k0;
}


template <class T>
LP_first_order_bilinear<T>::LP_first_order_bilinear( const float Te, const double wc, const T* const x_ptr, T* const y_ptr ) :
                            LP_first_order<T>( Te, wc, x_ptr, y_ptr )
{
	this->_Cy1 = ( 2 - wc*Te )/( 2 + wc*Te );
	this->_Cx0 = wc*Te/( 2 + wc*Te );
	this->_Cx1 = this->_Cx0;
}


template <class T>
LP_first_order_homographic<T>::LP_first_order_homographic( const float Te, const double wc, const T* const x_ptr, T* const y_ptr ) :
                               LP_first_order<T>( Te, wc, x_ptr, y_ptr )
{
	double twt = tan( wc*Te/2 );
	this->_Cy1 = ( 1 - twt )/( 1 + twt );
	this->_Cx0 = twt/( 1 + twt );
	this->_Cx1 = this->_Cx0;
}


template <class T>
LP_first_order_step_matching<T>::LP_first_order_step_matching( const float Te, const double wc, const T* const x_ptr, T* const y_ptr ) :
                                 LP_first_order<T>( Te, wc, x_ptr, y_ptr )
{
	double ewt = exp( -wc*Te );
	this->_Cy1 = ewt;
	this->_Cx0 = 0;
	this->_Cx1 = 1 - ewt;
}


template <class T>
LP_first_order_impulse_matching<T>::LP_first_order_impulse_matching( const float Te, const double wc, const T* const x_ptr, T* const y_ptr ) :
                                    LP_first_order<T>( Te, wc, x_ptr, y_ptr )
{
	double ewt = exp( -wc*Te );
	this->_Cy1 = ewt;
	this->_Cx0 = wc;
	this->_Cx1 = 0;
}



//----------------------//
// Second order filters //
//----------------------//


template <class T>
LP_second_order<T>::LP_second_order( const float Te, const double w0, const float Q, const T* const x_ptr, T* const y_ptr ) :
                    LP_filter<T>( Te, x_ptr, y_ptr ), _w0( w0 ), _Q( Q ), _y_k2( 0 ), _x_k2( 0 ) {}


template <class T>
void LP_second_order<T>::update( const T x_k0 )
{
	*this->_y_ptr = this->_Cy1*this->_y_k1 + _Cy2*_y_k2 + this->_Cx0*x_k0 + this->_Cx1*this->_x_k1 + _Cx2*_x_k2;
	_y_k2 = this->_y_k1;
	this->_y_k1 = *this->_y_ptr;
	_x_k2 = this->_x_k1;
	this->_x_k1 = x_k0;
}


template <class T>
LP_second_order_bilinear<T>::LP_second_order_bilinear( const float Te, const double w0, const float Q, const T* const x_ptr, T* const y_ptr ) :
                             LP_second_order<T>( Te, w0, Q, x_ptr, y_ptr )
{
	double w2T2 = pow( w0*Te, 2 );
	double den = 2*w0*Te + 4*Q + Q*w2T2;
	this->_Cy1 = 2*Q*( 4 - w2T2 )/den;
	this->_Cy2 = ( 2*w0*Te - 4*Q - Q*w2T2 )/den;
	this->_Cx0 = Q*w2T2/den;
	this->_Cx1 = 2*this->_Cx0;
	this->_Cx2 = this->_Cx0;
}


}
