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

#include "ode/ft_sensor.hh"
#include "ode/misc.hh"


using namespace ode;
using namespace Eigen;


FT_sensor::FT_sensor( const Object* A, const Object* B, const Vector3d& center, const Vector3d& k_lin_diag, const Vector3d& k_ang_diag,
	                                                                            const Vector3d& c_lin_diag, const Vector3d& c_ang_diag ) : _A( A->get_body() ), _B( B->get_body() )
{
	_set_rel_center_pos( A, center, _oc_A );
	_set_rel_center_pos( B, center, _oc_B );

	_K_lin = DiagonalMatrix<double,3>( k_lin_diag );
	_K_ang = DiagonalMatrix<double,3>( k_ang_diag );
	_C_lin = DiagonalMatrix<double,3>( c_lin_diag );
	_C_ang = DiagonalMatrix<double,3>( c_ang_diag );
}


FT_sensor::FT_sensor( const Object* A, const Object* B, const Vector3d& center, double k_lin, double k_ang, double c_lin, double c_ang ) : _A( A->get_body() ), _B( B->get_body() )
{
	_set_rel_center_pos( A, center, _oc_A );
	_set_rel_center_pos( B, center, _oc_B );

	_K_lin = k_lin*Matrix3d::Identity();
	_K_ang = k_ang*Matrix3d::Identity();
	_C_lin = c_lin*Matrix3d::Identity();
	_C_ang = c_ang*Matrix3d::Identity();
}


void FT_sensor::_set_rel_center_pos( const Object* object_ptr, const Vector3d& center, Vector3d& oc )
{
	dVector3 vec;
	dBodyGetPosRelPoint( object_ptr->get_body(), center.x(), center.y(), center.z(), vec );
	oc = Vector3d( vec[0], vec[1], vec[2] );
}


void FT_sensor::Update()
{
	dVector3 vec;
	dBodyGetRelPointPos( _A, _oc_A.x(), _oc_A.y(), _oc_A.z(), vec );
	Vector3d pos_A = ode_to_vectord( vec );
	dBodyGetRelPointPos( _B, _oc_B.x(), _oc_B.y(), _oc_B.z(), vec );
	Vector3d pos_B = ode_to_vectord( vec );

	dBodyGetRelPointVel( _A, _oc_A.x(), _oc_A.y(), _oc_A.z(), vec );
	Vector3d vel_A = ode_to_vectord( vec );
	dBodyGetRelPointVel( _B, _oc_B.x(), _oc_B.y(), _oc_B.z(), vec );
	Vector3d vel_B = ode_to_vectord( vec );

	_F = _K_lin*( pos_B - pos_A ) + _C_lin*( vel_B - vel_A );


	const dReal* qA = dBodyGetQuaternion( _A );
	const dReal* qB = dBodyGetQuaternion( _B );
	AngleAxisd rel_quat( Quaterniond( qB[0], qB[1], qB[2], qB[3] )*Quaterniond( qA[0], qA[1], qA[2], qA[3] ).inverse() );

	Vector3d angvel_A = ode_to_vectord( dBodyGetAngularVel( _A ) );
	Vector3d angvel_B = ode_to_vectord( dBodyGetAngularVel( _B ) );

	_T = rel_quat.angle()*_K_ang*rel_quat.axis() + _C_ang*( angvel_B - angvel_A );


	dBodyAddForceAtRelPos( _A, _F.x(), _F.y(), _F.z(), _oc_A.x(), _oc_A.y(), _oc_A.z() );
	dBodyAddForceAtRelPos( _B, -_F.x(), -_F.y(), -_F.z(), _oc_B.x(), _oc_B.y(), _oc_B.z() );
	dBodyAddTorque( _A, _T.x(), _T.y(), _T.z() );
	dBodyAddTorque( _B, -_T.x(), -_T.y(), -_T.z() );
}


