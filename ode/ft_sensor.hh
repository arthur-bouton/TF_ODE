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

#ifndef FT_SENSOR_HH
#define FT_SENSOR_HH

#include "ode/object.hh"
#include <Eigen/Core>
#include <Eigen/Geometry>
	

class FT_sensor
{
	public:

	FT_sensor() {}
	FT_sensor( const ode::Object* A, const ode::Object* B, const Eigen::Vector3d& center, const Eigen::Vector3d& k_lin_diag, const Eigen::Vector3d& k_ang_diag,
	                                                                                      const Eigen::Vector3d& c_lin_diag, const Eigen::Vector3d& c_ang_diag );
	FT_sensor( const ode::Object* A, const ode::Object* B, const Eigen::Vector3d& center, double k_lin, double k_ang, double c_lin, double c_ang );

	virtual void Update();

	inline const Eigen::Vector3d* GetForces()  const { return &_F; }
	inline const Eigen::Vector3d* GetTorques() const { return &_T; }

	protected:

	void _set_rel_center_pos( const ode::Object* O, const Eigen::Vector3d& center, Eigen::Vector3d& oc );

	dBodyID _A, _B;
	Eigen::Vector3d _oc_A, _oc_B;
	Eigen::Matrix3d _K_lin, _K_ang, _C_lin, _C_ang;
	Eigen::Vector3d _F, _T;
};


#endif
