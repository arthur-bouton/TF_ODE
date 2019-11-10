/* 
** Copyright (C) 2014 Arthur BOUTON
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

#ifndef   	HEIGHTFIELD_HH_
#define   	HEIGHTFIELD_HH_

#include <iostream>
#include "object.hh"

namespace ode
{
  class HeightField : public Object
  {
  public:
    HeightField( Environment& env , const Eigen::Vector3d& pos, double* heightmap, int n_rows, int n_cols,
	double l, double w, double skirt_height = 0, double bound_min = -dInfinity, double bound_max = dInfinity, bool casts_shadow = false ) :
	Object( env, pos ), data( heightmap ), nrow( n_rows ), ncol( n_cols ), length( l ), width( w ), skirt( skirt_height ), min( bound_min ), max( bound_max )
    {
      _casts_shadow = casts_shadow;
      init();
    }

    virtual Object::ptr_t clone(Environment& env) const 
    { return NULL; }

    void init_again()
    {
      if (_geom)
		dGeomDestroy(_geom);
      init();
    }
  
    virtual void accept(ConstVisitor &v) const
    {
      assert(_geom);
      v.visit(*this);
    }
	double* data;
	int nrow;
	int ncol;
	double length;
	double width;
	double skirt;
	double min;
	double max;
	
  protected:
    void init()
    {
      //Object::init();
		_id = dGeomHeightfieldDataCreate();
		dGeomHeightfieldDataBuildDouble( _id, data, 0, length, width, ncol, nrow, 1, 0, skirt, 0 );
		dGeomHeightfieldDataSetBounds( _id, min, max );
		_geom = dCreateHeightfield( _env.get_space(), _id, 1 );

		dGeomSetPosition( _geom, _init_pos.x(), _init_pos.y(), _init_pos.z() );
		dMatrix3 R;
		dRSetIdentity( R );
		dRFromAxisAndAngle( R, 1, 0, 0, 1.5707963267948966 );
		dGeomSetRotation( _geom, R );
    }
	dHeightfieldDataID _id;
  };
}


#endif
