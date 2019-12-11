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
#include <osgDB/ReadFile>


namespace ode
{


class HeightField : public Object
{
	public:

	HeightField( Environment& env , const Eigen::Vector3d& pos, double* heightmap, int n_rows, int n_cols,
	double l, double w, double skirt_height = 0, double bound_min = -dInfinity, double bound_max = dInfinity, bool casts_shadow = false ) :
	Object( env, pos ), data( heightmap ), nrow( n_rows ), ncol( n_cols ), length( l ), width( w ), skirt( skirt_height ), min( bound_min ), max( bound_max ), texture_path( nullptr )
	{
		data_alloc = false;
		_casts_shadow = casts_shadow;
		init();
	}

	HeightField( Environment& env , const Eigen::Vector3d& pos, const char* heightimage_path, int z_scale,
	double l, double w, double skirt_height = 0, double bound_min = -dInfinity, double bound_max = dInfinity, bool casts_shadow = false ) :
	Object( env, pos ), length( l ), width( w ), skirt( skirt_height ), min( bound_min ), max( bound_max ), texture_path( nullptr )
	{
		osg::Image* heightimage = osgDB::readImageFile( heightimage_path );
		if ( heightimage == nullptr )
			throw std::runtime_error( std::string( "Can't open " ) + std::string( heightimage_path ) );
		nrow = heightimage->t();
		ncol = heightimage->s();
		data = (double*) calloc( nrow*ncol, sizeof( double ) );
		for ( int r = 0 ; r < nrow ; r++ )
			for ( int c = 0 ; c < ncol ; c++ )
				data[(nrow-r-1)*ncol+c] = ( *heightimage->data( c, r ) )*0.3/255;

		data_alloc = true;
		_casts_shadow = casts_shadow;
		init();
	}

	virtual void set_texture( const char* const path_to_texture )
	{
		texture_path = path_to_texture;
	}

	virtual void accept(ConstVisitor &v) const
	{
		assert( !_geoms.empty() );
		v.visit(*this);
	}

	virtual ~HeightField()
	{
		if ( data_alloc )
			free( data );
	}


	double* data;
	int nrow;
	int ncol;
	double length;
	double width;
	double skirt;
	double min;
	double max;
	const char* texture_path;

	protected:

	void init()
	{
		_id = dGeomHeightfieldDataCreate();
		dGeomHeightfieldDataBuildDouble( _id, data, 0, length, width, ncol, nrow, 1, 0, skirt, 0 );
		dGeomHeightfieldDataSetBounds( _id, min, max );
		dGeomID g = dCreateHeightfield( _env.get_space(), _id, 1 );

		dGeomSetPosition( g, _init_pos.x(), _init_pos.y(), _init_pos.z() );
		dMatrix3 R;
		dRSetIdentity( R );
		dRFromAxisAndAngle( R, 1, 0, 0, 1.5707963267948966 );
		dGeomSetRotation( g, R );

		_geoms.push_back( g );
	}


	dHeightfieldDataID _id;
	bool data_alloc;
};


}


#endif
