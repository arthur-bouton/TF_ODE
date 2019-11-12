/*
** Copyright (C) 2014 Arthur BOUTON
** Copyright (C) 2006 Jean-baptiste MOURET
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef OSGVISITOR_HH
#define OSGVISITOR_HH

#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osgShadow/ShadowedScene>
#include <osgGA/NodeTrackerManipulator>
#include <osg/ShapeDrawable>

#include "ode/object.hh"
#include "osg_keyboard.hh"
//#include "osg_hud.hh"


namespace renderer
{


#define CASTS_SHADOW    0x1
#define RECEIVES_SHADOW 0x2

class OsgVisitor : public ode::ConstVisitor
{
	public:

	typedef enum { FREE, TRACK, FOLLOW, FIXED } camera_t;

	OsgVisitor( unsigned int screen, int wwidth, int wheight, int wxpos, int wypos,
				double ground_length, double ground_width,
				osg::Vec3d cam_pos = osg::Vec3d( 0, 0, 1 ),
				osg::Vec3d cam_center = osg::Vec3d( 0, 0, 0 ),
				camera_t cam = TRACK,
				bool shadows = true,
				osg::Vec3d light_position = osg::Vec3d( -1, -2, 3 ) );
	
	void disable_shadows();

	void set_window_name( std::string name );

	void set_ground_texture( const char* const path_to_texture );

	osgViewer::Viewer* get_viewer();

	KeyboardEventHandler* get_keh();

	inline bool paused() const { return _keh->paused(); }
	inline void set_pause() { _keh->set_pause(); }
	inline bool do_single_step() { return _keh->do_single_step(); }

	//OsgHud* get_hud() { return _hud; }

	void update();

	bool done();

	virtual void visit( const std::vector<ode::Object::ptr_t>& v );
	virtual void visit( const ode::Box& );
	virtual void visit( const ode::CappedCyl& );
	virtual void visit( const ode::Sphere& );
	virtual void visit( const ode::Cylinder& );
	virtual void visit( const ode::Wheel& );
	virtual void visit( const ode::HeightField& );

	void enable_dump( const std::string& prefix );

	inline int get_window_width() const { return _wwidth; }
	inline int get_window_height() const { return _wheight; }

	protected:

	void _set_tm( osg::ref_ptr<osg::PositionAttitudeTransform> pat );

	void _set_object_color( osg::ShapeDrawable* drawable, const ode::Object& o );

	osg::Texture2D* _load_texture( const std::string& fname );

	osg::ref_ptr<osg::Geode> _create_sqr( double width, double length );

	void _create_ground( const ode::Environment& env );

	void _create_std_object( const ode::Object& o, osg::ShapeDrawable* drawable = nullptr );

	void _update_traj();

	osgViewer::Viewer _viewer;
	osg::ref_ptr<KeyboardEventHandler> _keh;
	//OsgHud* _hud;
	osg::ref_ptr<osg::Group> _root;
	bool _shadows;
	osg::Vec3d _light_position;
	osg::ref_ptr<osgShadow::ShadowedScene> _shadowed_scene;
	osg::ref_ptr<osgGA::NodeTrackerManipulator> _tm;
	osg::ref_ptr<osg::PositionAttitudeTransform> _pat_ref;
	camera_t _camera_type;
	osg::Vec3d _camera_home_pos;
	osg::Vec3d _camera_center;
	osg::ref_ptr<osg::Group> _ground;
	double _ground_length, _ground_width;
	osg::Vec3 _prev_pos;
	int _wwidth, _wheight;
	const char* _ground_texture_path;
};


}

#endif
