#include "osg_visitor.hh"

#include <osgViewer/ViewerEventHandlers>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/Node>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osgShadow/ShadowMap>
#include <osgDB/WriteFile>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <iomanip>
#include <osg/TexMat>

#include "ode/capped_cyl.hh"
#include "ode/box.hh"
#include "ode/sphere.hh"
#include "ode/cylinder.hh"
#include "ode/wheel.hh"
#include "ode/heightfield.hh"


using namespace osg;

namespace renderer
{


inline Vec3 _osg_pos( const ode::Object& b )
{
	const dReal* pos = dBodyGetPosition( b.get_body() );
	return Vec3( pos[0], pos[1], pos[2] );
}


inline Quat _osg_quat( const ode::Object& b )
{
	const dReal* q = dBodyGetQuaternion( b.get_body() );
	return Quat( q[1], q[2], q[3], q[0] );
}


// apply ode transformation position only
class PositionCallback : public NodeCallback
{
	public:

	PositionCallback( const ode::Object& obj ) : _obj( obj ) {}

	virtual void operator()( Node* node, NodeVisitor* nv )
	{
		PositionAttitudeTransform* pat = dynamic_cast<PositionAttitudeTransform*>( node );
		pat->setPosition( _osg_pos( _obj ) );
		traverse( node, nv );
	}

	protected:

	const ode::Object& _obj;
};


// apply ode transformation
class UpdateCallback : public NodeCallback
{
	public:

	UpdateCallback( const ode::Object& obj ) : _obj( obj ) {}

	virtual void operator()( Node* node, NodeVisitor* nv )
	{
		PositionAttitudeTransform* pat = dynamic_cast<PositionAttitudeTransform*>( node );
		pat->setPosition( _osg_pos( _obj ) );
		pat->setAttitude( _osg_quat( _obj ) );
		traverse( node, nv );
	}

	protected:

	const ode::Object& _obj;
};


struct ImgPostDrawCallback : public Camera::DrawCallback
{
	ImgPostDrawCallback( const std::string& prefix ) : _prefix( prefix ), _image( new Image ), _k( 0 ) {}

	virtual void operator() ( const Camera &camera ) const
	{
		int x = ( int ) camera.getViewport()->x();
		int y = ( int ) camera.getViewport()->y();
		int width = ( int ) camera.getViewport()->width(); 
		int height = ( int )camera.getViewport()->height();

		_image->readPixels( x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE );
		osgDB::writeImageFile( *_image ,
							   _prefix + boost::str(boost::format("%1%")%boost::io::group(std::setfill('0'),std::setw(3),_k++)) + ".png" );
	}

	private:

	std::string _prefix;
	ref_ptr<Image> _image;
	mutable size_t _k;
};


OsgVisitor::OsgVisitor( unsigned int screen, int wwidth, int wheight, int wxpos, int wypos,
						double ground_length, double ground_width,
						Vec3d cam_pos, Vec3d cam_center, camera_t cam, bool shadows, Vec3d light_position ) :
						_ground_length( ground_length ),
						_ground_width( ground_width ),
						_camera_home_pos( cam_pos ),
						_camera_center( cam_center ),
						_pat_ref( NULL ),
						_camera_type( cam ),
						_root( new Group() ),
						_shadows( shadows ),
						_light_position( light_position ),
						_tm( NULL ),
						_keh( new KeyboardEventHandler( &_viewer ) ),
						_wwidth( wwidth ),
						_wheight( wheight ),
						_ground_texture_path( "../env_data/checker.tga" )
{
	if ( wwidth != 0 && wheight != 0 )
		_viewer.setUpViewInWindow( wxpos, wypos, wwidth, wheight, screen );

	osgViewer::WindowSizeHandler* wsh = new osgViewer::WindowSizeHandler;
	_viewer.addEventHandler( wsh );

	if ( _camera_type == TRACK || _camera_type == FOLLOW )
	{
		_tm = new osgGA::NodeTrackerManipulator;
		_tm->setRotationMode( osgGA::NodeTrackerManipulator::ELEVATION_AZIM );
		if ( _camera_type == TRACK )
			_tm->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER );
		if ( _camera_type == FOLLOW )
			_tm->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_AZIM );
	}

	if ( _camera_type == FREE )
	{
		_viewer.setCameraManipulator( new osgGA::OrbitManipulator() );
		_viewer.getCameraManipulator()->setHomePosition( _camera_home_pos,
														 _camera_center,
			   											 Vec3( 0, 0, 1 ) );
	}
	_viewer.addEventHandler( new osgViewer::StatsHandler );
	_viewer.addEventHandler( _keh.get() );
	_viewer.realize();

	//      _create_ground();
	//      _hud = new OsgHud( _shadowed_scene.get() );

	//_viewer.getCamera()->setClearColor( Vec4( 204./255, 82./255, 0./255, 0 ) );
	_viewer.getCamera()->setClearColor( Vec4( 179./255, 71./255, 0./255, 0 ) );
}


void OsgVisitor::disable_shadows() { _shadows = false; }


void OsgVisitor::set_window_name( std::string name )
{
	osgViewer::Viewer::Windows windows;
	_viewer.getWindows( windows );
	windows[0]->setWindowName( name );
}


osgViewer::Viewer* OsgVisitor::get_viewer() { return &_viewer; }


KeyboardEventHandler* OsgVisitor::get_keh() { return _keh.get(); }


void OsgVisitor::visit( const std::vector<ode::Object::ptr_t>& v )
{
	assert( v.size() );//hack..
	if ( !_viewer.getSceneData() )
		_create_ground( v[0]->get_env() );

	BOOST_FOREACH( ode::Object::ptr_t o, v )
		o->accept( *this );
}


void OsgVisitor::_create_std_object( const ode::Object& o, ShapeDrawable* drawable )
{
	ref_ptr<PositionAttitudeTransform> pat( new PositionAttitudeTransform );

	if ( drawable != nullptr )
	{
		ref_ptr<Geode> geode( new Geode );
		if ( !o.casts_shadow() )
			geode->setNodeMask( geode->getNodeMask() & ~CASTS_SHADOW );

		_set_object_color( drawable, o );
		geode->addDrawable( drawable );

		pat->addChild( geode.get() );
	}
	else if ( o.get_mesh_path() )
	{
		ref_ptr<Node> pLoadedModel( osgDB::readNodeFile( o.get_mesh_path() ) );
		//if ( ! pLoadedModel );
			//throw std::runtime_error( std::string( "Could not load " ) + std::string( o.get_mesh_path() ) );
		if ( !o.casts_shadow() )
			pLoadedModel->setNodeMask( pLoadedModel->getNodeMask() & ~CASTS_SHADOW );

		pat->addChild( pLoadedModel );
	}

	ref_ptr<NodeCallback> cb( new UpdateCallback( o ) );
	pat->setUpdateCallback( cb );

	_root->addChild( pat );
	_set_tm( pat );
}


void OsgVisitor::visit( const ode::Box& o )
{
	if ( o.get_mesh_path() )
		_create_std_object( o );
	else
	{
		ref_ptr<ShapeDrawable> drawable( new ShapeDrawable( new Box( Vec3d(), o.get_length(), o.get_width(), o.get_height() ) ) );
		_create_std_object( o, drawable );
	}
}


void OsgVisitor::visit( const ode::Sphere& o )
{
	if ( o.get_mesh_path() )
		_create_std_object( o );
	else
	{
		ref_ptr<ShapeDrawable> drawable( new ShapeDrawable( new Sphere( Vec3d(), o.get_rad() ) ) );
		_create_std_object( o, drawable );
	}
}


void OsgVisitor::visit( const ode::CappedCyl& o )
{
	if ( o.get_mesh_path() )
		_create_std_object( o );
	else
	{
		ref_ptr<ShapeDrawable> drawable( new ShapeDrawable( new Capsule( Vec3d(), o.get_radius(), o.get_length() ) ) );
		_create_std_object( o, drawable );
	}
}


void OsgVisitor::visit( const ode::Cylinder& o )
{
	if ( o.get_mesh_path() )
		_create_std_object( o );
	else
	{
		ref_ptr<ShapeDrawable> drawable( new ShapeDrawable( new Cylinder( Vec3d(), o.get_rad(), o.get_width() ) ) );
		_create_std_object( o, drawable );
	}
}


void OsgVisitor::visit( const ode::Wheel& o )
{
	ref_ptr<PositionAttitudeTransform> pat( new PositionAttitudeTransform );

	if ( o.get_mesh_path() )
	{
		ref_ptr<Node> pLoadedModel( osgDB::readNodeFile( o.get_mesh_path() ) );
		//if ( ! pLoadedModel );
			//throw std::runtime_error( std::string( "Could not load " ) + std::string( o.get_mesh_path() ) );
		if ( !o.casts_shadow() )
			pLoadedModel->setNodeMask( pLoadedModel->getNodeMask() & ~CASTS_SHADOW );

		pat->addChild( pLoadedModel );
	}
	else
	{
		double rad = o.get_rad();
		double width = o.get_width();
		double def = o.get_def();

		ref_ptr<Geode> geode( new Geode );
		if ( !o.casts_shadow() )
			geode->setNodeMask( geode->getNodeMask() & ~CASTS_SHADOW );

		ShapeDrawable* drawable = new ShapeDrawable( new Cylinder( Vec3d(), rad, width ) );
		_set_object_color( drawable, o );
		geode->addDrawable( drawable );
		for ( int i = 0 ; i < def ; i++ )
		{
			ShapeDrawable* drawable = new ShapeDrawable( new Sphere( Vec3d( rad*cos( i*2*M_PI/def ), rad*sin( i*2*M_PI/def ), 0 ), width/2 ) );
			_set_object_color( drawable, o );
			geode->addDrawable( drawable );
		}

		pat->addChild( geode.get() );
	}

	ref_ptr<NodeCallback> cb( new UpdateCallback( o ) );
	pat->setUpdateCallback( cb );

	_root->addChild( pat );
	_set_tm( pat );
}


void OsgVisitor::visit( const ode::HeightField& o )
{
	ref_ptr<Geode> geode( new Geode );
	if ( !o.casts_shadow() )
		geode->setNodeMask( geode->getNodeMask() & ~CASTS_SHADOW );

	double* data = o.data;
	int nrow = o.nrow;
	int ncol = o.ncol;
	double length = o.length;
	double width = o.width;
	double skirt = o.skirt;

	osg::HeightField* heightField = new osg::HeightField();
	heightField->allocate( ncol, nrow );
	heightField->setOrigin( osg::Vec3( o.get_init_pos().x() - length/2, o.get_init_pos().y() - width/2, o.get_init_pos().z() ) );
	heightField->setXInterval( length/ncol );
	heightField->setYInterval( width/nrow );
	heightField->setSkirtHeight( skirt );

	for ( int r = 0 ; r < nrow ; r++ )
		for ( int c = 0 ; c < ncol ; c++ )
			heightField->setHeight( c, r, *( data + ( nrow - r - 1 )*ncol + c ) );

	ShapeDrawable* drawable = new osg::ShapeDrawable( heightField );
	//_set_object_color( drawable, o );
	geode->addDrawable( drawable );

	const char* path_to_texture = o.texture_path;
	if ( path_to_texture == nullptr )
		path_to_texture = _ground_texture_path;
	osg::Texture2D* tex = new osg::Texture2D( osgDB::readImageFile( path_to_texture ) );
	tex->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_LINEAR );
	tex->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
	tex->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
	tex->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );

	Matrixd matrix;
	matrix.makeScale( Vec3( length, width, 1 ) );
	ref_ptr<TexMat> matTexture = new TexMat;
	matTexture->setMatrix( matrix );

	geode->getOrCreateStateSet()->setTextureAttributeAndModes( 0, tex );
	geode->getOrCreateStateSet()->setTextureAttributeAndModes( 0, matTexture ); 

	ref_ptr<PositionAttitudeTransform> pat( new PositionAttitudeTransform );
	pat->addChild( geode.get() );

	_root->addChild( pat.get() );
	_set_tm( pat );
}


void OsgVisitor::enable_dump( const std::string& prefix )
{ 
	_viewer.getCamera()->setPostDrawCallback( new ImgPostDrawCallback( prefix ) );
}


void OsgVisitor::update()
{
	if ( _camera_type == FIXED )
		_viewer.getCamera()->setViewMatrixAsLookAt( _camera_home_pos ,
													_camera_center,
													Vec3d( 0, 0, 1 ) );
	_viewer.frame();
	//_update_traj();
}


bool OsgVisitor::done() { return _viewer.done(); }


void OsgVisitor::_set_tm( ref_ptr<PositionAttitudeTransform> pat )
{
	if ( _pat_ref == NULL )
		_pat_ref = pat;

	if ( _tm == NULL )
		return;

	if ( !_tm->getTrackNode() && ( _camera_type == TRACK || _camera_type == FOLLOW ) )
	{
		ref_ptr<PositionAttitudeTransform> patc( new PositionAttitudeTransform() );
		patc->setPosition( _camera_center );
		pat->addChild( patc.get() );

		_tm->setTrackNode( patc.get() );
		_tm->setNode( patc.get() );
		_tm->setHomePosition( _camera_home_pos,
							  Vec3(),
			   				  Vec3( 0, 0, 1 ) );

		_viewer.setCameraManipulator( _tm.get() );
	}
}


void OsgVisitor::_set_object_color( ShapeDrawable* drawable, const ode::Object& o )
{
	const float* color = o.get_color();
	float alpha = o.get_alpha();
	if ( color != NULL )
		drawable->setColor( Vec4( color[0], color[1], color[2], alpha ) );
	else if ( alpha != 1 )
		drawable->setColor( Vec4( drawable->getColor()[0], drawable->getColor()[1], drawable->getColor()[2], alpha ) );
	if ( alpha != 1 )
		drawable->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
}


Texture2D* OsgVisitor::_load_texture( const std::string& fname )
{
	Texture2D* texture = new Texture2D();
	Image* image = osgDB::readImageFile( fname );
	assert( image );
	texture->setImage( image );
	texture->setWrap( Texture::WRAP_S, Texture::REPEAT );
	texture->setWrap( Texture::WRAP_T, Texture::REPEAT );
	texture->setDataVariance( Object::DYNAMIC );
	return texture;
}


ref_ptr<Geode> OsgVisitor::_create_sqr( double width, double length )
{
	ref_ptr<Geometry> sqr( new Geometry );
	ref_ptr<Geode> geode_sqr( new Geode );
	ref_ptr<Vec3Array> sqr_v( new Vec3Array );

	geode_sqr->addDrawable( sqr.get() );
	double x = width;
	double y = length;
	sqr_v->push_back( Vec3( 0, 0, 0 ) );
	sqr_v->push_back( Vec3( 0, y, 0 ) );
	sqr_v->push_back( Vec3( x, y, 0 ) );
	sqr_v->push_back( Vec3( x, 0, 0 ) );
	sqr->setVertexArray( sqr_v.get() );

	ref_ptr<DrawElementsUInt> quad( new DrawElementsUInt( PrimitiveSet::QUADS, 0 ) );
	quad->push_back( 3 );
	quad->push_back( 2 );
	quad->push_back( 1 );
	quad->push_back( 0 );

	sqr->addPrimitiveSet( quad.get() );

	ref_ptr<Vec2Array> texcoords( new Vec2Array( 4 ) );
	double rep = 1;
	(*texcoords)[0].set( 0.0f, 0.0f );
	(*texcoords)[1].set( 0.0f, rep );
	(*texcoords)[2].set( rep, rep );
	(*texcoords)[3].set( rep, 0.0f );
	sqr->setTexCoordArray( 0, texcoords.get() );

	ref_ptr<Vec3Array> normals( new Vec3Array );
	normals->push_back( Vec3( 0, 0, 1 ) );
	sqr->setNormalArray( normals.get() );
	sqr->setNormalBinding( Geometry::BIND_OVERALL );

	return geode_sqr;
}


void OsgVisitor::set_ground_texture( const char* const path_to_texture )
{
	_ground_texture_path = path_to_texture;
}


void OsgVisitor::_create_ground( const ode::Environment &env )
{
	const double checker_length = 1;
	const double checker_width = 1;

	ref_ptr<PositionAttitudeTransform> patg( new PositionAttitudeTransform() );
	patg->setAttitude( Quat( env.get_roll(), Vec3( 0, 1, 0 ),
							 env.get_pitch(), Vec3( 1, 0, 0 ),
							 0, Vec3( 0, 0, 1 ) ) );
	patg->setPosition( Vec3( 0, 0, env.get_z() ) );

	if ( env.get_ground() != 0x0 )
		_root->addChild( patg );

	_ground = new Group;
	patg->addChild( _ground.get() );

	const double x_nb_checkers = ceil( _ground_length/checker_length );
	const double y_nb_checkers = ceil( _ground_width/checker_width );

	for ( int i = 0 ; i < x_nb_checkers ; ++i )
		for ( int j = 0 ; j < y_nb_checkers ; ++j )
		{
			ref_ptr<Geode> geode_sqr = _create_sqr( checker_length, checker_width );
			geode_sqr->setNodeMask( geode_sqr->getNodeMask() & ~CASTS_SHADOW );

			ref_ptr<PositionAttitudeTransform> pat( new PositionAttitudeTransform() );
			pat->setPosition( Vec3( ( i - x_nb_checkers/2 )*checker_length, ( j - y_nb_checkers/2 )*checker_width, 0 ) );
			pat->addChild( geode_sqr.get() );

			ref_ptr<StateSet> ss_checker( new StateSet() );
			ss_checker->setTextureAttributeAndModes( 0, _load_texture( _ground_texture_path ) );
			geode_sqr->setStateSet( ss_checker.get() );

			_ground->addChild( pat.get() );
		}


	_shadowed_scene = new osgShadow::ShadowedScene;

	if ( _shadows )
	{
		ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
		_shadowed_scene->setShadowTechnique( sm.get() );

		_shadowed_scene->setCastsShadowTraversalMask( CASTS_SHADOW );
		_shadowed_scene->setReceivesShadowTraversalMask( RECEIVES_SHADOW );
	}

	LightSource*ls = new LightSource;
	ls->getLight()->setPosition( Vec4( _light_position, 0 ) );
	ls->getLight()->setDiffuse( Vec4( 1, 1, 1, 0 ) );
	_shadowed_scene->addChild( ls );

	_shadowed_scene->addChild( _root.get() );

	_viewer.setSceneData( _shadowed_scene.get() );
}


void OsgVisitor::_update_traj()
{
	Vec3 pos = _pat_ref->getBound().center()*computeLocalToWorld( _pat_ref->getParentalNodePaths()[0] );
	if ( ( pos - _prev_pos ).length() > 0.02 )
	{
		_prev_pos = pos;
		ref_ptr<Geode> geode( new Geode );
		ref_ptr<ShapeDrawable> sphere = new ShapeDrawable( new Sphere( Vec3( pos.x(), pos.y(), 0 ), 0.01 ) );
		sphere->setColor( Vec4( 1, 0, 0, 0.5 ) ); 
		sphere->getOrCreateStateSet()->setMode( GL_BLEND, StateAttribute::ON );
		geode->addDrawable( sphere.get() );

		_shadowed_scene->addChild( geode.get() );
	}
}


}
