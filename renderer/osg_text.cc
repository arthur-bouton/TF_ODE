#include "osg_text.hh"
#include <osg/Depth>


using namespace osg;


namespace renderer 
{

#define CASTS_SHADOW 0x1


OsgText::OsgText( ref_ptr<Group> root, int window_width, int window_height, int alignment ) :
         _root( root ), _width( window_width ), _height( window_height ), _callback( nullptr ), _margin( 0 )
{
	TextGeode = ref_ptr<Geode>( new Geode );
	TextGeode->setNodeMask( TextGeode->getNodeMask() & ~CASTS_SHADOW );
	TextGeode->getOrCreateStateSet()->setAttributeAndModes( ref_ptr<Depth>( new Depth ), osg::StateAttribute::OFF );

	TextProjectionMatrix = ref_ptr<Projection>( new Projection );
    TextProjectionMatrix->setMatrix( Matrix::ortho2D( 0, window_width, 0, window_height ) );

    // For the Text model view matrix use an identity matrix:
	TextModelViewMatrix = ref_ptr<MatrixTransform>( new MatrixTransform );
    TextModelViewMatrix->setMatrix( Matrix::identity() );

    // Make sure the model view matrix is not affected by any transforms
    // above it in the scene graph:
    TextModelViewMatrix->setReferenceFrame( Transform::ABSOLUTE_RF );

    // Add the Text projection matrix as a child of the root node
    // and the Text model view matrix as a child of the projection matrix
    // Anything under this node will be view using this projection matrix
    // and positioned with this model view matrix.
    root->addChild( TextProjectionMatrix );
    TextProjectionMatrix->addChild( TextModelViewMatrix );

    // Add the Geometry node to contain Text geometry as a child of the
    // Text model view matrix.
    TextModelViewMatrix->addChild( TextGeode );

    // Add the text ( Text class is derived from drawable ) to the geode:
	_text = osg::ref_ptr<osgText::Text>( new osgText::Text );
    TextGeode->addDrawable( _text );


    //_text->setAxisAlignment( osgText::Text::SCREEN );
	//_text->setCharacterSizeMode( osgText::Text::SCREEN_COORDS );

	set_alignment( alignment );
	set_pos( 0, 0 );
	set_size( 3 );

	_text->setFont( "/usr/share/fonts/truetype/noto/NotoMono-Regular.ttf" );
}


void OsgText::set_alignment( int alignment )
{
	_alignment = alignment;
	switch ( _alignment )
	{
		case 1 :
			_text->setAlignment( osgText::Text::LEFT_TOP );
			return;
		case 2 :
			_text->setAlignment( osgText::Text::RIGHT_TOP );
			return;
		case 3 :
			_text->setAlignment( osgText::Text::LEFT_BOTTOM );
			return;
		case 4 :
			_text->setAlignment( osgText::Text::RIGHT_BOTTOM );
			return;
	}
}


void OsgText::set_pos( float x, float y )
{
	_x = x;
	_y = y;
	if ( _y < 0 )
		switch ( _alignment )
		{
			case 1 :
				_y = _x*_width/_height;
				break;
			case 2 :
				_y = ( 100 - _x )*_width/_height;
				break;
			case 3 :
				_y = 100 - _x*_width/_height;
				break;
			case 4 :
				_y = 100 - ( 100 - _x )*_width/_height;
				break;
		}
	_text->setPosition( Vec3( _x/100*_width, ( 1 - _y/100 )*_height, 0 ) );
}


void OsgText::set_size( float size )
{	
	_size = size;
	_text->setCharacterSize( _size/100*_height );

	if ( _margin > 0 )
		_text->setBoundingBoxMargin( _margin*_size/100*_height );
}


void OsgText::add_background( float margin, float alpha, float r, float g, float b )
{
	_margin = margin;
	_text->setDrawMode( osgText::Text::DrawModeMask::FILLEDBOUNDINGBOX | osgText::Text::DrawModeMask::TEXT );
	_text->setBoundingBoxColor( osg::Vec4( r, g, b, alpha ) );
	_text->setBoundingBoxMargin( _margin*_size/100*_height );
}


OsgText::~OsgText()
{
	_root->removeChild( TextProjectionMatrix );
}


}
