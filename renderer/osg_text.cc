#include "osg_text.hh"
#include <osg/Depth>


using namespace osg;


namespace renderer 
{

#define CASTS_SHADOW 0x1


OsgText::OsgText( ref_ptr<Group> root, int window_width, int window_height ) :
         _root( root ), _width( window_width ), _height( window_height ), _callback( nullptr )
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
    // ( See figure "n" )
    TextModelViewMatrix->addChild( TextGeode );

    // Add the text ( Text class is derived from drawable ) to the geode:
	_text = osg::ref_ptr<osgText::Text>( new osgText::Text );
    TextGeode->addDrawable( _text );


    _text->setAxisAlignment( osgText::Text::SCREEN );

	_text->setFont( "/usr/share/fonts/truetype/noto/NotoMono-Regular.ttf" );
}


void OsgText::set_pos( int x, int y )
{
	_text->setPosition( Vec3( x, _height - y, 0 ) );
	_x = x;
	_y = y;
}


void OsgText::add_background( float margin, float alpha, float r, float g, float b )
{
	_text->setDrawMode( osgText::Text::DrawModeMask::FILLEDBOUNDINGBOX | osgText::Text::DrawModeMask::TEXT );
	_text->setBoundingBoxColor( osg::Vec4( r, g, b, alpha ) );
	_text->setBoundingBoxMargin( margin ); 
}


OsgText::~OsgText()
{
	_root->removeChild( TextProjectionMatrix );
}


}
